/*
 * mlx90393.c
 *
 * Copyright (c) 2025 Jan Rusnak <jan@rusnak.sk>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <FreeRTOS.h>
#include <task.h>
#include <gentyp.h>
#include <string.h>
#include "sysconf.h"
#include "criterr.h"
#include "dlycnt.h"
#include "hwerr.h"
#include "mlx90393.h"

#define SPI_HAL_SHORT_TRANS SPI_HAL_POLL
#define SPI_HAL_LONG_TRANS SPI_HAL_DMA
#define RESET_DELAY_MS 5U
#define CDM_HS_DELAY_MS 15U
#define CDM_EX_DELAY_US 1500
#define ID_CHECK_VAL 0xABCD

#define STAT_BURST_MODE (1 << 7)
#define STAT_WOC_MODE (1 << 6)
#define STAT_SM_MODE (1 << 5)
#define STAT_ERROR (1 << 4)
#define STAT_SED (1 << 3)
#define STAT_RS (1 << 2)
#define STAT_D1 (1 << 1)
#define STAT_D0 (1 << 0)

enum mlx90393_cmd {
	MLX90393_CMD_NOP = 0x00,
	MLX90393_CMD_SB  = 0x10,
	MLX90393_CMD_SW  = 0x20,
	MLX90393_CMD_SM  = 0x30,
	MLX90393_CMD_RM  = 0x40,
	MLX90393_CMD_RR  = 0x50,
	MLX90393_CMD_WR  = 0x60,
	MLX90393_CMD_EX  = 0x80,
	MLX90393_CMD_HR  = 0xD0,
	MLX90393_CMD_HS  = 0xE0,
	MLX90393_CMD_RT  = 0xF0
};

// Sensitivity derived from datasheet Tables 17 and 18 for HALLCONF=0xC.
// TCMP_EN=0 uses the T=25C table.
// TCMP_EN=1 uses the T=35C table because the compensation pivot TREF
// corresponds to 35C and the compensation correction is unity at TREF.
// Values are stored as integer nT/LSB.
static const uint16_t sens_xy_25c_nT_per_lsb_tbl[8][4] = {
	/* RES:       0     1     2     3 */
	/* G0 */ {  751, 1502, 3004, 6009 },
	/* G1 */ {  601, 1202, 2403, 4840 },
	/* G2 */ {  451,  901, 1803, 3605 },
	/* G3 */ {  376,  751, 1502, 3004 },
	/* G4 */ {  300,  601, 1202, 2403 },
	/* G5 */ {  250,  501, 1001, 2003 },
	/* G6 */ {  200,  401,  801, 1602 },
	/* G7 */ {  150,  300,  601, 1202 },
};

static const uint16_t sens_z_25c_nT_per_lsb_tbl[8][4] = {
	/* RES:       0     1     2     3 */
	/* G0 */ { 1210, 2420, 4840, 9680 },
	/* G1 */ {  968, 1936, 3872, 7744 },
	/* G2 */ {  726, 1452, 2904, 5808 },
	/* G3 */ {  605, 1210, 2420, 4840 },
	/* G4 */ {  484,  968, 1936, 3872 },
	/* G5 */ {  403,  807, 1613, 3227 },
	/* G6 */ {  323,  645, 1291, 2581 },
	/* G7 */ {  242,  484,  968, 1936 },
};

static const uint16_t sens_xy_35c_nT_per_lsb_tbl[8][4] = {
	/* RES:       0     1     2     3 */
	/* G0 */ {  787, 1573, 3146, 6292 },
	/* G1 */ {  629, 1258, 2517, 5034 },
	/* G2 */ {  472,  944, 1888, 3775 },
	/* G3 */ {  393,  787, 1573, 3146 },
	/* G4 */ {  315,  629, 1258, 2517 },
	/* G5 */ {  262,  524, 1049, 2097 },
	/* G6 */ {  210,  419,  839, 1678 },
	/* G7 */ {  157,  315,  629, 1258 },
};

static const uint16_t sens_z_35c_nT_per_lsb_tbl[8][4] = {
	/* RES:       0     1     2     3 */
	/* G0 */ { 1267, 2534, 5068, 10137 },
	/* G1 */ { 1014, 2027, 4055,  8109 },
	/* G2 */ {  760, 1521, 3041,  6082 },
	/* G3 */ {  634, 1267, 2534,  5068 },
	/* G4 */ {  507, 1014, 2027,  4055 },
	/* G5 */ {  422,  845, 1689,  3379 },
	/* G6 */ {  338,  676, 1352,  2703 },
	/* G7 */ {  253,  507, 1014,  2027 },
};

static int b1_cmd(mlx90393 mlx, enum mlx90393_cmd cmd);
static void mag_format(uint8_t res, uint8_t tcmp_en, boolean_t *is_twos, int *zero_lsb);
static int32_t mag_raw_to_lsb(uint16_t raw, boolean_t is_twos, int zero_lsb);

/**
 * init_mlx90393
 */
void init_mlx90393(mlx90393 mlx)
{
	spi_hal_dev_init(&mlx->spi);
	mlx->conv_cache_valid = FALSE;
}

/**
 * mlx90393_start_measure
 */
int mlx90393_start_measure(mlx90393 mlx, enum mlx90393_measure_mode mode, unsigned int measure)
{
	uint8_t cmd, stat = 0;
	int ret = 0;

	switch (mode) {
	case MLX90393_MEASURE_MODE_BURST :
		cmd = MLX90393_CMD_SB;
		break;
	case MLX90393_MEASURE_MODE_WAKE :
		cmd = MLX90393_CMD_SW;
		break;
	case MLX90393_MEASURE_MODE_SINGLE :
		cmd = MLX90393_CMD_SM;
		break;
	default :
		crit_err_exit(BAD_PARAMETER);
		break;
	}
	cmd |= measure & 0x0F;
	if ((ret = spi_hal_xfer(&mlx->spi, SPI_HAL_SHORT_TRANS, &cmd, 1, &stat, 1))) {
		return (ret);
	}
	if (stat & STAT_ERROR) {
		return (-EHW);
	}
	switch (mode) {
	case MLX90393_MEASURE_MODE_BURST :
		if (!(stat & STAT_BURST_MODE)) {
			return (-EHW);
		}
		break;
	case MLX90393_MEASURE_MODE_WAKE :
		if (!(stat & STAT_WOC_MODE)) {
			return (-EHW);
		}
		break;
	case MLX90393_MEASURE_MODE_SINGLE :
		if (!(stat & STAT_SM_MODE)) {
			return (-EHW);
		}
		break;
	}
	return (ret);
}

/**
 * mlx90393_read_measure_data
 */
int mlx90393_read_measure_data(mlx90393 mlx, unsigned int measure, struct mlx90393_measure_data *data)
{
	uint8_t cmd, dary[9];
	int n;
	int ret = 0;
	int idx;

	memset(dary, 0, sizeof(dary));
	memset(data, 0, sizeof(*data));
	cmd = MLX90393_CMD_RM | (measure & 0x0F);
	n = 1;
	if (measure & MLX90393_MEASURE_T) {
		n += 2;
	}
	if (measure & MLX90393_MEASURE_X) {
		n += 2;
	}
	if (measure & MLX90393_MEASURE_Y) {
		n += 2;
	}
	if (measure & MLX90393_MEASURE_Z) {
		n += 2;
	}
	if ((ret = spi_hal_xfer(&mlx->spi, SPI_HAL_LONG_TRANS, &cmd, 1, dary, n))) {
		return (ret);
	}
	if (dary[0] & STAT_ERROR) {
		return (-EHW);
	}
	{
		int comps = 0;
		int d = dary[0] & (STAT_D1 | STAT_D0);
		int exp_data_bytes;
		if (measure & MLX90393_MEASURE_T) {
			comps++;
		}
		if (measure & MLX90393_MEASURE_X) {
			comps++;
		}
		if (measure & MLX90393_MEASURE_Y) {
			comps++;
		}
		if (measure & MLX90393_MEASURE_Z) {
			comps++;
		}
		exp_data_bytes = 2 * comps;
		if (exp_data_bytes && (2 * d + 2) != exp_data_bytes) {
			return (-EHW);
		}
	}
	idx = 1;
	if (measure & MLX90393_MEASURE_T) {
		data->t = (dary[idx] << 8) | dary[idx + 1];
		data->t_valid = 1;
		idx += 2;
	}
	if (measure & MLX90393_MEASURE_X) {
		data->x = (dary[idx] << 8) | dary[idx + 1];
		data->x_valid = 1;
		idx += 2;
	}
	if (measure & MLX90393_MEASURE_Y) {
		data->y = (dary[idx] << 8) | dary[idx + 1];
		data->y_valid = 1;
		idx += 2;
	}
	if (measure & MLX90393_MEASURE_Z) {
		data->z = (dary[idx] << 8) | dary[idx + 1];
		data->z_valid = 1;
		idx += 2;
	}
	return (ret);
}

/**
 * mlx90393_exit_mode
 */
int mlx90393_exit_mode(mlx90393 mlx)
{
	int ret = b1_cmd(mlx, MLX90393_CMD_EX);

	if (ret) {
		return (ret);
	}
	delay_us(CDM_EX_DELAY_US);
	return (0);
}

/**
 * mlx90393_memory_recall
 */
int mlx90393_memory_recall(mlx90393 mlx)
{
	mlx->conv_cache_valid = FALSE;
	return (b1_cmd(mlx, MLX90393_CMD_HR));
}

/**
 * mlx90393_memory_store
 */
int mlx90393_memory_store(mlx90393 mlx)
{
	int ret = b1_cmd(mlx, MLX90393_CMD_HS);

	if (ret) {
		return (ret);
	}
	/* Round up to whole ticks, then add one tick for the phase of vTaskDelay(). */
	vTaskDelay(((CDM_HS_DELAY_MS * configTICK_RATE_HZ + 999U) / 1000U) + 1U);
	return (0);
}

/**
 * mlx90393_reg_read
 */
int mlx90393_reg_read(mlx90393 mlx, enum mlx90393_reg_adr adr, uint16_t *data)
{
	uint8_t buf[5];
	int ret = 0;
	int d;

	buf[0] = MLX90393_CMD_RR;
	buf[1] = (adr & 0x3F) << 2;
	buf[2] = 0x00;
	buf[3] = 0x00;
	buf[4] = 0x00;
	if ((ret = spi_hal_xfer(&mlx->spi, SPI_HAL_LONG_TRANS, buf, 2, buf + 2, 3))) {
		return (ret);
	}
	if (buf[2] & STAT_ERROR) {
		return (-EHW);
	}
	d = buf[2] & (STAT_D1 | STAT_D0);
	if ((2 * d + 2) != 2) {
		return (-EHW);
	}
	*data = (buf[3] << 8) | buf[4];
	return (ret);
}

/**
 * mlx90393_reg_write
 */
int mlx90393_reg_write(mlx90393 mlx, enum mlx90393_reg_adr adr, uint16_t data)
{
	uint8_t tx[4], stat = 0;
	int ret = 0;

	tx[0] = MLX90393_CMD_WR;
	tx[1] = data >> 8;
	tx[2] = data & 0xFF;
	tx[3] = (adr & 0x3F) << 2;
	if ((ret = spi_hal_xfer(&mlx->spi, SPI_HAL_LONG_TRANS, tx, sizeof(tx), &stat, 1))) {
		return (ret);
	}
	if (stat & STAT_ERROR) {
		return (-EHW);
	}
	return (ret);
}

/**
 * mlx90393_probe
 */
int mlx90393_probe(mlx90393 mlx)
{
	uint16_t id, old;
	int ret = 0;

	if ((ret = mlx90393_reg_read(mlx, MLX90393_USER1_REG, &old))) {
		return (ret);
	}
	id = ID_CHECK_VAL;
	if ((ret = mlx90393_reg_write(mlx, MLX90393_USER1_REG, id))) {
		return (ret);
	}
	id = 0;
	if ((ret = mlx90393_reg_read(mlx, MLX90393_USER1_REG, &id))) {
		return (ret);
	}
	if (id != ID_CHECK_VAL) {
		return (-EHW);
	}
	if ((ret = mlx90393_reg_write(mlx, MLX90393_USER1_REG, old))) {
		return (ret);
	}
	return (ret);
}

/**
 * mlx90393_soft_reset
 */
int mlx90393_soft_reset(mlx90393 mlx)
{
	uint8_t cmd, stat = 0;
	int ret = 0;
	boolean_t was_rs = FALSE;

	mlx->conv_cache_valid = FALSE;
	cmd = MLX90393_CMD_EX;
	if ((ret = spi_hal_xfer(&mlx->spi, SPI_HAL_SHORT_TRANS, &cmd, 1, &stat, 1))) {
		return (ret);
	}
	delay_us(CDM_EX_DELAY_US);
	cmd = MLX90393_CMD_RT;
	stat = 0;
	if ((ret = spi_hal_xfer(&mlx->spi, SPI_HAL_SHORT_TRANS, &cmd, 1, &stat, 1))) {
		return (ret);
	}
	if (stat & STAT_ERROR) {
		return (-EHW);
	}
	if (stat & STAT_RS) {
		was_rs = TRUE;
	}
	/* Round up to whole ticks, then add one tick for the phase of vTaskDelay(). */
	vTaskDelay(((RESET_DELAY_MS * configTICK_RATE_HZ + 999U) / 1000U) + 1U);
	cmd = MLX90393_CMD_EX;
	stat = 0;
	if ((ret = spi_hal_xfer(&mlx->spi, SPI_HAL_SHORT_TRANS, &cmd, 1, &stat, 1))) {
		return (ret);
	}
	if (stat & STAT_ERROR) {
		return (-EHW);
	}
	if (stat & STAT_RS) {
		was_rs = TRUE;
	}
	if (!was_rs) {
		return (-EHW);
	}
	return (ret);
}

/**
 * mlx90393_update_conv_cache
 */
int mlx90393_update_conv_cache(mlx90393 mlx)
{
	uint16_t cfg0, cfg1, cfg2;
	int ret = 0;

	mlx->conv_cache_valid = FALSE;
	if ((ret = mlx90393_reg_read(mlx, MLX90393_CFG0_REG, &cfg0))) {
		return (ret);
	}
	if ((ret = mlx90393_reg_read(mlx, MLX90393_CFG1_REG, &cfg1))) {
		return (ret);
	}
	if ((ret = mlx90393_reg_read(mlx, MLX90393_CFG2_REG, &cfg2))) {
		return (ret);
	}
	mlx->cfg0 = cfg0;
	mlx->cfg1 = cfg1;
	mlx->cfg2 = cfg2;
	mlx->hallconf = mlx90393_cfg0_hallconf_r(cfg0);
	mlx->gain_sel = mlx90393_cfg0_gain_sel_r(cfg0);
	mlx->tcmp_en = mlx90393_cfg1_tcmp_en_r(cfg1);

	mlx->osr = mlx90393_cfg2_osr_r(cfg2);
	mlx->dig_filt = mlx90393_cfg2_dig_filt_r(cfg2);
	mlx->osr2 = mlx90393_cfg2_osr2_r(cfg2);
	mlx->res_x = mlx90393_cfg2_res_x_r(cfg2);
	mlx->res_y = mlx90393_cfg2_res_y_r(cfg2);
	mlx->res_z = mlx90393_cfg2_res_z_r(cfg2);
	/* Select the sensitivity reference according to TCMP_EN. */
	{
		const uint16_t (*sens_xy_tbl)[4];
		const uint16_t (*sens_z_tbl)[4];
		if (mlx->tcmp_en) {
			sens_xy_tbl = sens_xy_35c_nT_per_lsb_tbl;
			sens_z_tbl = sens_z_35c_nT_per_lsb_tbl;
		} else {
			sens_xy_tbl = sens_xy_25c_nT_per_lsb_tbl;
			sens_z_tbl = sens_z_25c_nT_per_lsb_tbl;
		}
		int sx = sens_xy_tbl[mlx->gain_sel][mlx->res_x];
		int sy = sens_xy_tbl[mlx->gain_sel][mlx->res_y];
		int sz = sens_z_tbl[mlx->gain_sel][mlx->res_z];
		/* HALLCONF=0x0 scaling: *98/75 (integer rounding). */
		if (mlx->hallconf == 0x0) {
			sx = (sx * 98 + 37) / 75;
			sy = (sy * 98 + 37) / 75;
			sz = (sz * 98 + 37) / 75;
		}
		mlx->sens_x_nT_per_lsb = sx;
		mlx->sens_y_nT_per_lsb = sy;
		mlx->sens_z_nT_per_lsb = sz;
	}
	/* Raw format helpers from Table 21. */
	mag_format(mlx->res_x, mlx->tcmp_en, &mlx->x_is_twos_complement, &mlx->x_zero_lsb);
	mag_format(mlx->res_y, mlx->tcmp_en, &mlx->y_is_twos_complement, &mlx->y_zero_lsb);
	mag_format(mlx->res_z, mlx->tcmp_en, &mlx->z_is_twos_complement, &mlx->z_zero_lsb);
	mlx->conv_cache_valid = TRUE;
	return (ret);
}

/**
 * mlx90393_mag_raw_to_nt
 */
void mlx90393_mag_raw_to_nt(mlx90393 mlx, unsigned int axis, uint16_t raw, int32_t *out_nt)
{
	int sens;
	boolean_t twos;
	int zero;

	if (!mlx->conv_cache_valid) {
		crit_err_exit(BAD_PARAMETER);
	}
	switch (axis) {
	case MLX90393_MEASURE_X :
		sens = mlx->sens_x_nT_per_lsb;
		twos = mlx->x_is_twos_complement;
		zero = mlx->x_zero_lsb;
		break;
	case MLX90393_MEASURE_Y :
		sens = mlx->sens_y_nT_per_lsb;
		twos = mlx->y_is_twos_complement;
		zero = mlx->y_zero_lsb;
		break;
	case MLX90393_MEASURE_Z :
		sens = mlx->sens_z_nT_per_lsb;
		twos = mlx->z_is_twos_complement;
		zero = mlx->z_zero_lsb;
		break;
	default :
		crit_err_exit(BAD_PARAMETER);
		return;
	}
	int32_t lsb = mag_raw_to_lsb(raw, twos, zero);
	*out_nt = lsb * (int32_t) sens;
}

/**
 * mlx90393_mag_raw_to_ut
 */
void mlx90393_mag_raw_to_ut(mlx90393 mlx, unsigned int axis, uint16_t raw, int32_t *out_ut)
{
	int32_t nt = 0;

	mlx90393_mag_raw_to_nt(mlx, axis, raw, &nt);
	/* rounded nt -> ut */
	if (nt >= 0) {
		*out_ut = (nt + 500) / 1000;
	} else {
		*out_ut = (nt - 500) / 1000;
	}
}

/**
 * mlx90393_temp_raw_to_mc
 */
void mlx90393_temp_raw_to_mc(uint16_t raw_t, int32_t *out_mc)
{
	/* Datasheet: TRES=45.2 LSB/C, T25=46244 LSB (unsigned) */
	int32_t delta = (int32_t) raw_t - 46244;
	/* delta_mc = delta * 1000 / 45.2 = delta * 10000 / 452 */
	int64_t num = (int64_t) delta * 10000;

	if (num >= 0) {
		num += 226; /* rounding */
	} else {
		num -= 226;
	}
	int32_t delta_mc = num / 452;
	*out_mc = 25000 + delta_mc;
}

/**
 * b1_cmd
 */
static int b1_cmd(mlx90393 mlx, enum mlx90393_cmd cmd)
{
	uint8_t cmd_b = cmd, stat = 0;
	int ret = 0;

	if ((ret = spi_hal_xfer(&mlx->spi, SPI_HAL_SHORT_TRANS, &cmd_b, 1, &stat, 1))) {
		return (ret);
	}
	if (stat & STAT_ERROR) {
		return (-EHW);
	}
	return (ret);
}

/**
 * mag_format
 */
static void mag_format(uint8_t res, uint8_t tcmp_en, boolean_t *is_twos, int *zero_lsb)
{
	/* Datasheet Table 21 */
	if ((tcmp_en == 1) && (res > 1)) {
		/* N/A combination. */
		crit_err_exit(BAD_PARAMETER);
	}
	if ((res <= 1) && (tcmp_en == 0)) {
		*is_twos = TRUE;
		*zero_lsb = 0;
	} else {
		*is_twos = FALSE;
		if (res == 3) {
			/* RES=3, TCMP_EN=0: unsigned, 0uT at 2^14 (0x4000). */
			*zero_lsb = (1 << 14);
		} else {
			/* Unsigned: 0uT at 2^15 (0x8000). */
			*zero_lsb = (1 << 15);
		}
	}
}

/**
 * mag_raw_to_lsb
 */
static int32_t mag_raw_to_lsb(uint16_t raw, boolean_t is_twos, int zero_lsb)
{
	if (is_twos) {
		return ((int16_t) raw);
	}
	/* Unsigned encoding around 0uT point, allow wraparound to represent negatives */
	int32_t v = (int32_t) raw - (int32_t) zero_lsb;
	if (v > 32767) {
		v -= 65536;
	} else if (v < -32768) {
		v += 65536;
	}
	return (v);
}
