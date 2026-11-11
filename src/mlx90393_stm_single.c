/*
 * mlx90393_stm_single.c
 *
 * Autors: Jan Rusnak.
 * (c) 2026 AZTech.
 */

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <gentyp.h>
#include "sysconf.h"
#include "board.h"
#include <mmio.h>
#include "fmalloc.h"
#include "msgconf.h"
#include "criterr.h"
#include "hwerr.h"
#include "dlycnt.h"
#include "cmdln.h"
#include "gpio_hal.h"
#include "pinmux_hal.h"
#include "mlx90393.h"
#include "mlx90393_stm_single.h"

#if defined(MLX90393_SINGLE_MODE)
#if MLX90393_SENSOR_MODE == MLX90393_SINGLE_MODE

#define RESET_STM_TMO_MS 5000U
#define DLY_AFTER_INTR_US 140U
#if MLX90393_REPORT_FN == 1
#define MLX90393_REPORT_BUF_SIZE \
	sizeof("MLX90393 si [999]: X=-2147483.648 Y=-2147483.648 Z=-2147483.648 uT T=-2147483.648 C\n")
#endif

#ifndef MLX90393_MEASURE_PERIOD_MS
#error "MLX90393_MEASURE_PERIOD_MS must be defined in sysconf.h"
#endif
#if MLX90393_MEASURE_PERIOD_MS < 1U || MLX90393_MEASURE_PERIOD_MS > 1000U
#error "MLX90393_MEASURE_PERIOD_MS must be in the range 1..1000"
#endif
/* Single measurement uses temperature and all three magnetic axes. */
#define MLX90393_SINGLE_SEL_ZYXT 0x0FU
#ifndef MLX90393_OSR
#define MLX90393_OSR 0U
#endif
#ifndef MLX90393_DIG_FILT
#define MLX90393_DIG_FILT 4U
#endif
#define MLX90393_RANGE_5MT  5U
#define MLX90393_RANGE_25MT 25U
#define MLX90393_RANGE_50MT 50U
#ifndef MLX90393_RANGE_PROFILE
#define MLX90393_RANGE_PROFILE MLX90393_RANGE_5MT
#endif
#if MLX90393_RANGE_PROFILE == MLX90393_RANGE_5MT
#define MLX90393_GAIN_SEL 7U
#define MLX90393_RES_X 0U
#define MLX90393_RES_Y 0U
#define MLX90393_RES_Z 0U
#elif MLX90393_RANGE_PROFILE == MLX90393_RANGE_25MT
#define MLX90393_GAIN_SEL 0U
#define MLX90393_RES_X 0U
#define MLX90393_RES_Y 0U
#define MLX90393_RES_Z 0U
#elif MLX90393_RANGE_PROFILE == MLX90393_RANGE_50MT
#define MLX90393_GAIN_SEL 0U
#define MLX90393_RES_X 1U
#define MLX90393_RES_Y 1U
#define MLX90393_RES_Z 1U
#else
#error "Invalid MLX90393_RANGE_PROFILE (use MLX90393_RANGE_5MT/25MT/50MT)"
#endif
#ifndef MLX90393_OSR2
#define MLX90393_OSR2 0U
#endif
#ifndef MLX90393_TCMP_EN
#define MLX90393_TCMP_EN 0U
#endif
#define MLX90393_HALCONF 0xCU
#if MLX90393_OSR < 0 || MLX90393_OSR > 3U
#error "MLX90393_OSR must be in the range 0..3"
#endif
#if MLX90393_DIG_FILT < 0 || MLX90393_DIG_FILT > 7U
#error "MLX90393_DIG_FILT must be in the range 0..7"
#endif
#if MLX90393_OSR2 < 0 || MLX90393_OSR2 > 3U
#error "MLX90393_OSR2 must be in the range 0..3"
#endif
#if MLX90393_TCMP_EN != 0U
#error "MLX90393_SINGLE_MODE supports only MLX90393_TCMP_EN=0"
#endif
#ifndef MLX90393_HARDIRON_EN
#error "MLX90393_HARDIRON_EN must be defined in sysconf.h"
#endif
#if MLX90393_HARDIRON_EN != 0 && MLX90393_HARDIRON_EN != 1
#error "MLX90393_HARDIRON_EN must be 0 or 1"
#endif
#if MLX90393_HARDIRON_EN == 1
#if !defined(MLX90393_HARDIRON_X_NT) || !defined(MLX90393_HARDIRON_Y_NT) || \
    !defined(MLX90393_HARDIRON_Z_NT)
#error "MLX90393_HARDIRON_X/Y/Z_NT must be defined"
#endif
#if MLX90393_HARDIRON_X_NT < INT32_MIN || MLX90393_HARDIRON_X_NT > INT32_MAX || \
    MLX90393_HARDIRON_Y_NT < INT32_MIN || MLX90393_HARDIRON_Y_NT > INT32_MAX || \
    MLX90393_HARDIRON_Z_NT < INT32_MIN || MLX90393_HARDIRON_Z_NT > INT32_MAX
#error "MLX90393_HARDIRON_X/Y/Z_NT must fit int32_t"
#endif
#endif
#if (MLX90393_OSR == 0U && MLX90393_DIG_FILT <= 1U) || \
    (MLX90393_OSR == 1U && MLX90393_DIG_FILT == 0U)
#error "MLX90393_OSR/DIG_FILT combination is invalid for HALLCONF=0xC"
#endif

/* Nominal Single Measurement conversion time according to MLX90393 rev.012, Table 8. */
#define MLX90393_POW2(x)             (1U << (x))
#define MLX90393_TCONVM_US           (67U + 64U * MLX90393_POW2(MLX90393_OSR) * \
	(2U + MLX90393_POW2(MLX90393_DIG_FILT)))
#define MLX90393_TCONVT_US           (67U + 192U * MLX90393_POW2(MLX90393_OSR2))
#define MLX90393_SM_CONV_US          (220U + 360U + 3U * MLX90393_TCONVM_US + MLX90393_TCONVT_US + 100U)
#define MLX90393_SM_CONV_MS          ((MLX90393_SM_CONV_US + 999U) / 1000U)
#define MLX90393_WAIT_MARGIN_MS      20U
#define WAIT_ISR_SIG_TMO_MS          (MLX90393_SM_CONV_MS + MLX90393_WAIT_MARGIN_MS)
#define MLX90393_PERIOD_TICKS        ((MLX90393_MEASURE_PERIOD_MS * configTICK_RATE_HZ) / 1000U)
#if MLX90393_SM_CONV_US + DLY_AFTER_INTR_US >= MLX90393_MEASURE_PERIOD_MS * 1000U
#error "MLX90393_MEASURE_PERIOD_MS is too short for nominal single conversion plus DRDY delay"
#endif
/* The configured period must have an exact whole-tick representation. */
typedef char mlx90393_single_period_tick_check[
	((MLX90393_MEASURE_PERIOD_MS * configTICK_RATE_HZ) % 1000U == 0U &&
	 MLX90393_PERIOD_TICKS > 0U) ? 1 : -1];

static TaskHandle_t tsk_hndl;
static struct mlx90393_dsc mlx90393_dsc;
static mlx90393 mlx = &mlx90393_dsc;
static p_stf_t stmf;
static TickType_t last_wake_time;
static SemaphoreHandle_t isr_sig;
#if MLX90393_REPORT_FN == 1
static boolean_t (*report_clbk)(const char *);
static char *report_buf;
#endif

static struct {
	unsigned int sm_start_cnt;
	unsigned int sm_read_cnt;
	unsigned int missed_period_cnt;
	unsigned int wait_int_tmo_cnt;
	unsigned int stm_rst_cnt;
	unsigned int intr_pin_low_cnt;
#if MLX90393_REPORT_FN == 1
	unsigned int report_fn_err_cnt;
#endif
} stats;

static gfp_t state_sw_reset(void);
static gfp_t state_probe(void);
static gfp_t state_intr_cfg(void);
static gfp_t state_chip_cfg(void);
static gfp_t state_measure_start(void);
static gfp_t state_measure_cycle(void);
static gfp_t state_wait_intr(void);
static BaseType_t isr_clbk(uint32_t status);
static gfp_t state_error(void);
static void tsk(void *p);
static void report_data(const struct mlx90393_measure_data *data);
#if MLX90393_HARDIRON_EN == 1
static int32_t hardiron_correct_nt(int32_t measured_nt, int32_t offset_nt);
#endif
static void mlx90393_build_cfg(uint16_t *cfg0, uint16_t *cfg1, uint16_t *cfg2);
static void cmd_mlxs(void);

/**
 * init_mlx90393_stm_single
 */
void init_mlx90393_stm_single(void)
{
	mlx->spi.cfg.spi_bus_id = 0;
	mlx->spi.cfg.mode = SPI_HAL_MODE3;
	mlx->spi.cfg.bits_trans = SPI_HAL_8_BIT_TRANS;
	mlx->spi.cfg.dly_bct_ns = 0;
	mlx->spi.cfg.dly_bcs_ns = 10;
	mlx->spi.cfg.cs_rise = FALSE;
	mlx->spi.cfg.sck_hz = 10000000;
	mlx->spi.cfg.csel_num = SPI_HAL_CSEL1;
	mlx->spi.cfg.csel_cont = gpio_hal_get_ctrl(1);
	mlx->spi.cfg.csel_pin = (1U << 14);
	init_mlx90393(mlx);
#if MLX90393_REPORT_FN == 1
	if ((report_buf = pvPortMalloc(MLX90393_REPORT_BUF_SIZE)) == NULL) {
		crit_err_exit(MALLOC_ERROR);
	}
	memset(report_buf, 0, MLX90393_REPORT_BUF_SIZE);
#endif
	if ((isr_sig = xSemaphoreCreateBinary()) == NULL) {
		crit_err_exit(MALLOC_ERROR);
	}
	add_command_noargs("mlxs", cmd_mlxs);
	if (xTaskCreate(tsk, "MLX90393", MLX90393_TASK_STACK_SIZE, NULL,
	                MLX90393_TASK_PRIO, &tsk_hndl) != pdPASS) {
		crit_err_exit(MALLOC_ERROR);
	}
}

#if MLX90393_REPORT_FN == 1
/**
 * mlx90393_stm_set_report_fn
 */
void mlx90393_stm_set_report_fn(boolean_t (*report_fn)(const char *))
{
	report_clbk = report_fn;
}
#endif

/**
 * state_sw_reset
 */
static gfp_t state_sw_reset(void)
{
	int ret;

	ret = mlx90393_soft_reset(mlx);
	if (ret) {
		msg(INF, "MLX90393: Soft reset error (%s)\n", hwerr_str(ret));
		return ((gfp_t) state_error);
	}
	msg(INF, "MLX90393: Soft reset done (MLX90393_SINGLE_MODE)\n");
	return ((gfp_t) state_probe);
}

/**
 * state_probe
 */
static gfp_t state_probe(void)
{
	int ret;

	if ((ret = mlx90393_probe(mlx))) {
		msg(INF, "MLX90393: state_probe() (%s)\n", hwerr_str(ret));
		return ((gfp_t) state_error);
	}
	return ((gfp_t) state_intr_cfg);
}

/**
 * state_intr_cfg
 */
static gfp_t state_intr_cfg(void)
{
	pinmux_hal_set_func(MLX90393_INT_CONT, MLX90393_INT_PIN, PINMUX_HAL_FUNC_GPIO_IN);
	gpio_hal_set_pull(MLX90393_INT_CONT, MLX90393_INT_PIN, GPIO_HAL_PULL_NONE);
	gpio_hal_set_filter(MLX90393_INT_CONT, MLX90393_INT_PIN, GPIO_HAL_FILTER_NONE);
	gpio_hal_set_schmitt(MLX90393_INT_CONT, MLX90393_INT_PIN, TRUE);
	gpio_hal_intr_config(MLX90393_INT_CONT, MLX90393_INT_PIN, GPIO_HAL_INTR_LEVEL_HIGH);
	gpio_hal_intr_disable(MLX90393_INT_CONT, MLX90393_INT_PIN);
	if (!gpio_hal_isr_registered(MLX90393_INT_CONT, isr_clbk)) {
		gpio_hal_isr_register(MLX90393_INT_CONT, isr_clbk);
	}
	return ((gfp_t) state_chip_cfg);
}

/**
 * state_chip_cfg
 */
static gfp_t state_chip_cfg(void)
{
	uint16_t cfg0_w, cfg0, cfg1, cfg2;
	int ret;

	mlx90393_build_cfg(&cfg0, &cfg1, &cfg2);
	if ((ret = mlx90393_reg_read(mlx, MLX90393_CFG0_REG, &cfg0_w))) {
		goto err;
	}
	cfg0_w &= MLX90393_CFG0_ANA_RES_LOW_MSK;
	cfg0_w |= cfg0;
	if ((ret = mlx90393_reg_write(mlx, MLX90393_CFG0_REG, cfg0_w))) {
		goto err;
	}
	if ((ret = mlx90393_reg_write(mlx, MLX90393_CFG1_REG, cfg1))) {
		goto err;
	}
	if ((ret = mlx90393_reg_write(mlx, MLX90393_CFG2_REG, cfg2))) {
		goto err;
	}
	if ((ret = mlx90393_update_conv_cache(mlx))) {
		goto err;
	}
	return ((gfp_t) state_measure_start);
err:
	msg(INF, "MLX90393: chip_cfg() error (%s)\n", hwerr_str(ret));
	return ((gfp_t) state_error);
}

/**
 * state_measure_start
 */
static gfp_t state_measure_start(void)
{
	/* Schedule the first SM immediately; the following starts use absolute ticks. */
	last_wake_time = xTaskGetTickCount() - MLX90393_PERIOD_TICKS;
	return ((gfp_t) state_measure_cycle);
}

/**
 * state_measure_cycle
 */
static gfp_t state_measure_cycle(void)
{
	TickType_t elapsed = xTaskGetTickCount() - last_wake_time;
	int ret;

	if (elapsed > MLX90393_PERIOD_TICKS) {
		TickType_t missed = (elapsed - 1U) / MLX90393_PERIOD_TICKS;
		last_wake_time += missed * MLX90393_PERIOD_TICKS;
		stats.missed_period_cnt += missed;
	}
	vTaskDelayUntil(&last_wake_time, MLX90393_PERIOD_TICKS);
	gpio_hal_intr_disable(MLX90393_INT_CONT, MLX90393_INT_PIN);
	xSemaphoreTake(isr_sig, 0);
	if (gpio_hal_get_input(MLX90393_INT_CONT, MLX90393_INT_PIN) != GPIO_HAL_LOW) {
		msg(INF, "MLX90393: DRDY pin high before SM command\n");
		stats.intr_pin_low_cnt++;
		return ((gfp_t) state_error);
	}
	gpio_hal_intr_enable(MLX90393_INT_CONT, MLX90393_INT_PIN);
	ret = mlx90393_start_measure(mlx, MLX90393_MEASURE_MODE_SINGLE, MLX90393_SINGLE_SEL_ZYXT);
	if (ret) {
		gpio_hal_intr_disable(MLX90393_INT_CONT, MLX90393_INT_PIN);
		xSemaphoreTake(isr_sig, 0);
		msg(INF, "MLX90393: start_measure(SM) error (%s)\n", hwerr_str(ret));
		return ((gfp_t) state_error);
	}
	stats.sm_start_cnt++;
	return ((gfp_t) state_wait_intr);
}

/**
 * state_wait_intr
 */
static gfp_t state_wait_intr(void)
{
	struct mlx90393_measure_data data;
	int ret;

	if (xSemaphoreTake(isr_sig, ms_to_os_ticks(WAIT_ISR_SIG_TMO_MS)) == pdFALSE) {
		gpio_hal_intr_disable(MLX90393_INT_CONT, MLX90393_INT_PIN);
		xSemaphoreTake(isr_sig, 0);
		msg(INF, "MLX90393: wait_isr_tmo expired (SM)\n");
		stats.wait_int_tmo_cnt++;
		return ((gfp_t) state_error);
	}
	delay_us(DLY_AFTER_INTR_US);
	if (gpio_hal_get_input(MLX90393_INT_CONT, MLX90393_INT_PIN) != GPIO_HAL_HIGH) {
		msg(INF, "MLX90393: state_wait_intr() DRDY pin unset error\n");
		stats.intr_pin_low_cnt++;
		return ((gfp_t) state_error);
	}
	if ((ret = mlx90393_read_measure_data(mlx, MLX90393_SINGLE_SEL_ZYXT, &data))) {
		msg(INF, "MLX90393: read_measure_data() error (%s)\n", hwerr_str(ret));
		return ((gfp_t) state_error);
	}
	stats.sm_read_cnt++;
	report_data(&data);
	return ((gfp_t) state_measure_cycle);
}

/**
 * isr_clbk
 */
static BaseType_t isr_clbk(uint32_t status)
{
	BaseType_t tsk_wkn = pdFALSE;

	if (status & MLX90393_INT_PIN &&
	    gpio_hal_is_intr_enabled(MLX90393_INT_CONT, MLX90393_INT_PIN) &&
	    gpio_hal_get_input(MLX90393_INT_CONT, MLX90393_INT_PIN) == GPIO_HAL_HIGH) {
		gpio_hal_intr_disable(MLX90393_INT_CONT, MLX90393_INT_PIN);
		xSemaphoreGiveFromISR(isr_sig, &tsk_wkn);
	}
	return (tsk_wkn);
}

/**
 * state_error
 */
static gfp_t state_error(void)
{
	msg(INF, "MLX90393: resetting state machine after error\n");
	vTaskDelay(ms_to_os_ticks(RESET_STM_TMO_MS));
	stats.stm_rst_cnt++;
	return ((gfp_t) state_sw_reset);
}

/**
 * tsk
 */
static void tsk(void *p)
{
	stmf = state_sw_reset;
	while (TRUE) {
		stmf = (p_stf_t) (*stmf)();
	}
}

#if MLX90393_REPORT_SI == 1 || MLX90393_REPORT_FN == 1
/**
 * format_mag_nt
 */
static void format_mag_nt(char *buf, size_t size, int32_t nt)
{
	char sign = '+';

	if (nt == INT32_MIN) {
		snprintf(buf, size, "----.---");
		return;
	}
	if (nt < 0) {
		sign = '-';
		nt = -nt;
	}
	snprintf(buf, size, "%c%ld.%03ld", sign, (long) (nt / 1000), (long) (nt % 1000));
}
#endif

#if MLX90393_HARDIRON_EN == 1
/**
 * hardiron_correct_nt
 */
static int32_t hardiron_correct_nt(int32_t measured_nt, int32_t offset_nt)
{
	int64_t corrected = (int64_t) measured_nt - offset_nt;

	if (corrected <= INT32_MIN || corrected > INT32_MAX) {
		return (INT32_MIN);
	}
	return ((int32_t) corrected);
}
#endif

/**
 * report_data
 */
static void report_data(const struct mlx90393_measure_data *data)
{
	int32_t x_nt, y_nt, z_nt, t_mc;
#if (MLX90393_REPORT_SI == 1 || MLX90393_REPORT_FN == 1) && MLX90393_REPORT_MSG_NUM == 1
	static unsigned int msg_num;
#endif

	if (data->x_valid) {
		mlx90393_mag_raw_to_nt(mlx, MLX90393_MEASURE_X, data->x, &x_nt);
	} else {
		x_nt = INT32_MIN;
	}
	if (data->y_valid) {
		mlx90393_mag_raw_to_nt(mlx, MLX90393_MEASURE_Y, data->y, &y_nt);
	} else {
		y_nt = INT32_MIN;
	}
	if (data->z_valid) {
		mlx90393_mag_raw_to_nt(mlx, MLX90393_MEASURE_Z, data->z, &z_nt);
	} else {
		z_nt = INT32_MIN;
	}
	if (data->t_valid) {
		mlx90393_temp_raw_to_mc(data->t, &t_mc);
	} else {
		t_mc = INT32_MIN;
	}
#if MLX90393_HARDIRON_EN == 1
	if (data->x_valid) {
		x_nt = hardiron_correct_nt(x_nt, MLX90393_HARDIRON_X_NT);
	}
	if (data->y_valid) {
		y_nt = hardiron_correct_nt(y_nt, MLX90393_HARDIRON_Y_NT);
	}
	if (data->z_valid) {
		z_nt = hardiron_correct_nt(z_nt, MLX90393_HARDIRON_Z_NT);
	}
#endif
#if MLX90393_REPORT_FLIP_X == 1
	if (x_nt != INT32_MIN) {
		x_nt = -x_nt;
	}
#endif
#if MLX90393_REPORT_SI == 1 || MLX90393_REPORT_FN == 1
	enum {
		FIELD_BUF_M = 13,
		FIELD_BUF_T = 13
	};
	char xs[FIELD_BUF_M], ys[FIELD_BUF_M], zs[FIELD_BUF_M], ts[FIELD_BUF_T];
	format_mag_nt(xs, sizeof(xs), x_nt);
	format_mag_nt(ys, sizeof(ys), y_nt);
	format_mag_nt(zs, sizeof(zs), z_nt);
	if (t_mc == INT32_MIN) {
		snprintf(ts, sizeof(ts), "----.---");
	} else {
		char sign = '+';
		int32_t t_abs = t_mc;
		int32_t t_c;
		int32_t t_frac;

		if (t_mc < 0) {
			sign = '-';
			t_abs = -t_mc;
		}
		t_c = t_abs / 1000;
		t_frac = t_abs % 1000;
		snprintf(ts, sizeof(ts), "%c%ld.%03ld", sign, (long) t_c, (long) t_frac);
	}
#if MLX90393_REPORT_FN == 1
#if MLX90393_REPORT_MSG_NUM == 1
	snprintf(report_buf, MLX90393_REPORT_BUF_SIZE,
		"MLX90393 si [%03u]: X=%s Y=%s Z=%s uT T=%s C\n", msg_num, xs, ys, zs, ts);
#else
	snprintf(report_buf, MLX90393_REPORT_BUF_SIZE, "MLX90393 si: X=%s Y=%s Z=%s uT T=%s C\n", xs, ys, zs, ts);
#endif
#if MLX90393_REPORT_SI == 1
	msg(INF, "%s", report_buf);
#endif
	if (report_clbk && !report_clbk(report_buf)) {
		stats.report_fn_err_cnt++;
	}
#else
#if MLX90393_REPORT_MSG_NUM == 1
	msg(INF, "MLX90393 si [%03u]: X=%s Y=%s Z=%s uT T=%s C\n", msg_num, xs, ys, zs, ts);
#else
	msg(INF, "MLX90393 si: X=%s Y=%s Z=%s uT T=%s C\n", xs, ys, zs, ts);
#endif
#endif
#if MLX90393_REPORT_MSG_NUM == 1
	if (++msg_num > 999) {
		msg_num = 0;
	}
#endif
#endif
}

/**
 * mlx90393_build_cfg
 */
static void mlx90393_build_cfg(uint16_t *cfg0, uint16_t *cfg1, uint16_t *cfg2)
{
	*cfg0 = mlx90393_cfg0_hallconf_s(MLX90393_HALCONF) | mlx90393_cfg0_gain_sel_s(MLX90393_GAIN_SEL) |
		mlx90393_cfg0_z_series_s(0U) | mlx90393_cfg0_bist_s(0U);
	*cfg1 = mlx90393_cfg1_bdr_s(0U) | mlx90393_cfg1_burst_sel_zyxt_s(MLX90393_SINGLE_SEL_ZYXT) |
		mlx90393_cfg1_tcmp_en_s(MLX90393_TCMP_EN) | mlx90393_cfg1_ext_trg_s(0U) |
		mlx90393_cfg1_woc_diff_s(0U) | mlx90393_cfg1_comm_mode_s(2U) |
		mlx90393_cfg1_trig_int_s(1U);
	*cfg2 = mlx90393_cfg2_osr_s(MLX90393_OSR) | mlx90393_cfg2_dig_filt_s(MLX90393_DIG_FILT) |
		mlx90393_cfg2_res_x_s(MLX90393_RES_X) | mlx90393_cfg2_res_y_s(MLX90393_RES_Y) |
		mlx90393_cfg2_res_z_s(MLX90393_RES_Z) | mlx90393_cfg2_osr2_s(MLX90393_OSR2);
}

/**
 * cmd_mlxs
 */
static void cmd_mlxs(void)
{
	UBaseType_t pr;

	pr = uxTaskPriorityGet(NULL);
	vTaskPrioritySet(NULL, TASK_PRIO_HIGH);
	msg(INF, cmd_accp);
	msg(INF, "MLX90393 SM cnt: start=%u read=%u missed_period=%u wait_int_tmo=%u intr_pin_low=%u\n",
	    stats.sm_start_cnt, stats.sm_read_cnt, stats.missed_period_cnt,
	    stats.wait_int_tmo_cnt, stats.intr_pin_low_cnt);
#if MLX90393_REPORT_FN == 1
	msg(INF, "MLX90393 cnt: stm_rst=%u report_fn_err=%u\n",
	    stats.stm_rst_cnt, stats.report_fn_err_cnt);
#else
	msg(INF, "MLX90393 cnt: stm_rst=%u\n", stats.stm_rst_cnt);
#endif
	vTaskPrioritySet(NULL, pr);
}
#endif
#endif
