/*
 * mlx90393.h
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

/**
 * @file mlx90393.h
 *
 * @brief MLX90393 driver interface and raw-to-physical-unit conversion model.
 *
 * The MLX90393 returns measurement samples (X/Y/Z magnetic and T temperature) as 16-bit words.
 * The driver stores these samples in @c uint16_t (see @ref mlx90393_measure_data) exactly as read
 * from the device. SPI communication follows the MLX90393 command/status protocol; this module
 * does not configure the measurement schedule or use the DRDY interrupt itself. The conversion
 * helpers interpret raw words and scale them into integer physical units:
 *   - Magnetic field: nT (nanoTesla) or uT (microTesla).
 *   - Temperature: mC (milliCelsius).
 *
 * @section mlx90393_conv_cache Conversion cache and what affects scaling.
 * Magnetic conversion depends on the current sensor configuration. The driver therefore caches
 * the relevant configuration and derived conversion parameters in the descriptor
 * (see @ref mlx90393_dsc). Call @ref mlx90393_update_conv_cache() after configuring the
 * sensor and whenever the scaling-relevant configuration changes. A successful register write
 * does not automatically invalidate or refresh the cache. Initialization, soft reset and memory
 * recall invalidate the cache. Magnetic conversion requires a valid cache.
 *
 * Cached parameters that affect magnetic conversion:
 *   - @c GAIN_SEL and @c RES_X/Y/Z select sensitivity (scale) in uT/LSB (different for XY vs Z).
 *     The driver derives an integer sensitivity in nT/LSB from the datasheet sensitivity tables
 *     for HALLCONF=0xC. With @c TCMP_EN disabled, the 25C reference table is used. With
 *     @c TCMP_EN enabled, the 35C reference table is used because the temperature-compensation
 *     pivot TREF corresponds to 35C. For @c HALLCONF=0x0 the datasheet specifies a sensitivity
 *     scaling factor 98/75, which the driver applies when computing the cache. Other HALLCONF
 *     values are unsupported and fatal via @c crit_err_exit(BAD_PARAMETER).
 *   - @c TCMP_EN affects the **output coding** (2's complement vs unsigned with a 0uT bias point)
 *     and enables on-chip sensitivity drift compensation. The driver caches per-axis raw format
 *     helpers:
 *       - @c *_is_twos_complement : whether raw words are 2's complement.
 *       - @c *_zero_lsb           : the raw code corresponding to 0uT when unsigned-coded.
 *
 * Note: @c OSR / @c DIG_FILT influence conversion time and noise, but not the magnetic scale factor.
 * The conversion helpers use typical datasheet sensitivity, not per-device calibration.
 *
 * @section mlx90393_mag_conv Magnetic conversion (raw -> nT / uT).
 * The conversion is performed in two steps:
 *
 * 1) Convert raw @c uint16_t into a signed "LSB" code:
 *    - If the axis is 2's complement coded:
 *        @code
 *        lsb = (int16_t) raw;
 *        @endcode
 *    - If the axis is unsigned-coded with a 0uT bias point:
 *        @code
 *        diff = (int32_t) raw - zero_lsb;
 *        // wrap diff to the signed 16-bit range to handle negative values in modular arithmetic.
 *        if (diff >  32767) diff -= 65536;
 *        if (diff < -32768) diff += 65536;
 *        lsb = diff;
 *        @endcode
 *
 * 2) Scale the signed LSB code into field units:
 *    - nT:
 *        @code
 *        out_nt = lsb * sens_nt_per_lsb;
 *        @endcode
 *    - uT:
 *        @code
 *        out_ut = round(out_nt / 1000.0);
 *        @endcode
 *    The driver implementation uses integer arithmetic with rounding to the nearest integer.
 *
 * @section mlx90393_temp_conv Temperature conversion (raw -> mC)
 * Temperature conversion is based on the datasheet constants:
 *   - @c T25  : raw code at 25C
 *   - @c TRES : temperature resolution in LSB/C
 *
 * The driver computes:
 *   @code
 *   // delta in raw codes from 25C reference
 *   d = (int32_t) raw_t - T25;
 *
 *   // T in mC, using integer math (TRES = 45.2 LSB/C => 452/10)
 *   out_mC = 25000 + round(d * 10000 / 452);
 *   @endcode
 *
 * @section mlx90393_driver_context Execution context
 * All SPI operations are synchronous through @c spi_hal_xfer(). The driver may also perform
 * microsecond delays or FreeRTOS task delays. It is intended for initialization/task context,
 * not for ISR use. The driver does not serialize access by multiple tasks to one sensor handle;
 * the caller must prevent overlapping commands and configuration updates.
 *
 * Reference: Melexis MLX90393 datasheet, revision 012 (18 November 2025), sections 11-16.
 *
 * @section mlx90393_conv_notes Practical notes.
 * - Sensitivities used are typical table values: the 25C reference with @c TCMP_EN disabled
 *   and the 35C compensation-pivot reference with @c TCMP_EN enabled. Absolute accuracy depends
 *   on device tolerances, temperature drift, and any user calibration/offset strategy.
 * - When @c TCMP_EN is enabled, the sensor performs on-chip sensitivity drift compensation; the
 *   output coding and 0uT bias point depend on @c RES as documented in the datasheet.
 */

#ifndef MLX90393_H
#define MLX90393_H

#include <stdint.h>
#include <gentyp.h>
#include "spi_hal.h"

/**
 * @name Measurement channel selection bits
 * @brief Bitmask used to select which channels are included in measurement and readout commands.
 *
 * These bits are passed to @ref mlx90393_start_measure() and @ref mlx90393_read_measure_data()
 * to request temperature and/or magnetic axes.
 * @{
 */
#define MLX90393_MEASURE_T (1 << 0) /**< Temperature channel. */
#define MLX90393_MEASURE_X (1 << 1) /**< X-axis magnetic field channel. */
#define MLX90393_MEASURE_Y (1 << 2) /**< Y-axis magnetic field channel. */
#define MLX90393_MEASURE_Z (1 << 3) /**< Z-axis magnetic field channel. */
/** @} */

/**
 * @enum mlx90393_measure_mode
 * @brief Measurement command mode selector.
 *
 * Selects which measurement command is issued by @ref mlx90393_start_measure().
 */
enum mlx90393_measure_mode {
	/**
	 * @brief Burst mode measurement (SB command).
	 *
	 * Starts repeated burst acquisitions (SB). The command channel mask
	 * selects the converted quantities; a zero mask selects BURST_SEL in
	 * sensor RAM. BURST_DATA_RATE (BDR) controls the acquisition interval:
	 * BDR=0 runs without a programmed inactive interval, while BDR=1..63
	 * inserts an interval in steps of 20 ms between burst cycles.
	 */
	MLX90393_MEASURE_MODE_BURST,
	/**
	 * @brief Wake-on-change / wake measurement (SW command).
	 *
	 * Enters wake-up-on-change mode (SW). Channels and wake thresholds are
	 * determined by the command mask and sensor register configuration.
	 */
	MLX90393_MEASURE_MODE_WAKE,
	/**
	 * @brief Single measurement (SM command).
	 *
	 * Starts a single acquisition for the channels requested by the measurement
	 * mask passed to @ref mlx90393_start_measure().
	 */
	MLX90393_MEASURE_MODE_SINGLE
};

/**
 * @enum mlx90393_reg_adr
 * @brief Register address selector.
 *
 * Logical register identifiers used by @ref mlx90393_reg_read() and
 * @ref mlx90393_reg_write().
 */
enum mlx90393_reg_adr {
	/** @brief Configuration register 0 (basic measurement configuration). */
	MLX90393_CFG0_REG,
	/** @brief Configuration register 1 (advanced measurement settings). */
	MLX90393_CFG1_REG,
	/** @brief Configuration register 2 (OSR, DIG_FILT, RES_X/Y/Z and OSR2). */
	MLX90393_CFG2_REG,
	/** @brief Temperature sensitivity compensation coefficients SENS_TC_HT/LT (gain is in CFG0). */
	MLX90393_SENS_TC_REG,
	/** @brief X-axis offset register (offset compensation). */
	MLX90393_OFFSET_X_REG,
	/** @brief Y-axis offset register (offset compensation). */
	MLX90393_OFFSET_Y_REG,
	/** @brief Z-axis offset register (offset compensation). */
	MLX90393_OFFSET_Z_REG,
	/** @brief Wake-on-change threshold for X/Y axes. */
	MLX90393_WOXY_THRESH_REG,
	/** @brief Wake-on-change threshold for Z axis. */
	MLX90393_WOZ_THRESH_REG,
	/** @brief Wake-on-change threshold for temperature. */
	MLX90393_WOT_THRESH_REG,
	/** @brief User register 1 (RAM; application/user-defined storage). */
	MLX90393_USER1_REG
};

#define MLX90393_CFG0_HALLCONF_POS 0
#define MLX90393_CFG0_HALLCONF_MSK (0x0F << MLX90393_CFG0_HALLCONF_POS)
#define MLX90393_CFG0_GAIN_SEL_POS 4
#define MLX90393_CFG0_GAIN_SEL_MSK (0x07 << MLX90393_CFG0_GAIN_SEL_POS)
#define MLX90393_CFG0_Z_SERIES_POS 7
#define MLX90393_CFG0_Z_SERIES_MSK (0x01 << MLX90393_CFG0_Z_SERIES_POS)
#define MLX90393_CFG0_BIST_POS 8
#define MLX90393_CFG0_BIST_MSK (0x01 << MLX90393_CFG0_BIST_POS)
#define MLX90393_CFG0_ANA_RES_LOW_POS 9
#define MLX90393_CFG0_ANA_RES_LOW_MSK (0x7F << MLX90393_CFG0_ANA_RES_LOW_POS)

/**
 * HALLCONF - Hall plate spinning rate adjustment (default=0xC).
 */
#define mlx90393_cfg0_hallconf_s(val) (MLX90393_CFG0_HALLCONF_MSK & ((val) << MLX90393_CFG0_HALLCONF_POS))
#define mlx90393_cfg0_hallconf_r(val) ((MLX90393_CFG0_HALLCONF_MSK & (val)) >> MLX90393_CFG0_HALLCONF_POS)

/**
 * GAIN_SEL - Analog chain gain setting, factor 5 between min and max code (default=0x7).
 */
#define mlx90393_cfg0_gain_sel_s(val) (MLX90393_CFG0_GAIN_SEL_MSK & ((val) << MLX90393_CFG0_GAIN_SEL_POS))
#define mlx90393_cfg0_gain_sel_r(val) ((MLX90393_CFG0_GAIN_SEL_MSK & (val)) >> MLX90393_CFG0_GAIN_SEL_POS)

/**
 * Z_SERIES - Enable all plates for Z-measurement (default=0x0).
 */
#define mlx90393_cfg0_z_series_s(val) (MLX90393_CFG0_Z_SERIES_MSK & ((val) << MLX90393_CFG0_Z_SERIES_POS))
#define mlx90393_cfg0_z_series_r(val) ((MLX90393_CFG0_Z_SERIES_MSK & (val)) >> MLX90393_CFG0_Z_SERIES_POS)

/**
 * BIST - Enabled the on-chip coil, applying a Z-field [Built-In Self Test] (default=0x0).
 */
#define mlx90393_cfg0_bist_s(val) (MLX90393_CFG0_BIST_MSK & ((val) << MLX90393_CFG0_BIST_POS))
#define mlx90393_cfg0_bist_r(val) ((MLX90393_CFG0_BIST_MSK & (val)) >> MLX90393_CFG0_BIST_POS)

/**
 * ANA_RESERVED_LOW - Reserved IO trimming bits (default=0x00).
 */
#define mlx90393_cfg0_ana_res_low_s(val) (MLX90393_CFG0_ANA_RES_LOW_MSK & ((val) << MLX90393_CFG0_ANA_RES_LOW_POS))
#define mlx90393_cfg0_ana_res_low_r(val) ((MLX90393_CFG0_ANA_RES_LOW_MSK & (val)) >> MLX90393_CFG0_ANA_RES_LOW_POS)

#define MLX90393_CFG1_BDR_POS 0
#define MLX90393_CFG1_BDR_MSK (0x3F << MLX90393_CFG1_BDR_POS)
#define MLX90393_CFG1_BURST_SEL_ZYXT_POS 6
#define MLX90393_CFG1_BURST_SEL_ZYXT_MSK (0xF << MLX90393_CFG1_BURST_SEL_ZYXT_POS)
#define MLX90393_CFG1_TCMP_EN_POS 10
#define MLX90393_CFG1_TCMP_EN_MSK (0x01 << MLX90393_CFG1_TCMP_EN_POS)
#define MLX90393_CFG1_EXT_TRG_POS 11
#define MLX90393_CFG1_EXT_TRG_MSK (0x01 << MLX90393_CFG1_EXT_TRG_POS)
#define MLX90393_CFG1_WOC_DIFF_POS 12
#define MLX90393_CFG1_WOC_DIFF_MSK (0x01 << MLX90393_CFG1_WOC_DIFF_POS)
#define MLX90393_CFG1_COMM_MODE_POS 13
#define MLX90393_CFG1_COMM_MODE_MSK (0x03 << MLX90393_CFG1_COMM_MODE_POS)
#define MLX90393_CFG1_TRIG_INT_POS 15
#define MLX90393_CFG1_TRIG_INT_MSK (0x01 << MLX90393_CFG1_TRIG_INT_POS)

/**
 * BURST_DATARATE - Defines TINTERVAL as BURST_DATA_RATE * 20ms (default=0x00).
 */
#define mlx90393_cfg1_bdr_s(val) (MLX90393_CFG1_BDR_MSK & ((val) << MLX90393_CFG1_BDR_POS))
#define mlx90393_cfg1_bdr_r(val) ((MLX90393_CFG1_BDR_MSK & (val)) >> MLX90393_CFG1_BDR_POS)

/**
 * BURST_SEL - Defines the MDATA in burst mode if SB command argument = 0 (default=0x0).
 */
#define mlx90393_cfg1_burst_sel_zyxt_s(val) \
	(MLX90393_CFG1_BURST_SEL_ZYXT_MSK & ((val) << MLX90393_CFG1_BURST_SEL_ZYXT_POS))
#define mlx90393_cfg1_burst_sel_zyxt_r(val) \
	((MLX90393_CFG1_BURST_SEL_ZYXT_MSK & (val)) >> MLX90393_CFG1_BURST_SEL_ZYXT_POS)

/**
 * TCMP_EN - Enables on-chip sensitivity drift compensation (default=0x0).
 */
#define mlx90393_cfg1_tcmp_en_s(val) (MLX90393_CFG1_TCMP_EN_MSK & ((val) << MLX90393_CFG1_TCMP_EN_POS))
#define mlx90393_cfg1_tcmp_en_r(val) ((MLX90393_CFG1_TCMP_EN_MSK & (val)) >> MLX90393_CFG1_TCMP_EN_POS)

/**
 * EXT_TRIG - Allows external trigger inputs when set, if TRIG_INT_SEL = 0 (default=0x0).
 */
#define mlx90393_cfg1_ext_trg_s(val) (MLX90393_CFG1_EXT_TRG_MSK & ((val) << MLX90393_CFG1_EXT_TRG_POS))
#define mlx90393_cfg1_ext_trg_r(val) ((MLX90393_CFG1_EXT_TRG_MSK & (val)) >> MLX90393_CFG1_EXT_TRG_POS)

/**
 * WOC_DIFF - Sets the Wake-up On Change based on Delta{sample(t),sample(t-1)} (default=0x0).
 */
#define mlx90393_cfg1_woc_diff_s(val) (MLX90393_CFG1_WOC_DIFF_MSK & ((val) << MLX90393_CFG1_WOC_DIFF_POS))
#define mlx90393_cfg1_woc_diff_r(val) ((MLX90393_CFG1_WOC_DIFF_MSK & (val)) >> MLX90393_CFG1_WOC_DIFF_POS)

/**
 * COMM_MODE - Allow only SPI [10b], only I2C [11b] or both [0Xb] according to CS pin (default=0x0).
 */
#define mlx90393_cfg1_comm_mode_s(val) (MLX90393_CFG1_COMM_MODE_MSK & ((val) << MLX90393_CFG1_COMM_MODE_POS))
#define mlx90393_cfg1_comm_mode_r(val) ((MLX90393_CFG1_COMM_MODE_MSK & (val)) >> MLX90393_CFG1_COMM_MODE_POS)

/**
 * TRIG_INT_SEL - Puts TRIG_INT pin in TRIG mode when cleared, INT mode otherwise (default=0x0).
 */
#define mlx90393_cfg1_trig_int_s(val) (MLX90393_CFG1_TRIG_INT_MSK & ((val) << MLX90393_CFG1_TRIG_INT_POS))
#define mlx90393_cfg1_trig_int_r(val) ((MLX90393_CFG1_TRIG_INT_MSK & (val)) >> MLX90393_CFG1_TRIG_INT_POS)

#define MLX90393_CFG2_OSR_POS 0
#define MLX90393_CFG2_OSR_MSK (0x03 << MLX90393_CFG2_OSR_POS)
#define MLX90393_CFG2_DIG_FILT_POS 2
#define MLX90393_CFG2_DIG_FILT_MSK (0x07 << MLX90393_CFG2_DIG_FILT_POS)
#define MLX90393_CFG2_RES_X_POS 5
#define MLX90393_CFG2_RES_X_MSK (0x03 << MLX90393_CFG2_RES_X_POS)
#define MLX90393_CFG2_RES_Y_POS 7
#define MLX90393_CFG2_RES_Y_MSK (0x03 << MLX90393_CFG2_RES_Y_POS)
#define MLX90393_CFG2_RES_Z_POS 9
#define MLX90393_CFG2_RES_Z_MSK (0x03 << MLX90393_CFG2_RES_Z_POS)
#define MLX90393_CFG2_OSR2_POS 11
#define MLX90393_CFG2_OSR2_MSK (0x03 << MLX90393_CFG2_OSR2_POS)

/**
 * OSR - Magnetic sensor ADC oversampling ratio (default=0x0).
 */
#define mlx90393_cfg2_osr_s(val) (MLX90393_CFG2_OSR_MSK & ((val) << MLX90393_CFG2_OSR_POS))
#define mlx90393_cfg2_osr_r(val) ((MLX90393_CFG2_OSR_MSK & (val)) >> MLX90393_CFG2_OSR_POS)

/**
 * DIG_FILT - Digital filter applicable to ADC (default=0x0).
 */
#define mlx90393_cfg2_dig_filt_s(val) (MLX90393_CFG2_DIG_FILT_MSK & ((val) << MLX90393_CFG2_DIG_FILT_POS))
#define mlx90393_cfg2_dig_filt_r(val) ((MLX90393_CFG2_DIG_FILT_MSK & (val)) >> MLX90393_CFG2_DIG_FILT_POS)

/**
 * RES_X - Selects the desired 16-bit output value from the 19-bit ADC (default=0x00).
 */
#define mlx90393_cfg2_res_x_s(val) (MLX90393_CFG2_RES_X_MSK & ((val) << MLX90393_CFG2_RES_X_POS))
#define mlx90393_cfg2_res_x_r(val) ((MLX90393_CFG2_RES_X_MSK & (val)) >> MLX90393_CFG2_RES_X_POS)

/**
 * RES_Y - Selects the desired 16-bit output value from the 19-bit ADC (default=0x00).
 */
#define mlx90393_cfg2_res_y_s(val) (MLX90393_CFG2_RES_Y_MSK & ((val) << MLX90393_CFG2_RES_Y_POS))
#define mlx90393_cfg2_res_y_r(val) ((MLX90393_CFG2_RES_Y_MSK & (val)) >> MLX90393_CFG2_RES_Y_POS)

/**
 * RES_Z - Selects the desired 16-bit output value from the 19-bit ADC (default=0x00).
 */
#define mlx90393_cfg2_res_z_s(val) (MLX90393_CFG2_RES_Z_MSK & ((val) << MLX90393_CFG2_RES_Z_POS))
#define mlx90393_cfg2_res_z_r(val) ((MLX90393_CFG2_RES_Z_MSK & (val)) >> MLX90393_CFG2_RES_Z_POS)

/**
 * OSR2 - Temperature sensor ADC oversampling ratio (default=0x0).
 */
#define mlx90393_cfg2_osr2_s(val) (MLX90393_CFG2_OSR2_MSK & ((val) << MLX90393_CFG2_OSR2_POS))
#define mlx90393_cfg2_osr2_r(val) ((MLX90393_CFG2_OSR2_MSK & (val)) >> MLX90393_CFG2_OSR2_POS)

typedef struct mlx90393_dsc *mlx90393;

/**
 * @struct mlx90393_dsc
 * @brief MLX90393 driver descriptor (handle).
 *
 * The public handle type @ref mlx90393 is a pointer to this structure.
 * The structure is exposed to allow static allocation. Before initialization,
 * the caller sets @c spi.cfg (SPI controller, mode, speed, chip select, etc.).
 * Other fields are driver-owned; the conversion fields are populated by
 * @ref mlx90393_update_conv_cache(). A system-lifetime descriptor is supported.
 */
struct mlx90393_dsc {
	struct spi_hal_dev spi;
	/** Cached configuration + derived conversion parameters (filled by mlx90393_update_conv_cache()). */
	uint16_t cfg0;
	uint16_t cfg1;
	uint16_t cfg2;
	uint8_t hallconf;
	uint8_t gain_sel;
	uint8_t tcmp_en;
	uint8_t res_x;
	uint8_t res_y;
	uint8_t res_z;
	uint8_t osr;
	uint8_t dig_filt;
	uint8_t osr2;
	/* Sensitivity in nT/LSB (integer). */
	int sens_x_nT_per_lsb;
	int sens_y_nT_per_lsb;
	int sens_z_nT_per_lsb;
	/* Raw format helpers (0uT point and signedness depend on TCMP_EN + RES). */
	boolean_t x_is_twos_complement;
	boolean_t y_is_twos_complement;
	boolean_t z_is_twos_complement;
	int x_zero_lsb;
	int y_zero_lsb;
	int z_zero_lsb;
	boolean_t conv_cache_valid;
};

/**
 * @struct mlx90393_measure_data
 * @brief Measurement output container.
 *
 * Holds raw measurement samples returned by the sensor for each requested
 * channel. Before each read, the driver clears the entire structure. It then
 * sets a validity flag for every requested and successfully decoded channel;
 * unrequested channels remain zero with their validity flags cleared.
 *
 * @note All samples are returned in **raw** format as @c uint16_t exactly as read
 *       from the device. The application must convert/interpret them according
 *       to the current sensor configuration (e.g. 2's complement vs unsigned,
 *       resolution, gain/sensitivity scaling, temperature conversion, etc.).
 */
struct mlx90393_measure_data {
	boolean_t x_valid;
	uint16_t x;
	boolean_t y_valid;
	uint16_t y;
	boolean_t z_valid;
	uint16_t z;
	boolean_t t_valid;
	uint16_t t;
};

/**
 * @struct mlx90393_sample
 * @brief STM measurement with original RAW codes and processed SI values.
 *
 * The selected STM fills this structure after a successful RM read.
 * @c raw preserves the unmodified sensor words and validity flags.
 * @c x_nt, @c y_nt and @c z_nt are converted to nanotesla, with optional
 * software hard-iron offsets subtracted and REPORT_FLIP_X applied.
 * @c t_mc is temperature in milli-degrees Celsius; it is not affected by
 * magnetic corrections. The STM uses the same converted values for its
 * formatted-text reports.
 *
 * @note INT32_MIN marks an unavailable converted field (invalid RAW channel
 *       or hard-iron correction overflow). Consult @c raw.*_valid to
 *       distinguish an invalid RAW channel from a correction overflow.
 * @note The STM supplies this structure by pointer in task context. The
 *       client must copy it during the callback to retain it afterwards.
 */
struct mlx90393_sample {
	struct mlx90393_measure_data raw;
	int32_t x_nt;
	int32_t y_nt;
	int32_t z_nt;
	int32_t t_mc;
};

/**
 * @brief Initialize one MLX90393 handle and its SPI HAL device.
 *
 * Calls @c spi_hal_dev_init() and invalidates the conversion cache. This
 * function does not reset the sensor or write its configuration registers.
 *
 * @param[in,out] mlx Caller-owned, initialized descriptor with valid @c spi.cfg.
 * @pre Configure @c mlx->spi.cfg before calling; invoke once per device.
 * @note Initialization may create SPI HAL resources. Call from startup/task
 *       context, not from an ISR. Device lifetime is managed by the caller.
 */
void init_mlx90393(mlx90393 mlx);

/**
 * @brief Issue a sensor measurement-mode command (SB, SW or SM).
 *
 * Transmits the selected command with the low four channel-selection bits
 * (bit 3=Z, 2=Y, 1=X, 0=T). A zero channel mask requests BURST_SEL from
 * sensor RAM, as specified by the MLX90393 command protocol. The function
 * checks the ERROR status bit and the expected measurement-mode status bit.
 * It starts the acquisition but does not wait for data readiness.
 *
 * @param[in,out] mlx Initialized sensor handle.
 * @param[in] mode One of @ref mlx90393_measure_mode.
 * @param[in] measure Channel mask (MLX90393_MEASURE_*); bits above bit 3 are ignored.
 * @return 0 if the command is accepted, -EHW on sensor status/mode error,
 *         otherwise the negative SPI HAL error code.
 * @pre The sensor is configured and @p mode is valid. An invalid mode is fatal
 *      via @c crit_err_exit(BAD_PARAMETER).
 * @note Synchronous SPI operation; task context only; no internal locking.
 */
int mlx90393_start_measure(mlx90393 mlx, enum mlx90393_measure_mode mode, unsigned int measure);

/**
 * @brief Read the most recent measurement for the requested channels (RM).
 *
 * Sends RM with the selected channel mask and reads one status byte plus
 * two bytes per channel. The sensor returns enabled quantities in T, X, Y,
 * Z order, most-significant byte first. The function verifies the ERROR
 * bit and the D1:D0 response-length encoding, then populates the raw values
 * and their validity flags. The destination is zeroed before the SPI call,
 * including on SPI/status failure. This function does not wait for DRDY.
 *
 * @param[in,out] mlx Initialized sensor handle.
 * @param[in] measure Nonzero combination of MLX90393_MEASURE_* matching the acquisition.
 * @param[out] data Caller-owned structure to receive raw 16-bit samples.
 * @return 0 on successful SPI read and decoding, -EHW on invalid sensor
 *         status/response length, otherwise the negative SPI HAL error code.
 * @pre Call only after the measurement is ready; supply a valid @p data pointer.
 * @note Blocking DMA SPI transfer; task context only; no internal locking.
 */
int mlx90393_read_measure_data(mlx90393 mlx, unsigned int measure, struct mlx90393_measure_data *data);

/**
 * @brief Exit the active sensor measurement mode (EX command).
 *
 * Sends EX and, on success, waits 1500 us before returning.
 *
 * @param[in,out] mlx Initialized sensor handle.
 * @return 0 on success, -EHW for a sensor error, or a negative SPI HAL code.
 * @note Blocking SPI operation and delay; task context only.
 */
int mlx90393_exit_mode(mlx90393 mlx);

/**
 * @brief Recall nonvolatile configuration into sensor working RAM (HR).
 *
 * Marks the conversion cache invalid before issuing the command. Read the
 * resulting configuration again with @ref mlx90393_update_conv_cache()
 * before converting magnetic samples.
 *
 * @param[in,out] mlx Initialized sensor handle.
 * @return 0 on success, -EHW for a sensor error, or a negative SPI HAL code.
 * @note Blocking SPI operation; task context only.
 */
int mlx90393_memory_recall(mlx90393 mlx);

/**
 * @brief Store the sensor working configuration into NVRAM (HS).
 *
 * After a successful HS command, waits at least 15 ms before returning to
 * allow NVRAM storage to complete (rounded-up FreeRTOS ticks plus one
 * extra tick for the scheduler tick phase). The caller must not issue
 * another sensor command until this function returns. It does not modify
 * the driver's conversion cache.
 *
 * @param[in,out] mlx Initialized sensor handle.
 * @return 0 on success, -EHW for a sensor error, or a negative SPI HAL code.
 * @warning Reliable NVRAM storage requires VDD >= 3.3 V during HS. Do not
 *          remove power while storing. HS is intended for calibration, not
 *          repeated application writes; minimize NVRAM write cycles.
 *          Ordinary WR commands affect volatile RAM without this limit.
 * @note Blocking SPI operation followed by FreeRTOS task delay; not ISR-safe.
 */
int mlx90393_memory_store(mlx90393 mlx);

/**
 * @brief Read one 16-bit MLX90393 RAM register (RR command).
 *
 * Sends the register address shifted left by two bits, then reads the
 * status byte and two data bytes in big-endian order. Verifies ERROR and
 * that D1:D0 indicates exactly two data bytes.
 *
 * @param[in,out] mlx Initialized sensor handle.
 * @param[in] adr Register address from @ref mlx90393_reg_adr.
 * @param[out] data Caller-provided location for the register value, written on success.
 * @return 0 on success, -EHW on sensor status/length error, or a negative SPI HAL code.
 * @note Blocking SPI/DMA operation; task context only; no internal locking.
 */
int mlx90393_reg_read(mlx90393 mlx, enum mlx90393_reg_adr adr, uint16_t *data);

/**
 * @brief Write one 16-bit MLX90393 RAM register (WR command).
 *
 * Sends the register value in big-endian order followed by the register
 * address shifted left by two bits; checks the status ERROR bit.
 *
 * @param[in,out] mlx Initialized sensor handle.
 * @param[in] adr Register address from @ref mlx90393_reg_adr.
 * @param[in] data New 16-bit register value.
 * @return 0 on success, -EHW on sensor status error, or a negative SPI HAL code.
 * @warning A successful write does not invalidate or update the driver's
 *          conversion cache. After changing CFG0, CFG1 or CFG2, call
 *          @ref mlx90393_update_conv_cache() before magnetic conversion.
 * @note Blocking SPI/DMA operation; task context only; no internal locking.
 */
int mlx90393_reg_write(mlx90393 mlx, enum mlx90393_reg_adr adr, uint16_t data);

/**
 * @brief Test SPI communication using the sensor's USER1 RAM register.
 *
 * Saves USER1, writes test pattern 0xABCD, reads it back, and restores the
 * saved value only if the preceding operations succeed. An interrupted or
 * failed probe may leave USER1 changed; this is not a nondestructive read.
 *
 * @param[in,out] mlx Initialized sensor handle.
 * @return 0 on successful verification and restore, -EHW on pattern mismatch
 *         or sensor status error, or a negative SPI HAL error code.
 * @note Blocking SPI/DMA operations; task context only; no internal locking.
 */
int mlx90393_probe(mlx90393 mlx);

/**
 * @brief Soft-reset the MLX90393 and invalidate its conversion cache.
 *
 * Sends EX, waits 1500 us, sends RT, waits at least 5 ms (FreeRTOS
 * tick-rounded delay with one additional tick), then sends EX again.
 * Checks for sensor ERROR and requires the reset-status (RS) indication
 * on at least one of the status responses inspected after RT.
 *
 * @param[in,out] mlx Initialized sensor handle.
 * @return 0 on success, -EHW on status/reset-indication error, or a negative SPI HAL code.
 * @note Blocking SPI plus microsecond and FreeRTOS task delays; not ISR-safe.
 * @post Reconfigure the sensor and call @ref mlx90393_update_conv_cache()
 *       before further magnetic conversion.
 */
int mlx90393_soft_reset(mlx90393 mlx);

/**
 * @brief Read CFG0, CFG1 and CFG2 and rebuild the conversion cache.
 *
 * Caches HALLCONF, GAIN_SEL, TCMP_EN, RES_X/Y/Z, OSR, DIG_FILT and OSR2;
 * derives per-axis integer sensitivity in nT/LSB and signed/offset coding.
 * Sensitivity uses the datasheet's 25 C table with TCMP_EN=0 or 35 C table
 * with TCMP_EN=1. For HALLCONF=0x0 it scales by 98/75. Only HALLCONF=0x0
 * and 0xC are supported.
 *
 * @param[in,out] mlx Initialized sensor handle and conversion cache destination.
 * @return 0 when all three registers were read and the cache is valid;
 *         -EHW or a negative SPI HAL code on a read error.
 * @pre Sensor RAM configuration must already contain the desired settings.
 * @note Invalidates the cache at entry, including on failure. Unsupported
 *       HALLCONF or TCMP_EN=1 with RES_X/Y/Z greater than 1 is fatal via
 *       @c crit_err_exit(BAD_PARAMETER).
 * @note Blocking SPI/DMA operation; task context only; no internal locking.
 */
int mlx90393_update_conv_cache(mlx90393 mlx);

/**
 * @brief Convert one raw magnetic sample into signed nanotesla (nT).
 *
 * Uses the cached axis sensitivity and output coding to compute nT from
 * the sensor's 16-bit sample. This is based on typical datasheet sensitivity;
 * it does not subtract a measured zero-field offset or calibrate the sensor.
 *
 * @param[in] mlx Sensor handle containing an up-to-date conversion cache.
 * @param[in] axis One of MLX90393_MEASURE_X, _Y or _Z (single axis only).
 * @param[in] raw Raw 16-bit sample for that axis.
 * @param[out] out_nt Caller-owned destination for the signed nT result.
 * @pre @ref mlx90393_update_conv_cache() must have succeeded for the current
 *      configuration. Invalid cache or axis is fatal via @c crit_err_exit().
 * @note Integer calculation; no I/O or blocking. Not a calibrated result.
 */
void mlx90393_mag_raw_to_nt(mlx90393 mlx, unsigned int axis, uint16_t raw, int32_t *out_nt);

/**
 * @brief Convert one raw magnetic sample into signed microtesla (uT).
 *
 * First converts to nT, then rounds to the nearest integer uT, discarding
 * sub-microtesla resolution. Half values are rounded away from zero.
 *
 * @param[in] mlx Sensor handle containing an up-to-date conversion cache.
 * @param[in] axis One of MLX90393_MEASURE_X, _Y or _Z (single axis only).
 * @param[in] raw Raw 16-bit sample for that axis.
 * @param[out] out_ut Caller-owned destination for the signed uT result.
 * @pre @ref mlx90393_update_conv_cache() must have succeeded for the current
 *      configuration. Invalid cache or axis is fatal via @c crit_err_exit().
 * @note Integer calculation; no I/O or blocking. Not a calibrated result.
 */
void mlx90393_mag_raw_to_ut(mlx90393 mlx, unsigned int axis, uint16_t raw, int32_t *out_ut);

/**
 * @brief Convert raw temperature to integer milli-degrees Celsius (mC).
 *
 * Uses typical sensor calibration constants T25=46244 and TRES=45.2 LSB/C:
 * T(mC) = 25000 + round((raw_t - 46244) * 10000 / 452).
 * The calculation uses signed 64-bit intermediate arithmetic.
 *
 * @param[in] raw_t Unsigned 16-bit temperature sample from the sensor.
 * @param[out] out_mc Caller-owned destination for temperature in 0.001 C.
 * @note Does not access the device or depend on the conversion cache.
 */
void mlx90393_temp_raw_to_mc(uint16_t raw_t, int32_t *out_mc);

#endif
