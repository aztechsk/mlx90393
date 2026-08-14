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
  * from the device. The conversion helpers provided by this driver interpret the raw words and
  * scale them into integer physical units:
  *   - Magnetic field: nT (nanoTesla) or uT (microTesla).
  *   - Temperature: mC (milliCelsius).
  *
  * @section mlx90393_conv_cache Conversion cache and what affects scaling.
  * Magnetic conversion depends on the current sensor configuration. The driver therefore caches
  * the relevant configuration and derived conversion parameters in the descriptor
  * (see @ref mlx90393_dsc). Call @ref mlx90393_update_conv_cache() whenever configuration
  * changes (or after reset), before using conversion helpers.
  *
  * Cached parameters that affect magnetic conversion:
  *   - @c GAIN_SEL and @c RES_X/Y/Z select sensitivity (scale) in uT/LSB (different for XY vs Z).
  *     The driver derives an integer sensitivity in nT/LSB from the datasheet sensitivity tables
  *     for HALLCONF=0xC. With @c TCMP_EN disabled, the 25C reference table is used. With
  *     @c TCMP_EN enabled, the 35C reference table is used because the temperature-compensation
  *     pivot TREF corresponds to 35C. For @c HALLCONF=0x0 the datasheet specifies a sensitivity
  *     scaling factor 98/75, which the driver applies when computing the cache.
  *   - @c TCMP_EN affects the **output coding** (2's complement vs unsigned with a 0uT bias point)
  *     and enables on-chip sensitivity drift compensation. The driver caches per-axis raw format
  *     helpers:
  *       - @c *_is_twos_complement : whether raw words are 2's complement.
  *       - @c *_zero_lsb           : the raw code corresponding to 0uT when unsigned-coded.
  *
  * Note: @c OSR / @c DIG_FILT influence conversion time and noise, but not the magnetic scale factor.
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
  * @section mlx90393_conv_notes Practical notes.
  * - Sensitivities used are typical table values: the 25C reference with @c TCMP_EN disabled
  *   and the 35C compensation-pivot reference with @c TCMP_EN enabled. Absolute accuracy depends
  *   on device tolerances, temperature drift, and any user calibration/offset strategy.
  * - When @c TCMP_EN is enabled, the sensor performs on-chip sensitivity drift compensation; the
  *   output coding and 0uT bias point depend on @c RES as documented in the datasheet.
  */

#ifndef MLX90393_H
#define MLX90393_H

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
	 * Starts a burst acquisition according to the sensor burst configuration
	 * (e.g. BURST_SEL / enabled channels). Typically used for continuous or
	 * repeated sampling workflows.
	 */
	MLX90393_MEASURE_MODE_BURST,
	/**
	 * @brief Wake-on-change / wake measurement (SW command).
	 *
	 * Issues a wake measurement intended to be used with wake-on-change / low-power
	 * operation. The exact behavior depends on sensor configuration.
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
	/** @brief Configuration register 2 (interrupt, burst and system settings). */
	MLX90393_CFG2_REG,
	/** @brief Sensitivity / temperature compensation register (gain/TC settings). */
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
#define mlx90393_cfg1_burst_sel_zyxt_s(val) (MLX90393_CFG1_BURST_SEL_ZYXT_MSK & ((val) << MLX90393_CFG1_BURST_SEL_ZYXT_POS))
#define mlx90393_cfg1_burst_sel_zyxt_r(val) ((MLX90393_CFG1_BURST_SEL_ZYXT_MSK & (val)) >> MLX90393_CFG1_BURST_SEL_ZYXT_POS)

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
 * The structure is exposed to allow static allocation, but its fields should be
 * treated as private to the driver (except for initialization).
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
 * channel. Validity flags indicate which fields contain updated data.
 *
 * @note All samples are returned in **raw** format as @c uint16_t exactly as read
 *       from the device. The application must convert/interpret them according
 *       to the current sensor configuration (e.g. 2's complement vs unsigned,
 *       resolution/OSR, gain/sensitivity scaling, temperature conversion, etc.).
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
 * @brief Initialize the MLX90393 driver instance.
 *
 * Initializes the underlying SPI HAL device referenced by @p mlx.
 * This function does not configure the SPI bus parameters (mode/clock/word size);
 * the upper layer is responsible for proper SPI configuration.
 *
 * @param mlx  Sensor handle/instance.
 */
void init_mlx90393(mlx90393 mlx);

/**
 * @brief Start a measurement (single/burst) for selected axes/temperature.
 *
 * @param mlx      Sensor handle/instance.
 * @param mode     Measurement mode (see @ref mlx90393_measure_mode).
 * @param measure  Channel bitmask (X/Y/Z/T) as defined by the driver.
 *
 * @return 0 on success, otherwise a negative error code (typically propagated from SPI HAL).
 *
 * @pre @p measure and @p mode are valid per the driver contract.
 */
int mlx90393_start_measure(mlx90393 mlx, enum mlx90393_measure_mode mode, unsigned int measure);

/**
 * @brief Read measured data for selected channels.
 *
 * Reads measurement results for channels specified by @p measure and stores them into @p data.
 * Fields for non-requested channels remain zero.
 *
 * @param  mlx      Sensor handle/instance.
 * @param  measure  Channel bitmask (X/Y/Z/T) as defined by the driver.
 * @param  data     Output structure holding measurement results.
 *
 * @return 0 on success, otherwise a negative error code.
 *
 * @note Sign/format interpretation depends on sensor configuration (e.g. 2's complement vs unsigned).
 */
int mlx90393_read_measure_data(mlx90393 mlx, unsigned int measure, struct mlx90393_measure_data *data);

/**
 * @brief Exit the current sensor mode (enter idle).
 *
 * @param mlx  Sensor handle/instance.
 *
 * @return 0 on success, otherwise a negative error code.
 */
int mlx90393_exit_mode(mlx90393 mlx);

/**
 * @brief Recall configuration from NVRAM into working registers (RAM).
 *
 * @param mlx  Sensor handle/instance.
 *
 * @return 0 on success, otherwise a negative error code.
 */
int mlx90393_memory_recall(mlx90393 mlx);

/**
 * @brief Store current configuration from working registers (RAM) into NVRAM.
 *
 * @param mlx  Sensor handle/instance.
 *
 * @return 0 on success, otherwise a negative error code.
 *
 * @warning Do not remove power during the store operation.
 * @warning Store reliability may depend on supply voltage (see datasheet).
 */
int mlx90393_memory_store(mlx90393 mlx);

/**
 * @brief Read a 16-bit sensor register.
 *
 * @param mlx   Sensor handle/instance.
 * @param adr   Register address.
 * @param data  Output register value.
 *
 * @return 0 on success, otherwise a negative error code.
 *
 * @note Blocking call.
 */
int mlx90393_reg_read(mlx90393 mlx, enum mlx90393_reg_adr adr, uint16_t *data);

/**
 * @brief Write a 16-bit value to a sensor register.
 *
 * @param mlx   Sensor handle/instance.
 * @param adr   Register address.
 * @param data  Value to write.
 *
 * @return 0 on success, otherwise a negative error code.
 */
int mlx90393_reg_write(mlx90393 mlx, enum mlx90393_reg_adr adr, uint16_t data);

/**
 * @brief Probe the sensor (communication/link test).
 *
 * The implementation typically performs a read/write operation on a selected USER register
 * and verifies that SPI communication works.
 *
 * @param mlx  Sensor handle/instance.
 *
 * @return 0 on success, otherwise a negative error code.
 *
 * @warning This function modifies the USER1 register in chip RAM.
 */
int mlx90393_probe(mlx90393 mlx);

/**
 * @brief Perform a sensor soft reset.
 *
 * Resets the internal sensor state; typically restores default values of volatile registers.
 *
 * @param mlx  Sensor handle/instance.
 *
 * @return 0 on success, otherwise a negative error code.
 */
int mlx90393_soft_reset(mlx90393 mlx);

/**
 * @brief Read CFG registers and update cached conversion parameters.
 *
 * @param mlx  Sensor handle/instance.
 *
 * @return 0 on success, otherwise a negative error code.
 */
int mlx90393_update_conv_cache(mlx90393 mlx);

/**
 * @brief Convert raw magnetic sample to nanoTesla (nT).
 *
 * Converts a single raw 16-bit sample read from the sensor into magnetic field
 * in nanoTesla, using conversion parameters cached in @p mlx (gain, resolution,
 * TCMP format, zero-point).
 *
 * @param  mlx     Sensor handle/instance (conversion cache source).
 * @param  axis    Axis selector. Must be one of: @ref MLX90393_MEASURE_X,
 *                     @ref MLX90393_MEASURE_Y, @ref MLX90393_MEASURE_Z.
 * @param  raw     Raw 16-bit sample as returned by the sensor.
 * @param  out_nt  Converted magnetic field in nanoTesla (nT).
 *
 * @pre @ref mlx90393_update_conv_cache() has been called successfully before this conversion.
 *
 * @note Rounding is determined by integer arithmetic; output is signed.
 */
void mlx90393_mag_raw_to_nt(mlx90393 mlx, unsigned int axis, uint16_t raw, int32_t *out_nt);

/**
 * @brief Convert raw magnetic sample to microTesla (uT).
 *
 * Converts a raw 16-bit sample to microTesla by first converting to nT and then
 * rounding to the nearest uT.
 *
 * @param  mlx     Sensor handle/instance (conversion cache source).
 * @param  axis    Axis selector. Must be one of: @ref MLX90393_MEASURE_X,
 *                     @ref MLX90393_MEASURE_Y, @ref MLX90393_MEASURE_Z.
 * @param  raw     Raw 16-bit sample as returned by the sensor.
 * @param  out_ut  Converted magnetic field in microTesla (uT).
 *
 * @pre @ref mlx90393_update_conv_cache() has been called successfully before this conversion.
 *
 * @note Output is signed. Conversion uses rounding (nT -> uT).
 */
void mlx90393_mag_raw_to_ut(mlx90393 mlx, unsigned int axis, uint16_t raw, int32_t *out_ut);

/**
 * @brief Convert raw temperature sample to milli-degrees Celsius (mC).
 *
 * Converts a raw temperature sample (as returned by the sensor when measuring T)
 * into milli-degrees Celsius using the sensor's typical calibration constants.
 *
 * @param  raw_t    Raw 16-bit temperature sample as returned by the sensor.
 * @param  out_mc   Temperature in milli-degrees Celsius (mC).
 *
 * @note This conversion uses integer arithmetic and rounding.
 */
void mlx90393_temp_raw_to_mc(uint16_t raw_t, int32_t *out_mc);

#endif
