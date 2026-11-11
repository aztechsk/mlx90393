/*
 * mlx90393_stm.h
 *
 * Autors: Jan Rusnak.
 * (c) 2025 AZTech.
 */

#ifndef MLX90393_STM_H
#define MLX90393_STM_H

#include <gentyp.h>
#include "sysconf.h"

/**
 * @file mlx90393_stm.h
 * @brief FreeRTOS state machine for periodic MLX90393 SPI burst measurements.
 *
 * This is the application-level acquisition layer built on top of
 * mlx90393.h. The implementation in mlx90393_stm.c is compiled only
 * when MLX90393_SENSOR_MODE == MLX90393_INTR_MODE. Other operating modes
 * have no implementation in this module.
 *
 * @section mlx90393_stm_setup Sensor and board configuration
 *
 * The board-specific SPI/GPIO selections are in mlx90393_stm.c and board.h.
 * init_mlx90393_stm() configures SPI bus 0, mode 3, 8-bit transfers, 10 MHz
 * clock, CS1 on GPIO controller 1/pin 14, CS held over each transfer, and
 * a 10 ns CS-to-clock delay. The interrupt input uses MLX90393_INT_CONT
 * and MLX90393_INT_PIN from the board configuration: GPIO input, no pull,
 * no glitch filter, Schmitt trigger enabled, active-high level interrupt.
 * The software configures TRIG_INT_SEL=1 (INT function) in sensor CFG1.
 *
 * The STM programs the following sensor RAM registers on each startup
 * and after an error-triggered reset:
 * - CFG0: HALLCONF=0xC, Z_SERIES=0, BIST=0 and GAIN_SEL from the selected
 *   range preset. Existing ANA_RESERVED_LOW trimming bits are preserved.
 * - CFG1: BURST_SEL=0xF (all T/X/Y/Z), TCMP_EN=0,
 *   EXT_TRIG=0, WOC_DIFF=0, COMM_MODE=2 (SPI only), TRIG_INT_SEL=1,
 *   BDR derived from the requested data rate and conversion time.
 * - CFG2: configured OSR, DIG_FILT and OSR2; RES_X/Y/Z from the
 *   selected range preset.
 *
 * The following build-time settings may be supplied by sysconf.h:
 * - MLX90393_OSR (0..3; default 0): magnetic ADC oversampling.
 * - MLX90393_DIG_FILT (0..7; default 4): magnetic digital filtering.
 * - MLX90393_OSR2 (0..3; default 0): temperature ADC oversampling.
 * - MLX90393_TCMP_EN (must be 0; default 0): on-chip temperature
 *   compensation is not supported by this STM. Selecting 1 fails at build
 *   time; a different acquisition mode may support it in the future.
 *
 * With HALLCONF=0xC, OSR/DIG_FILT combinations (0,0), (0,1) and (1,0)
 * are not permitted by the datasheet. The STM rejects these and all
 * out-of-range values at build time. Increasing oversampling or filtering
 * generally reduces noise but increases the conversion time, changing the
 * BDR code and the achievable sample rate. TCMP_EN remains fixed at zero
 * in this STM.
 *
 * MLX90393_RANGE_PROFILE is selected at build time (sysconf.h override):
 * - MLX90393_RANGE_5MT (default): GAIN_SEL=7, RES_X/Y/Z=0; XY +/-4.9 mT.
 * - MLX90393_RANGE_25MT: GAIN_SEL=0, RES_X/Y/Z=0; XY +/-24.6 mT.
 * - MLX90393_RANGE_50MT: GAIN_SEL=0, RES_X/Y/Z=1; XY +/-49.2 mT.
 * These are approximate digital output ranges for XY, derived from
 * typical 25 C sensitivity and 16-bit coding; Z has a different
 * sensitivity. They do not extend the physical magnetic range of the
 * sensor. Datasheet magnetic saturation onset is about 50 mT for
 * standard IMC parts (ABA-0xx) and 25 mT for thin IMC parts
 * (ABA-1xx). Thus the +/-50 mT digital preset does not guarantee
 * unsaturated measurements to +/-50 mT on every sensor variant.
 * With TCMP_EN=0 and the supported RES=0/1 settings, magnetic readings
 * use signed two's-complement coding.
 *
 * @section mlx90393_stm_hardiron Software hard-iron correction
 *
 * The STM does not program the sensor's OFFSET_X/Y/Z registers. Instead,
 * it optionally subtracts fixed per-axis offsets from converted magnetic
 * values, in integer nanotesla (nT), before reporting. Configure in
 * sysconf.h:
 * - MLX90393_HARDIRON_EN (required; 0=disabled, 1=enabled).
 * - MLX90393_HARDIRON_X_NT, MLX90393_HARDIRON_Y_NT,
 *   MLX90393_HARDIRON_Z_NT (required when enabled; signed int32_t nT).
 *
 * For each valid axis, reported_nT = converted_nT - hardiron_offset_nT.
 * The offsets use the native sensor axis directions; REPORT_FLIP_X is
 * applied afterwards. Invalid samples remain invalid; if a corrected
 * result cannot fit in the report's signed int32_t range (INT32_MIN is
 * reserved for invalid data), it is reported as an invalid field.
 * Both logger and report callback receive corrected values. Temperature,
 * sensor RAW data and the low-level driver are unaffected. These offsets
 * compensate a constant field bias, not scale, cross-axis or soft-iron
 * errors. The STM always uses TCMP_EN=0.
 *
 * @section mlx90393_stm_timing Burst rate and measurement cycle
 *
 * MLX90393_BURST_RATE_HZ defaults to 10 (build-time override). The STM
 * estimates conversion time using the selected OSR, DIG_FILT and OSR2
 * settings and the datasheet's T_ACTIVE, T_CONVM, T_CONVT and T_CONV_END
 * formulas for BDR > 0. It rounds the inactive burst interval UP to a
 * multiple of 20 ms so that the nominal sample rate does not exceed the
 * requested rate. BDR=0 (continuous burst) is not supported by this STM;
 * configurations outside the supported BDR=1..63 range fail at build time.
 *
 * At default settings (TXYZ, OSR=0, DIG_FILT=4, OSR2=0):
 * - T_CONVM = 1219 us per magnetic axis; T_CONVT = 259 us.
 * - T_CONV_BURST = 360 + 3*1219 + 259 + 100 = 4376 us.
 * - BDR=1 gives 24.376 ms / 41.024 Hz, the highest nominal burst rate.
 * - The accepted integer MLX90393_BURST_RATE_HZ range is 1..42 Hz.
 * - BDR=5 gives T_INTERVAL=100 ms, nominal period=104.376 ms,
 *   and about 9.581 samples/s for the default request of 10 Hz.
 *
 * The following table groups the whole-Hz requests producing the same BDR
 * only with the default OSR=0, DIG_FILT=4 and OSR2=0 settings:
 *
 * | Requested rate (Hz) | BDR | Nominal output (Hz) |
 * | :------------------ | --: | ------------------: |
 * | 1                   |  50 |               0.996 |
 * | 2                   |  25 |               1.983 |
 * | 3                   |  17 |               2.904 |
 * | 4                   |  13 |               3.782 |
 * | 5                   |  10 |               4.893 |
 * | 6                   |   9 |               5.424 |
 * | 7-8                 |   7 |               6.926 |
 * | 9                   |   6 |               8.040 |
 * | 10-11               |   5 |               9.581 |
 * | 12-15               |   4 |              11.852 |
 * | 16-22               |   3 |              15.534 |
 * | 23-41               |   2 |              22.535 |
 * | 42                  |   1 |              41.024 |
 *
 * Requests of 0 or more than 42 Hz are rejected for these settings.
 * Changing OSR, DIG_FILT or OSR2 changes the allowable request range and
 * output rates; the STM validates the calculated BDR at build time.
 * The requested Hz value is an upper limit for the nominal rate, not a
 * guarantee of an exact sampling frequency. Other BDR values exist in
 * hardware but cannot all be selected by this whole-Hz request interface.
 * These are typical datasheet-based timing estimates, not measured rates;
 * oscillator tolerances can change the actual rate.
 *
 * @section mlx90393_stm_readout Interrupt-driven readout
 *
 * The dedicated FreeRTOS task runs the following cycle:
 * - Soft reset (EX, RT, EX), test SPI using the USER1 register, and set up
 *   the board's level-high GPIO interrupt input.
 * - Write CFG0/1/2, refresh the magnetic conversion cache, send SB with
 *   channel mask 0xF (T, X, Y and Z), and enable the GPIO interrupt.
 * - A data-ready interrupt disables further GPIO interrupts and gives a
 *   binary semaphore from ISR context. The task waits for the semaphore
 *   with a timeout based on the expected burst period plus 200 ms.
 * - After wake-up, delay 140 us (datasheet T_INT_RDY minimum 132 us),
 *   verify the GPIO level is still high, and read all four channels using
 *   RM. The low-level driver decodes status and reads raw 16-bit T/X/Y/Z.
 * - Convert X/Y/Z to nT and T to mC; optionally subtract the per-axis
 *   software hard-iron offsets, invert X, and report signed values with
 *   three fractional digits in uT and degrees C.
 *   Re-enable the GPIO interrupt for the next sample.
 * - On a timeout, read error or other state-machine failure, delay 5000 ms,
 *   then restart the reset/probe/configuration sequence. Diagnostic counters
 *   can be displayed by the registered terminal command "mlxs".
 *
 * @note The STM does not guarantee lossless acquisition of every burst
 *       conversion. If readout or reporting is delayed beyond the sensor's
 *       measurement period, intermediate samples may be overwritten; the
 *       next RM reads the available measurement. The read_data_reg_cnt
 *       diagnostic counts successful RM reads, not all sensor conversions.
 *
 * The report formats magnetic values from the low-level conversion to whole
 * nT, preserving 0.001 uT display increments without floating-point math.
 * This is numeric resolution, not guaranteed sensor accuracy. For precision
 * analysis, raw values remain available through the low-level driver.
 *
 * @section mlx90393_stm_reporting Reporting configuration
 *
 * The build configuration (included via sysconf.h) controls these flags:
 * - MLX90393_REPORT_SI: report formatted samples through the message logger.
 * - MLX90393_REPORT_FN: include a report callback and its formatting buffer.
 * - MLX90393_REPORT_FLIP_X: reverse the sign of X after hard-iron
 *   correction (if enabled).
 * - MLX90393_REPORT_MSG_NUM: include a 000..999 rolling sample number.
 * Reports contain X/Y/Z as signed uT with three decimal places, and T as
 * signed degrees C with three decimal places. Invalid fields are printed
 * as "----.---". The text format is identical for logging and callbacks.
 *
 * Reference: Melexis MLX90393 datasheet, revision 012 (18 November 2025),
 * sections 11, 12, 14, 15 and 16.
 */

/**
 * @brief Initialize and start the MLX90393 interrupt-driven acquisition task.
 *
 * Initializes the SPI HAL descriptor, allocates the optional report buffer,
 * creates a binary interrupt semaphore, registers the "mlxs" command, and
 * creates the fixed FreeRTOS MLX90393 task. The task itself performs sensor
 * reset, probe, configuration and continuous burst acquisition.
 *
 * @pre Call once during system startup from task context, not from an ISR.
 *      The platform SPI/GPIO HAL, board configuration and FreeRTOS allocation
 *      facilities must be available. No repeated initialization is supported.
 * @note Report-buffer allocation, semaphore creation and task creation
 *       failures are fatal via crit_err_exit(). There is no deinitialization;
 *       allocated objects have system lifetime.
 * @note Calling this function does not itself wait for a valid measurement.
 */
void init_mlx90393_stm(void);

#if MLX90393_REPORT_FN == 1
/**
 * @brief Register or clear the formatted-sample reporting callback.
 *
 * Invoked from the MLX90393 acquisition task after each successful RM
 * read, with a NUL-terminated text report. The callback returns TRUE on
 * success; FALSE increments the report_fn_err_cnt diagnostic counter.
 * The STM does not retry failed callbacks or reset the sensor because of
 * a callback failure. Passing NULL disables callback delivery.
 *
 * @param[in] report_fn Function pointer or NULL to disable reporting.
 * @pre Call during startup before the acquisition task can report data.
 * @note The text buffer is owned by the STM and reused for later reports.
 *       Copy the string if it must survive beyond the callback invocation.
 * @note Callback executes in FreeRTOS task context, not ISR context.
 *       Registration is not synchronized with concurrent callback execution;
 *       avoid changing the callback while measurements are running.
 */
void mlx90393_stm_set_report_fn(boolean_t (*report_fn)(const char *));
#endif

#endif
