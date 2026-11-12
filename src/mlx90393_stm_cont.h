/*
 * mlx90393_stm_cont.h
 *
 * Autors: Jan Rusnak.
 * (c) 2026 AZTech.
 */

#ifndef MLX90393_STM_CONT_H
#define MLX90393_STM_CONT_H

#include <gentyp.h>
#include "sysconf.h"

#ifndef MLX90393_REPORT_SAMPLE_FN
#define MLX90393_REPORT_SAMPLE_FN 0
#endif
#if MLX90393_REPORT_SAMPLE_FN == 1
#include "mlx90393.h"
#endif

/**
 * @file mlx90393_stm_cont.h
 * @brief FreeRTOS MLX90393 continuous-burst state machine using the DRDY interrupt.
 *
 * This is a third, independent STM over mlx90393.c. The project selects it
 * with MLX90393_SENSOR_MODE == MLX90393_CONT_MODE in sysconf.h, defining
 * MLX90393_CONT_MODE as a distinct mode identifier. The periodic burst STM
 * (mlx90393_stm.c) and single-measurement STM (mlx90393_stm_single.c) remain
 * unchanged. Exactly one STM must be selected for a sensor instance.
 *
 * @section mlx90393_cont_setup Sensor configuration
 *
 * The STM initializes SPI bus 0, mode 3, 8 bits, 10 MHz, CS1 on GPIO
 * controller 1/pin 14, and the board-defined MLX90393_INT_CONT / INT_PIN
 * input as an unfiltered, level-high DRDY interrupt. One FreeRTOS task and
 * one binary semaphore perform all acquisition and readout.
 *
 * After soft reset and USER1 communication probe, it writes CFG0/1/2:
 * - CFG0: HALLCONF=0xC, GAIN_SEL from the range profile, BIST=0 and
 *   Z_SERIES=0, preserving the ANA_RESERVED_LOW factory-trim bits.
 * - CFG1: BDR=0, BURST_SEL=0xF (T/X/Y/Z), configured TCMP_EN, EXT_TRIG=0,
 *   WOC_DIFF=0, COMM_MODE=2 (SPI only) and TRIG_INT_SEL=1 (DRDY).
 * - CFG2: OSR, DIG_FILT, OSR2 and RES_X/Y/Z from the selected profile.
 * - OFFSET_X/Y/Z: 0x8000 (neutral offset) only with TCMP_EN=1;
 *   with TCMP_EN=0 the reset defaults are left unchanged.
 *
 * The conversion cache is refreshed, then one SB command starts automatic
 * continuous measurements. No SM commands or periodic MCU timer are used.
 * The sensor runs without the BDR>0 standby interval until an error/reset.
 *
 * @section mlx90393_cont_config Build configuration (sysconf.h)
 *
 * - MLX90393_CONT_MODE: a mode identifier provided by the application.
 * - MLX90393_SENSOR_MODE: select MLX90393_CONT_MODE.
 * - MLX90393_RANGE_PROFILE: MLX90393_RANGE_5MT (default), _25MT or _50MT;
 *   these are digital XY range presets, not physical saturation limits.
 * - MLX90393_OSR: magnetic ADC oversampling, 0..3 (default 0).
 * - MLX90393_DIG_FILT: digital filtering, 0..7 (default 4).
 * - MLX90393_OSR2: temperature oversampling, 0..3 (default 0).
 * - MLX90393_TCMP_EN: 0 (default) or 1; enables on-chip sensitivity
 *   temperature compensation using factory-trimmed coefficients.
 *   With TCMP_EN=1 the STM programs neutral OFFSET_X/Y/Z to 0x8000;
 *   with TCMP_EN=0 these registers are not written.
 * - MLX90393_HARDIRON_EN: required, 0 or 1. When enabled, signed int32_t
 *   MLX90393_HARDIRON_X_NT, _Y_NT, _Z_NT offsets must be defined. They
 *   are subtracted in software after RAW-to-nT conversion and before the
 *   optional MLX90393_REPORT_FLIP_X operation. Hardware OFFSET registers
 *   programmed at TCMP_EN=1 are neutral, not a hard-iron calibration.
 * - MLX90393_TASK_STACK_SIZE, MLX90393_TASK_PRIO: FreeRTOS task settings.
 * - MLX90393_REPORT_SI, MLX90393_REPORT_FN, MLX90393_REPORT_FLIP_X and
 *   MLX90393_REPORT_MSG_NUM: identical text reporting options to other STMs.
 * - MLX90393_REPORT_SAMPLE_FN: optional (default 0) callback for original
 *   RAW words and corrected nT/mC values.
 *
 * The HALLCONF=0xC restrictions on OSR/DIG_FILT are enforced at compile
 * time. With TCMP_EN=0, RES=0/1 magnetic RAW data are signed and use the
 * 25 C sensitivity reference. With TCMP_EN=1 they are unsigned, centered
 * at 0x8000, and use the 35 C reference. The common LL conversion cache
 * handles both formats; timing is unchanged because T/X/Y/Z are measured
 * in both configurations. Software hard-iron correction is independent
 * of TCMP_EN. MLX90393_BURST_RATE_HZ and MLX90393_MEASURE_PERIOD_MS have
 * no effect in this STM; BDR is always zero.
 *
 * @section mlx90393_cont_timing Continuous burst timing
 *
 * According to Melexis MLX90393 datasheet Rev.012, Table 8, at BDR=0
 * the nominal repeated conversion interval is:
 *
 * T_CONT = 3*T_CONVM(OSR,DIG_FILT) + T_CONVT(OSR2) + T_CONV_END.
 *
 * At OSR=0, DIG_FILT=4, OSR2=0, T/X/Y/Z:
 * T_CONT = 3*1219 + 259 + 100 = 4016 us, approximately 249 samples/s.
 * The first conversion after SB may also incur startup/active overhead.
 * Timing is nominal; internal oscillator tolerances affect the actual rate.
 *
 * @section mlx90393_cont_readout Interrupt handling and data loss
 *
 * The ISR disables its level-high interrupt and gives a binary semaphore.
 * The task waits for DRDY, delays 140 us (T_INT_RDY), then reads T/X/Y/Z
 * using one RM command and reports the converted values. After reporting,
 * the GPIO interrupt is re-enabled for the next available measurement.
 * The wait timeout is the rounded-up nominal conversion time plus 200 ms.
 * Timeout or SPI/status errors cause a reset/reconfiguration after 5 s.
 *
 * When a new conversion completes before the previous data is read,
 * DRDY can pulse LOW for 10 us and then become HIGH again. If the task
 * observes DRDY LOW after being signaled, it skips that RM, increments
 * drdy_low_cnt and re-enables the interrupt to wait for a fresh HIGH level;
 * it does not treat the brief pulse as a sensor failure.
 *
 * The MLX90393 has no FIFO. Slow processing or reporting may overwrite
 * intermediate results. The read counter records successful RM commands,
 * not every sensor conversion; missing samples cannot be counted exactly
 * without a separate hardware timing reference. The level-high GPIO HAL
 * must signal an already-HIGH DRDY level when interrupts are re-enabled.
 * The "mlxs" command reports successful reads, interrupt timeouts,
 * observed low-level events, STM resets and optional callback failures.
 *
 * @section mlx90393_cont_reporting Reporting
 *
 * Readings are converted to nT and mC; optional software hard-iron
 * offsets and X sign inversion are applied before reporting. The
 * binary sample callback receives original RAW and corrected nT/mC data.
 * Text reports contain signed X/Y/Z in uT with three decimals and temperature
 * in degrees C with three decimals, as in the other MLX90393 STMs.
 * Avoid per-sample serial logging or slow callbacks when attempting to
 * capture the maximum continuous-burst throughput (~249 Hz by default).
 *
 * Reference: Melexis MLX90393 datasheet, Rev.012 (18 November 2025),
 * sections 12, 14.1, 15 and 16.
 */

/**
 * @brief Initialize and start the continuous-burst MLX90393 acquisition task.
 *
 * Initializes SPI, creates the binary semaphore, allocates the optional
 * report buffer, registers the "mlxs" diagnostic command and creates one
 * system-lifetime FreeRTOS task. That task resets, probes and configures
 * the sensor before starting continuous-burst acquisition.
 *
 * @pre Call once during system startup, from task context. HAL, board and
 *      FreeRTOS facilities must already be initialized.
 * @note Allocation and task-creation failures are fatal via crit_err_exit().
 * @note Initialization does not wait for the first measurement.
 */
void init_mlx90393_stm_cont(void);

#if MLX90393_REPORT_FN == 1
/**
 * @brief Register or clear the formatted-measurement reporting callback.
 *
 * Invoked from the acquisition task after each successful RM. The
 * NUL-terminated buffer is STM-owned and reused by subsequent reports.
 * The callback returns TRUE on success; FALSE increments the diagnostic
 * report_fn_err_cnt but does not stop acquisition. NULL disables delivery.
 *
 * @param[in] report_fn Callback pointer, or NULL to disable it.
 * @pre Register during startup, before the acquisition task can report.
 * @note The callback runs in task context, not ISR context. It must not
 *       retain the supplied buffer or block for long; registration is not
 *       synchronized with concurrent callback execution.
 */
void mlx90393_stm_set_report_fn(boolean_t (*report_fn)(const char *));
#endif

#if MLX90393_REPORT_SAMPLE_FN == 1
/**
 * @brief Register or clear the RAW plus converted-sample callback.
 *
 * Called synchronously by the acquisition task after each successful RM
 * read and after all configured software corrections. @c sample->raw
 * retains original sensor codes and validity flags; @c x_nt, @c y_nt and
 * @c z_nt contain converted nT values after optional hard-iron subtraction
 * and X sign inversion. @c t_mc contains converted temperature in mC.
 * INT32_MIN marks an invalid or unrepresentable converted value.
 * A FALSE return increments sample_fn_err_cnt; it does not reset the STM.
 * NULL disables callback delivery.
 *
 * @param[in] sample_fn Callback pointer or NULL to disable delivery.
 * @pre Register during startup before the task starts reporting. Changing
 *      the callback while acquisition is running is not synchronized.
 * @note Called in FreeRTOS task context, not ISR context. The pointer is
 *       valid only until return: copy the sample to retain it. Keep callback
 *       execution bounded/nonblocking to avoid missed sensor conversions.
 * @note Only the selected STM provides this symbol.
 */
void mlx90393_stm_set_sample_fn(boolean_t (*sample_fn)(const struct mlx90393_sample *));
#endif

#endif
