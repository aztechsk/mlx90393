/*
 * mlx90393_stm_single.h
 *
 * Autors: Jan Rusnak.
 * (c) 2026 AZTech.
 */

#ifndef MLX90393_STM_SINGLE_H
#define MLX90393_STM_SINGLE_H

#include <gentyp.h>
#include "sysconf.h"

/**
 * @file mlx90393_stm_single.h
 * @brief FreeRTOS periodic single-measurement MLX90393 state machine using DRDY.
 *
 * This is an alternative STM over the same mlx90393.c low-level driver.
 * mlx90393_stm_single.c is compiled only when
 * MLX90393_SENSOR_MODE == MLX90393_SINGLE_MODE. The tested burst STM in
 * mlx90393_stm.c remains independent and unchanged. Exactly one STM mode
 * must be active per sensor instance. MLX90393_SINGLE_MODE must be defined
 * with a value distinct from MLX90393_INTR_MODE in sysconf.h.
 *
 * @section mlx90393_single_setup Sensor setup
 *
 * Like the burst STM, this module initializes SPI bus 0, mode 3, 8 bits,
 * 10 MHz, CS1 on GPIO controller 1/pin 14, and the board-defined DRDY
 * input MLX90393_INT_CONT / MLX90393_INT_PIN. The DRDY input uses a
 * level-high interrupt and a binary FreeRTOS semaphore.
 *
 * After EX/RT/EX reset and USER1 probe it writes the following RAM
 * registers and refreshes the LL conversion cache:
 * - CFG0: HALLCONF=0xC, GAIN_SEL from the range profile, no BIST or
 *   Z_SERIES; preserves ANA_RESERVED_LOW trimming bits.
 * - CFG1: BDR=0 (unused in SM mode), BURST_SEL=0xF, configured TCMP_EN,
 *   EXT_TRIG=0, WOC_DIFF=0, COMM_MODE=2 (SPI only), TRIG_INT_SEL=1.
 * - CFG2: OSR, DIG_FILT, OSR2 and RES_X/Y/Z for the range profile.
 * - OFFSET_X/Y/Z: neutral value 0x8000 only when TCMP_EN=1;
 *   with TCMP_EN=0 the reset defaults are left unchanged.
 *
 * BDR=0 does not enable continuous burst: the STM issues only SM commands,
 * never SB. Every SM command selects T, X, Y and Z (mask 0xF). After a
 * conversion the device returns to IDLE; the next measurement is started
 * by a separate SM command from the MCU.
 *
 * @section mlx90393_single_config Build-time settings (sysconf.h)
 *
 * - MLX90393_SINGLE_MODE: a unique mode identifier defined in sysconf.h.
 * - MLX90393_SENSOR_MODE: select MLX90393_SINGLE_MODE.
 * - MLX90393_MEASURE_PERIOD_MS: required whole-millisecond interval
 *   between SM command starts, in the range 1..1000 ms (10 ms = 100 Hz).
 * - MLX90393_RANGE_PROFILE: MLX90393_RANGE_5MT (default),
 *   MLX90393_RANGE_25MT or MLX90393_RANGE_50MT; these presets configure
 *   the digital XY range, not the physical saturation limit.
 * - MLX90393_OSR: magnetic ADC oversampling 0..3 (default 0).
 * - MLX90393_DIG_FILT: digital filtering 0..7 (default 4).
 * - MLX90393_OSR2: temperature oversampling 0..3 (default 0).
 * - MLX90393_TCMP_EN: 0 (default) or 1; enables on-chip magnetic
 *   sensitivity temperature compensation using factory-trimmed coefficients.
 *   With TCMP_EN=1, neutral OFFSET_X/Y/Z registers are programmed to 0x8000;
 *   with TCMP_EN=0, they are not written.
 * - MLX90393_HARDIRON_EN: required, either 0 or 1. If enabled,
 *   MLX90393_HARDIRON_X_NT, _Y_NT and _Z_NT are required int32_t
 *   offsets. The STM subtracts offsets from converted nT before
 *   MLX90393_REPORT_FLIP_X is applied.
 * - MLX90393_TASK_STACK_SIZE and MLX90393_TASK_PRIO: FreeRTOS task
 *   stack and priority, from project configuration.
 * - MLX90393_REPORT_SI, MLX90393_REPORT_FN, MLX90393_REPORT_FLIP_X,
 *   MLX90393_REPORT_MSG_NUM: same text reporting options as burst STM.
 *
 * HALLCONF=0xC forbids (OSR,DIG_FILT)=(0,0), (0,1), (1,0). Invalid
 * settings and periods shorter than the nominal conversion plus 140 us
 * DRDY-read delay are rejected at build time. A period must be an exact
 * integer number of FreeRTOS ticks; otherwise compilation fails. The
 * STM cannot guarantee the deadline when reporting, SPI transfers or
 * higher-priority tasks consume the remaining execution time.
 *
 * With TCMP_EN=0, RES=0/1 magnetic RAW values are signed two's-complement
 * values and use the 25 C sensitivity reference. With TCMP_EN=1, they are
 * unsigned values centered at 0x8000 and use the 35 C sensitivity reference.
 * The common LL conversion cache handles both formats; measurement timing
 * is unchanged because the STM already reads T/X/Y/Z in each conversion.
 * The separate software hard-iron correction works in both TCMP modes.
 *
 * @section mlx90393_single_timing Scheduling and interrupt handling
 *
 * The task uses vTaskDelayUntil() and an absolute tick schedule rather
 * than adding the measurement/processing time to the requested period.
 * When a deadline was missed, the STM skips all elapsed sample slots
 * and waits for the next scheduled slot; it never starts back-to-back
 * measurements to catch up. The "mlxs" diagnostic command reports
 * sm_start_cnt, sm_read_cnt and missed_period_cnt, as well as ISR
 * timeouts, unexpected DRDY levels, resets and optional callback errors.
 * The legacy diagnostic field intr_pin_low counts both unexpected DRDY HIGH
 * before SM (expected LOW) and DRDY LOW after ISR wakeup (expected HIGH).
 *
 * Before each SM command the STM drains the old semaphore, verifies
 * DRDY LOW, enables the level-high GPIO interrupt, then sends SM. The
 * ISR disables further interrupts and wakes the acquisition task. The
 * task waits for DRDY with a timeout derived from the nominal conversion
 * time plus 20 ms, delays 140 us after the interrupt, verifies DRDY HIGH
 * and reads T/X/Y/Z with RM. RM deasserts DRDY and the STM reports data.
 * Timeout and SPI/status errors cause a reset/reconfiguration after 5 s.
 * No measurements are queued, and an extended report callback can cause
 * missed scheduled periods (but never overlapping SM commands).
 *
 * At OSR=0, DIG_FILT=4, OSR2=0 with T/X/Y/Z, the nominal SM time is
 * 220 + 360 + 3*1219 + 259 + 100 = 4596 us. A 10 ms period leaves
 * 5404 us before the next scheduled SM, of which 140 us is reserved for
 * post-DRDY settling; remaining time is for SPI, scheduling and reporting.
 * The time estimates are nominal and include no guarantee of zero jitter.
 *
 * @section mlx90393_single_reporting Reporting
 *
 * Valid magnetic values are converted RAW -> nT, optionally corrected
 * for fixed hard-iron offsets, optionally inverted on X, and formatted
 * as signed uT with three fractional digits. Temperature is printed in
 * degrees C with three fractional digits. The callback receives the
 * same NUL-terminated text as the logger, with invalid fields shown as
 * "----.---". It is invoked synchronously from the acquisition task.
 *
 * Reference: Melexis MLX90393 datasheet Rev. 012 (18 November 2025),
 * sections 12, 14.2 and 15.
 */

/**
 * @brief Initialize and start periodic single-measurement acquisition.
 *
 * Configures SPI, creates the binary DRDY semaphore, registers the "mlxs"
 * diagnostic command and creates one fixed FreeRTOS acquisition task.
 * The task performs reset, probe, sensor configuration, scheduled SM
 * commands, interrupt-driven RM readout, conversion and reporting.
 *
 * @pre Call once from initialization/task context, not an ISR. The target
 *      must provide the configured SPI/GPIO HAL, board DRDY mapping,
 *      FreeRTOS facilities and the relevant sysconf.h settings.
 * @note Memory, semaphore or task creation failure invokes
 *       crit_err_exit(); resources have system lifetime.
 * @note Available only if MLX90393_SENSOR_MODE == MLX90393_SINGLE_MODE.
 */
void init_mlx90393_stm_single(void);

#if MLX90393_REPORT_FN == 1
/**
 * @brief Install or disable the formatted-sample report callback.
 *
 * The callback is invoked by the acquisition task after each successful
 * RM read. A FALSE return increments report_fn_err_cnt; callback errors
 * do not reset the STM. Passing NULL disables callback delivery.
 *
 * @param[in] report_fn Callback function or NULL.
 * @pre Register before acquisition can report; registration does not
 *      synchronize with concurrent callback invocation.
 * @note The NUL-terminated text belongs to the STM and is reused at the
 *       next report. Copy it within the callback to retain it.
 * @note Callback runs in task context, not ISR context. Only the selected
 *       STM defines this symbol.
 */
void mlx90393_stm_set_report_fn(boolean_t (*report_fn)(const char *));
#endif

#endif
