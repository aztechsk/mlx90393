/*
 * mlx90393_stm.h
 *
 * Autors: Jan Rusnak.
 * (c) 2025 AZTech.
 */

#ifndef MLX90393_STM_H
#define MLX90393_STM_H

/**
 * init_mlx90393_stm
 */
void init_mlx90393_stm(void);

#if MLX90393_REPORT_FN == 1
void mlx90393_stm_set_report_fn(boolean_t (*report_fn)(const char *));
#endif

#endif
