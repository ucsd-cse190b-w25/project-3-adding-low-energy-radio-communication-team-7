/*
 * lsm6dsl.h
 *
 */

/* Include type definitions */
#include <stm32l475xx.h>

void lsm6dsl_init();

void lsm6dsl_read_xyz(int16_t* x, int16_t* y, int16_t* z);