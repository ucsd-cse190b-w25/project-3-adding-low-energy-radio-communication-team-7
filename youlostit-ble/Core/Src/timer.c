/*
 * timer.c
 *
 *  Created on: Oct 5, 2023
 *      Author: schulman
 */

#include "timer.h"

void lptim_init(LPTIM_TypeDef *timer)
{
    // Enable LPTIM1 clock
    RCC->APB1ENR1 |= RCC_APB1ENR1_LPTIM1EN;

    // Select LSI as LPTIM clock source
    RCC->CCIPR &= ~RCC_CCIPR_LPTIM1SEL;
    RCC->CCIPR |= RCC_CCIPR_LPTIM1SEL_0; // 01: LSI selected

    // Wait for LSI to be ready
    RCC->CSR |= RCC_CSR_LSION;
    while (!(RCC->CSR & RCC_CSR_LSIRDY))
        ;

    // Disable LPTIM before configuration
    timer->CR &= ~LPTIM_CR_ENABLE;

    // Set prescaler to 128 (maximum division for lowest power)
    timer->CFGR &= ~LPTIM_CFGR_PRESC;
    timer->CFGR |=
        LPTIM_CFGR_PRESC_2 | LPTIM_CFGR_PRESC_1 | LPTIM_CFGR_PRESC_0; // 111: Prescaler = 128

    // Configure for single autoreload match mode
    timer->CFGR |= LPTIM_CFGR_WAVE;

    // Enable autoreload match interrupt
    timer->IER |= LPTIM_IER_ARRMIE;

    // Clear any pending flags
    timer->ICR |= LPTIM_ICR_ARRMCF;

    // Enable LPTIM1 interrupt in NVIC
    NVIC_EnableIRQ(LPTIM1_IRQn);
    NVIC_SetPriority(LPTIM1_IRQn, 0);

    // Enable the LPTIM
    timer->CR |= LPTIM_CR_ENABLE;
}

void timer_init(TIM_TypeDef *timer)
{
    // enable on rcc
    RCC->APB1ENR1 |= RCC_APB1SMENR1_TIM2SMEN;

    // disables the timer
    timer->CR1 &= ~TIM_CR1_CEN;

    // clear counter
    timer->CNT &= ~TIM_CNT_CNT;

    // clear auto-reload register
    timer->ARR &= ~TIM_ARR_ARR;

    // clear prescalar
    timer->PSC &= ~TIM_PSC_PSC;

    // clear uif flag
    timer->SR &= ~TIM_SR_UIF;

    // disable interrupts
    timer->DIER &= ~TIM_DIER_UIE;

    // set prescalar so that unit is ms
    timer->PSC = 8000 - 1;

    // set auto-reload register so that the timer will go off every 50ms
    timer->ARR = 50 - 1;

    // enable update interrupt
    timer->DIER |= TIM_DIER_UIE;

    // enable in nvic
    NVIC_EnableIRQ(TIM2_IRQn);

    // set priority
    NVIC_SetPriority(TIM2_IRQn, 0);

    // enable the timer
    timer->CR1 |= TIM_CR1_CEN;
}

void timer_reset(TIM_TypeDef *timer)
{
    // clear counter
    timer->CNT &= ~TIM_CNT_CNT;
}

void timer_set_ms(TIM_TypeDef *timer, uint16_t period_ms)
{
    // Stop the timer
    timer->CR1 &= ~TIM_CR1_CEN;

    // Set the auto-reload register
    timer->ARR = period_ms - 1;

    // Restart the timer
    timer->CR1 |= TIM_CR1_CEN;
}

void lptim_set_ms(uint32_t ms)
{
    // LSI frequency is approximately 32 kHz
    // With prescaler of 128, timer frequency is 32000/128 = 250 Hz
    // Each tick is 4 ms, so divide ms by 4
    uint32_t ticks = ms / 4;

    if (ticks > 0xFFFF)
        ticks = 0xFFFF; // Ensure value fits in 16-bit register

    // Disable LPTIM
    LPTIM1->CR &= ~LPTIM_CR_ENABLE;

    // Clear any pending flags
    LPTIM1->ICR |= LPTIM_ICR_ARRMCF;

    // Enable LPTIM
    LPTIM1->CR |= LPTIM_CR_ENABLE;

    // Write to ARR register
    LPTIM1->ARR = ticks;

    // Start the timer in continuous mode
    LPTIM1->CR |= LPTIM_CR_CNTSTRT;
}
