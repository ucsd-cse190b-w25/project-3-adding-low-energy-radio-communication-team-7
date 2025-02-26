/*
 * timer.c
 *
 *  Created on: Oct 5, 2023
 *      Author: schulman
 */

#include "timer.h"

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
    timer->PSC = 4000 - 1;

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
