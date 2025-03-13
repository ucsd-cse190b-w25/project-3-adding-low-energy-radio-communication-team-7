/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
// #include "ble_commands.h"
#include "ble.h"
#include "i2c.h"
#include "leds.h"
#include "lsm6dsl.h"
#include "timer.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MOTION_THRESHOLD 5000
#define LOST_TIME 1
#define SECONDS 10
#define RATE 10000

int dataAvailable = 0;

SPI_HandleTypeDef hspi3;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI3_Init(void);

volatile uint32_t time = 0;          // Time the device has been still
volatile uint8_t lost_mode = 0;      // Flag for lost mode
volatile uint8_t interrupt_flag = 0; // Flag for timer interrupt

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_SPI3_Init();

    // RESET BLE MODULE
    HAL_GPIO_WritePin(BLE_RESET_GPIO_Port, BLE_RESET_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(BLE_RESET_GPIO_Port, BLE_RESET_Pin, GPIO_PIN_SET);

    // initialize all functions
    ble_init();
    // timer_init(TIM2);
    // timer_set_ms(TIM2, RATE);
    lptim_init(LPTIM1);
    lptim_set_ms(RATE);
    i2c_init();
    lsm6dsl_init();

    HAL_Delay(10);

    // Be careful with ports containing wake-up pins

    // __HAL_RCC_GPIOC_CLK_DISABLE();
    // __HAL_RCC_GPIOD_CLK_DISABLE();
    // __HAL_RCC_GPIOE_CLK_DISABLE();

    // Re-enable after wakeup if needed

    __HAL_RCC_MSI_RANGE_CONFIG(RCC_MSIRANGE_7);
    // start discoverablity at 0
    setDiscoverability(0);
    __HAL_RCC_MSI_RANGE_CONFIG(RCC_MSIRANGE_0);

    uint8_t nonDiscoverable = 1; // flag to check if the device is discoverable

    while (1)
    {
        if (!nonDiscoverable && HAL_GPIO_ReadPin(BLE_INT_GPIO_Port, BLE_INT_Pin))
        {
        	__HAL_RCC_MSI_RANGE_CONFIG(RCC_MSIRANGE_7);
            catchBLE();
            __HAL_RCC_MSI_RANGE_CONFIG(RCC_MSIRANGE_0);
        }
        if (interrupt_flag) // check if the timer interrupt has occurred
        {
            uint32_t catch_time = time;
            interrupt_flag = 0; // clear the interrupt flag	debug
            __HAL_RCC_MSI_RANGE_CONFIG(RCC_MSIRANGE_7);
            check_movement();   // check for movement
             __HAL_RCC_MSI_RANGE_CONFIG(RCC_MSIRANGE_0);

            if (lost_mode) // check if the device is in lost mode
            {
                if (nonDiscoverable) // check if the device is not discoverable
                {
                	__HAL_RCC_MSI_RANGE_CONFIG(RCC_MSIRANGE_7);
                    setDiscoverability(1); // set the device to discoverable
                    nonDiscoverable = 0;   // set the flag to 0
                    __HAL_RCC_MSI_RANGE_CONFIG(RCC_MSIRANGE_0);
                }
                __HAL_RCC_MSI_RANGE_CONFIG(RCC_MSIRANGE_0);
                //  Send a string to the NORDIC UART service, remember to not include the newline
                unsigned char test_str[20]; // buffer to store the string

                char time_str[10];                     // buffer to store the time
                int sec = (catch_time * SECONDS) - 60; // calculate the time in seconds
                // if (sec % 10 == 0 && catch_time % SECONDS == 0)
                //{
                sprintf(time_str, "%d", sec);        // convert the time to a string
                strcpy((char *)test_str, "TURTLE "); // copy the string to the buffer
                strcat((char *)test_str, time_str);  // concatenate the time to the string
                strcat((char *)test_str, " seconds");

                __HAL_RCC_MSI_RANGE_CONFIG(RCC_MSIRANGE_7);
                updateCharValue(NORDIC_UART_SERVICE_HANDLE, READ_CHAR_HANDLE, 0,
                                sizeof(test_str) - 1, test_str);
                __HAL_RCC_MSI_RANGE_CONFIG(RCC_MSIRANGE_0);
                //}
            }
            else if (!nonDiscoverable) // check if the device is not in lost mode
            {
                //				leds_set(0b00);
            	__HAL_RCC_MSI_RANGE_CONFIG(RCC_MSIRANGE_7);
                disconnectBLE();       // disconnect the BLE
                setDiscoverability(0); // set the device to non discoverable
                standbyBle();
                nonDiscoverable = 1; // set the flag to 1
                __HAL_RCC_MSI_RANGE_CONFIG(RCC_MSIRANGE_0);
            }
        }

        // Wait for interrupt, only uncomment if low power is needed
        __HAL_RCC_MSI_RANGE_CONFIG(RCC_MSIRANGE_0);
        HAL_SuspendTick();
        __HAL_RCC_GPIOA_CLK_DISABLE();
        __HAL_RCC_GPIOB_CLK_DISABLE();
        __HAL_RCC_SPI3_CLK_DISABLE();

//        PWR->CR1 |= PWR_CR1_LPR;
//        interrupt_flag = 0; // clear the interrupt flag	debug
//        __WFI();
         HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);

        HAL_ResumeTick();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_SPI3_CLK_ENABLE();
        __HAL_RCC_MSI_RANGE_CONFIG(RCC_MSIRANGE_7);
    }
}

/**
 * @brief System Clock Configuration
 * @attention This changes the System clock frequency, make sure you reflect that change in your
 * timer
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
     */
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.MSIState = RCC_MSI_ON;
    RCC_OscInitStruct.MSICalibrationValue = 0;
    // This lines changes system clock frequency
    RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_7;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief SPI3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI3_Init(void)
{

    /* USER CODE BEGIN SPI3_Init 0 */

    /* USER CODE END SPI3_Init 0 */

    /* USER CODE BEGIN SPI3_Init 1 */

    /* USER CODE END SPI3_Init 1 */
    /* SPI3 parameter configuration*/
    hspi3.Instance = SPI3;
    hspi3.Init.Mode = SPI_MODE_MASTER;
    hspi3.Init.Direction = SPI_DIRECTION_2LINES;
    hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi3.Init.NSS = SPI_NSS_SOFT;
    hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi3.Init.CRCPolynomial = 7;
    hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
    if (HAL_SPI_Init(&hspi3) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN SPI3_Init 2 */

    /* USER CODE END SPI3_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /* USER CODE BEGIN MX_GPIO_Init_1 */
    /* USER CODE END MX_GPIO_Init_1 */

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIO_LED1_GPIO_Port, GPIO_LED1_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(BLE_CS_GPIO_Port, BLE_CS_Pin, GPIO_PIN_SET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(BLE_RESET_GPIO_Port, BLE_RESET_Pin, GPIO_PIN_SET);

    /*Configure GPIO pin : BLE_INT_Pin */
    GPIO_InitStruct.Pin = BLE_INT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BLE_INT_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : GPIO_LED1_Pin BLE_RESET_Pin */
    GPIO_InitStruct.Pin = GPIO_LED1_Pin | BLE_RESET_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : BLE_CS_Pin */
    GPIO_InitStruct.Pin = BLE_CS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(BLE_CS_GPIO_Port, &GPIO_InitStruct);

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

    /* USER CODE BEGIN MX_GPIO_Init_2 */
    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

// use the accelerometer output to check for movement
void check_movement()
{
    static int16_t prev_x = 0, prev_y = 0, prev_z = 0; // previous accelerometer values
    int16_t x, y, z;

    lsm6dsl_read_xyz(&x, &y, &z); // Read the accelerometer values

    int deltaX = x - prev_x; // calculate the change in x
    int deltaY = y - prev_y; // calculate the change in y
    int deltaZ = z - prev_z; // calculate the change in z

    if ((abs(deltaX) > MOTION_THRESHOLD) || (abs(deltaY) > MOTION_THRESHOLD) ||
        (abs(deltaZ) > MOTION_THRESHOLD)) // check if the change in any of the axes is greater
                                          // than the threshold
    {
        time = 0;      // set the time the device has been still to 0
        lost_mode = 0; // reset the lost mode flag
    }
    else
    {
        if (time >= LOST_TIME &&
            !lost_mode) // check if the device has been still for a certain amount of time
        {
            lost_mode = 1;
        }
    }

    // update the previous values
    prev_x = x;
    prev_y = y;
    prev_z = z;
}

// timer interrupt handler
void TIM2_IRQHandler()
{
    if (TIM2->SR & TIM_SR_UIF) // check if the update interrupt flag is set
    {
        TIM2->SR &= ~TIM_SR_UIF; // clear the update interrupt flag
        time++;                  // increment the time the device has been still
        interrupt_flag = 1;
    }
};

// LPTIM1 interrupt handler
void LPTIM1_IRQHandler()
{
    if (LPTIM1->ISR & LPTIM_ISR_ARRM) // Check if autoreload match flag is set
    {
        LPTIM1->ICR |= LPTIM_ICR_ARRMCF; // Clear the flag
        time++;                          // Increment the time the device has been still
        interrupt_flag = 1;              // Set the interrupt flag for the main loop
    }
}

// write function for printf
int _write(int file, char *ptr, int len)
{
    int i = 0;
    for (i = 0; i < len; i++)
    {
        ITM_SendChar(*ptr++);
    }
    return len;
}
