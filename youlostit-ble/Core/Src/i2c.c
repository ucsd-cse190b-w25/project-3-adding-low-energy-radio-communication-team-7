/*
 * leds.c
 *
 */

/* Include memory map of our MCU */
#include <stm32l475xx.h>

void i2c_init()
{
    // set clocks
    RCC->APB1ENR1 |= RCC_APB1ENR1_I2C2EN;
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;

    // clear pe bit to disable the peripheral
    I2C2->CR1 &= ~I2C_CR1_PE;

    // configure the timing register for 100 kHz
    I2C2->TIMINGR &= ~I2C_TIMINGR_PRESC; // set the prescaler to 0

    I2C2->TIMINGR &= ~I2C_TIMINGR_SCLDEL;
    I2C2->TIMINGR |= (0x4UL << I2C_TIMINGR_SCLDEL_Pos); // set the SCLDEL to 4

    I2C2->TIMINGR &= ~I2C_TIMINGR_SDADEL;
    I2C2->TIMINGR |= (0x2UL << I2C_TIMINGR_SDADEL_Pos); // set the SDADEL to 2

    I2C2->TIMINGR &= ~I2C_TIMINGR_SCLH;
    I2C2->TIMINGR |= (0xFUL << I2C_TIMINGR_SCLH_Pos); // set the SCLH to 15

    I2C2->TIMINGR &= ~I2C_TIMINGR_SCLL;
    I2C2->TIMINGR |= (0x13UL << I2C_TIMINGR_SCLL_Pos); // set the SCLL to 19

    // set PB10 and PB11 to alternate function mode
    GPIOB->MODER &= ~(GPIO_MODER_MODE10 | GPIO_MODER_MODE11);
    GPIOB->MODER |= (GPIO_MODER_MODE10_1 | GPIO_MODER_MODE11_1);

    // set alternate function to I2C2 (AF4) for PB10 and PB11
    GPIOB->AFR[1] &= ~(GPIO_AFRH_AFSEL10 | GPIO_AFRH_AFSEL11);
    GPIOB->AFR[1] |= (0x4 << GPIO_AFRH_AFSEL10_Pos | 0x4 << GPIO_AFRH_AFSEL11_Pos);

    // set pe bit to enable the peripheral
    I2C2->CR1 |= I2C_CR1_PE;
};

uint8_t i2c_transaction(uint8_t address, uint8_t dir, uint8_t *data, uint8_t len)
{   
    // wait until I2C is not busy
    while (I2C2->ISR & I2C_ISR_BUSY);

    // set secondary address 
    I2C2->CR2 &= ~I2C_CR2_SADD;
    I2C2->CR2 |= (address << 1);

    // set read/write bit
    I2C2->CR2 &= ~I2C_CR2_RD_WRN;
    I2C2->CR2 |= (dir << I2C_CR2_RD_WRN_Pos);

    // set the number of bytes to transfer
    I2C2->CR2 &= ~I2C_CR2_NBYTES;
    I2C2->CR2 |= (len << I2C_CR2_NBYTES_Pos);

    I2C2->CR2 |= I2C_CR2_AUTOEND; // set autoend so that hardware automatically sends stop bit

    // start the transfer
    I2C2->CR2 |= I2C_CR2_START;

    // write or read data
    if (!dir)
    {

        for (int i = 0; i < len; i++)
        {
            // wait for TXIS to be set
            while (!(I2C2->ISR & I2C_ISR_TXIS));
            I2C2->TXDR = data[i];
        }
    }
    else
    {

        for (int i = 0; i < len; i++)
        {
            // wait for RXNE to be set
            while (!(I2C2->ISR & I2C_ISR_RXNE));
            data[i] = I2C2->RXDR;
        }
    }

    // wait for transfer to complete
    while (!(I2C2->ISR & I2C_ISR_STOPF));


    return 0;
};
