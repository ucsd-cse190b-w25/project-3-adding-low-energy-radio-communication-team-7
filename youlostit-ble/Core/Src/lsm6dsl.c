/*
 * lsm6dsl.c
 *
 */

/* Include type definitions */
#include <stm32l475xx.h>

void lsm6dsl_init()
{

    uint8_t ctrl1_xl[2] = {0x10, 0x4c};		// 104 Hz for timer, +-8g
//    uint8_t int1_ctrl[2] = {0x0D, 0x01};	// TODO: change interrupt config

    i2c_transaction(0x6a, 0, ctrl1_xl, 2); // write CTRL1_XL = 40h

//    i2c_transaction(0x6a, 0, int1_ctrl, 2); // write INT1_CTRL = 01h
};

void lsm6dsl_setup_interrupts()	//TODO: i2c addresses
{
	//turn on gyroscope
	uint8_t ctrl2_g[2] = {0x11, 0x40};
	i2c_transaction(0x6a, 0, ctrl2_g, 2);
	// set inactivity time threshold
	uint8_t wake_up_dur[2] = {0x5c, 0x0c};
	i2c_transaction(0x6a, 0, wake_up_dur, 2);	// inactive dur thresh = 12 (60 seconds)
	// set gyroscope threshold
	uint8_t wake_up_ths[2] = {0x5b, 0x14};
	isc_transaction(0x6a, 0, wake_up_ths, 2);	// +-5g
	// enable interrupts
	uint8_t tap_cfg[2] = {0x58, 0xe0};
	i2c_transaction(0x6a, 0, tap_cfg, 2);		// gyro to power down mode {?}
	// drive interrupts to pin
	uint8_t md1_cfg[2] = {0x5e, 0x80};
	i2c_transaction(0x6a, 0, md1_cfg, 2);

	// change ODR_XL
}

void lsm6dsl_read_xyz(int16_t *x, int16_t *y, int16_t *z)
{

    uint8_t status[2] = {0x1E, 0};

    // wait until XLDA (bit 0) is set
    while (!(status[1] & 0x01))
    {
        i2c_transaction(0x6a, 0, status, 1);
        i2c_transaction(0x6a, 1, status + 1, 1);
    }

    // set up registers and write buffers for x, y, z
    uint8_t dataX[4] = {0x28, 0, 0x29, 0};
    uint8_t dataY[4] = {0x2A, 0, 0x2B, 0};
    uint8_t dataZ[4] = {0x2C, 0, 0x2D, 0};

    // loop through low then high registers for x, y, z
    for (int i = 0; i < 4; i = i + 2)
    {
        // write x register
        i2c_transaction(0x6a, 0, dataX + i, 1);
        // read value from x register
        i2c_transaction(0x6a, 1, dataX + i + 1, 1);

        // write y register
        i2c_transaction(0x6a, 0, dataY + i, 1);
        // read value from y register
        i2c_transaction(0x6a, 1, dataY + i + 1, 1);

        // write z register
        i2c_transaction(0x6a, 0, dataZ + i, 1);
        // read value from z register
        i2c_transaction(0x6a, 1, dataZ + i + 1, 1);
    }

    // combine high and low bytes for x, y, z
    *x = (dataX[3] << 8) | dataX[1];
    *y = (dataY[3] << 8) | dataY[1];
    *z = (dataZ[3] << 8) | dataZ[1];
};
