/*
 * i2c.h
 *
 *  Created on: Aug 15, 2024
 *      Author: kaushikbalantrapu
 */

#ifndef I2C_H
#define I2C_H

#include <stdint.h>

typedef enum
{
    TX_REGISTER_ADDRESS_STATE,
    SWITCH_TO_RX_STATE,
    TX_STATE,
    RX_STATE,
    IDLE_STATE
} i2c_state;

typedef enum
{
    I2C_SUCCESS,
    I2C_START_ERROR,
    I2C_STOP_ERROR,
    I2C_TX_ERROR,
    I2C_RX_ERROR,
    I2C_TIMEOUT_ERROR,
    I2C_NACK_ERROR,
} i2c_result;

void i2c_init(void);
//void i2c_set_slave_address(uint8_t slave_address);
i2c_result i2c_write(uint8_t register_address, uint8_t *data, uint8_t data_size_in_bytes);
i2c_result i2c_read(uint8_t register_address, uint8_t *data, uint8_t data_size_in_bytes);

#endif /* I2C_H */
