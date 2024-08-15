/*
 * i2c.h
 *
 *  Created on: Aug 15, 2024
 *      Author: kaushikbalantrapu
 */

#ifndef I2C_H
#define I2C_H


typedef enum
{
    I2C_SUCCESS,
    I2C_START_ERROR,
    I2C_STOP_ERROR,
    I2C_TX_ERROR,
    I2C_RX_ERROR,
    I2C_TIMEOUT_ERROR
} i2c_result;

void i2c_init(void);
void i2c_set_slave_address(const uint8_t slave_address);
i2c_result i2c_write(const uint8_t slave_address, const uint8_t register_address, const uint8_t *data);
i2c_result i2c_read(const uint8_t slave_address, const uint8_t register_address, uint8_t *data);


#endif /* I2C_H */
