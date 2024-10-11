/*
 * test_i2c.c
 *
 *  Created on: Sep 22, 2024
 *      Author: kaushikbalantrapu
 */

#include "src/drivers/i2c.h"


void test_i2c()
{
    // also sets slave address to default (0x68)
    i2c_init();

    uint8_t data_packet[] = {0x13, 0x15};
    uint8_t rx_buffer[3];
    // year register
    uint8_t register_address = 0x03;
    i2c_result current_result;
    uint16_t i = 0;
    while (i < 1000)
    {
        //current_result = i2c_write(register_address, data_packet, 2);
        // READ IS BROKEN
        current_result = i2c_read(register_address, rx_buffer, 3);
        i++;
    }

    while (1)
    {

    }

}

/*
int main(void)
{
    test_i2c();
}
*/
