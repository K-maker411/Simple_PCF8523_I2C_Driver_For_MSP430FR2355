/*
 * i2c.c
 *
 *  Created on: Aug 15, 2024
 *      Author: kaushikbalantrapu
 *
 *  Heavily taken from: https://dev.ti.com/tirex/explore/node?node=A__AKfFkihGP6fGq.afs94WMQ__msp430ware__IOGqZri__LATEST
 */
#include "i2c.h"
#include <msp430.h>

// total number of registers in PCF8523 (though i'll probably only be reading one byte at a time?)
#define MAX_BUFFER_SIZE 20

/**
 * TODO: i2c write
 * We can do some cool low power mode switching until something occurs
 * e.g. once start condition and slave address (along with r/w) are sent,
 * switch to low power mode and wait for interrupt for ACK
 */
const uint8_t DEFAULT_SLAVE_ADDRESS = 0x68;
const uint16_t RETRY_COUNT = UINT16_MAX;

// TODO - should tx_data_buf be volatile?
static uint8_t tx_data_buf[MAX_BUFFER_SIZE];
static volatile uint8_t tx_data_buf_index;
static volatile uint8_t tx_remaining_byte_count;

// TODO - is this necessary? or can the tx and rx use the same vars?
static volatile uint8_t rx_data_buf[MAX_BUFFER_SIZE];
static volatile uint8_t rx_data_buf_index;
static volatile uint8_t rx_remaining_byte_count;
// holds direct result from UCRXBUF temporarily
static volatile uint8_t rx_temp_byte_buf;

// state machine state variable
static volatile i2c_state current_i2c_state;
// state machine result variable
static volatile i2c_result current_i2c_result;

// register address in slave to write to/read from
// TODO - set in appropriate functions, I don't think this should be volatile but check
static uint8_t current_register_address;


// byte-by-byte copy
static void copy_array(uint8_t *source, uint8_t *destination, uint8_t num_bytes_to_copy)
{
    uint8_t copy_index = 0;
    for (copy_index = 0; copy_index < num_bytes_to_copy; copy_index++)
    {
        destination[copy_index] = source[copy_index];
    }
}

// byte-by-byte copy (works with volatile source)
static void copy_array_volatile_source(volatile uint8_t *source, uint8_t *destination, uint8_t num_bytes_to_copy)
{
    uint8_t copy_index = 0;
    for (copy_index = 0; copy_index < num_bytes_to_copy; copy_index++)
    {
        destination[copy_index] = source[copy_index];
    }
}

static void i2c_transmit_register_address_settings(void)
{
    // clear any existing TX and RX flags
    UCB0IFG &= ~UCTXIFG;
    UCB0IFG &= ~UCRXIFG;

    // enable TX and disable RX interrupts
    UCB0IE |= UCTXIE;
    UCB0IE &= ~UCRXIE;
}

// NOTE - write and read assume that slave address has already been set outside, since it is common to both
i2c_result i2c_write(uint8_t register_address, uint8_t *data, uint8_t data_size_in_bytes)
{
    current_i2c_state = TX_REGISTER_ADDRESS_STATE;
    current_register_address = register_address;

    // copy to-be-TXd data to variable (which will be loaded, byte-by-byte, into TX buffer
    copy_array(data, tx_data_buf, data_size_in_bytes);
    tx_data_buf_index = 0;
    tx_remaining_byte_count = data_size_in_bytes;

    // set rx_remaining_byte_count to 0 to ensure that we don't accidentally go into RX mode
    rx_remaining_byte_count = 0;
    rx_data_buf_index = 0;

    i2c_transmit_register_address_settings();

    // Tx mode and send start condition
    UCB0CTLW0 |= UCTR;
    UCB0CTLW0 |= UCTXSTT;

    // activate low power mode and global interrupts enable
    __bis_SR_register(LPM0_bits + GIE);

    return current_i2c_result;
}

/*
static void i2c_read_interrupt_settings(void)
{
    // clear any existing TX and RX flags
    UCB0IFG &= ~UCTXIFG;
    UCB0IFG &= ~UCRXIFG;

    // enable RX and disable TX interrupts
    UCB0IE &= ~UCTXIE;
    UCB0IE |= UCRXIE;
}*/



i2c_result i2c_read(uint8_t register_address, uint8_t *data, uint8_t data_size_in_bytes)
{
    current_i2c_state = TX_REGISTER_ADDRESS_STATE;
    current_register_address = register_address;

    // reset remaining byte count and index
    rx_remaining_byte_count = data_size_in_bytes;
    rx_data_buf_index = 0;

    // reset Tx vars (precaution!)
    tx_remaining_byte_count = 0;
    tx_data_buf_index = 0;

    //i2c_read_interrupt_settings();
    i2c_transmit_register_address_settings();
    UCB0CTLW0 |= UCTR;
    UCB0CTLW0 |= UCTXSTT;
    __bis_SR_register(LPM0_bits + GIE);

    // copy from rx_data_buf (which now holds the received data)
    // into the given data buffer
    copy_array_volatile_source(rx_data_buf, data, data_size_in_bytes);
    return current_i2c_result;
}

static void gpio_init(void)
{
    // TODO - currently hardcoding port selects, find better way to do this
    // since both eUSCI_B0 and eUSCI_B1 can be used for I2C (possibly 2 different functions?)
    P1SEL1 &= ~BIT3;            // P1.3 = SCL
    P1SEL0 |= BIT3;

    P1SEL1 &= ~BIT2;            // P1.2 = SDA
    P1SEL0 |= BIT2;

    PM5CTL0 &= ~LOCKLPM5;
}

void i2c_init(void)
{
    gpio_init();
    // put peripheral into software reset
    UCB0CTLW0 |= UCSWRST;

    // configure eUSCI_B0
    UCB0CTLW0 |= UCSSEL__SMCLK; // choose BRCLK = SMCLK = 1 MHz
    UCB0BRW = 20;               // BRCLK = 1 MHz / 20 = 50 kHz
    UCB0CTLW0 |= UCMST__MASTER; // master mode select
    UCB0CTLW0 |= UCMODE_3;      // I2C mode
    UCB0I2CSA = DEFAULT_SLAVE_ADDRESS;  // set slave address register to default slave address of PCF8523
    //UCB0CTLW1 |= UCCLTO_3;      // set clock low timeout to ~34 ms

    UCB0CTLW0 &= ~UCSWRST;
    UCB0IE |= UCNACKIE;         // enable NACK interrupt
}

void __interrupt_vec(EUSCI_B0_VECTOR) EUSCI_B0_I2C_ISR(void)
{
    // NOTE - reading from UCBxRXBUF clears UCRXIFGx flags, and writing to UCBxTXBUF clears UCTXIFGx,
    // so no need to do it explicitly - we only enable/disable the Tx/Rx enables when necessary
    switch(__even_in_range(UCB0IV, UCIV__UCBIT9IFG))
    {
        // in the case of receiving a NACK, send STOP condition, go back to idle, indicate NACK error,
        // and exit LPM
        case UCIV__UCNACKIFG:
            UCB0IFG &= ~UCNACKIFG;
            UCB0CTLW0 |= UCTXSTP;
            current_i2c_state = IDLE_STATE;
            current_i2c_result = I2C_NACK_ERROR;
            LPM0_EXIT;
            break;
        // UCBxRXBUF has received complete byte
        case UCIV__UCRXIFG0:
            // here, we are guaranteed to be in RX_STATE, so no need to check for it explicitly

            // obtain byte from UCB0RXBUF
            rx_temp_byte_buf = UCB0RXBUF;

            if (rx_remaining_byte_count > 0)
            {
                rx_data_buf[rx_data_buf_index] = rx_temp_byte_buf;
                rx_data_buf_index++;
                rx_remaining_byte_count--;
            }

            // only 1 byte left to read, so prepare stop condition
            if (rx_remaining_byte_count == 1)
            {
                UCB0CTLW0 |= UCTXSTP;
            }

            // we're done receiving, so clear Rx interrupt enable and get out of low power mode
            // also, return to idle state
            else if (rx_remaining_byte_count == 0)
            {
                UCB0IE &= ~UCRXIE;
                current_i2c_state = IDLE_STATE;
                current_i2c_result = I2C_SUCCESS;
                LPM0_EXIT;
            }

            break;

        // UCBxTXBUF is empty
        case UCIV__UCTXIFG0:
            switch(current_i2c_state)
            {
                // first, we must Tx register address
                case TX_REGISTER_ADDRESS_STATE:
                    UCB0TXBUF = current_register_address;
                    // if there are bytes to read, that means we currently want to switch to read
                    // otherwise, we don't want to read, so we currently want to transmit
                    if (rx_remaining_byte_count > 0)
                    {
                        current_i2c_state = SWITCH_TO_RX_STATE;
                    }

                    else
                    {
                        current_i2c_state = TX_STATE;
                    }
                    // register address is in the TXBUF, so we break to wait for
                    // the address to be sent out
                    break;

                case SWITCH_TO_RX_STATE:
                    // enable Rx interrupt and disable Tx interrupt, since we're
                    // switching to Rx state
                    UCB0IE |= UCRXIE;
                    UCB0IE &= ~UCTXIE;

                    // change master to receiver (|= UCTR__RX does not work here)
                    UCB0CTLW0 &= ~UCTR;
                    current_i2c_state = RX_STATE;

                    // NOTE - PCF8523 doesn't work with repeated start condition, must use stop condition
                    UCB0CTLW0 |= UCTXSTP;
                    while (UCB0CTLW0 & UCTXSTP) {}
                    UCB0CTLW0 |= UCTXSTT;


                    // edge case - if we only want to receive 1 byte (total),
                    // we need to make sure the start condition that we just prepared has been fully sent,
                    // then prepare the stop condition (which will be sent after the 1 byte is read)
                    if (rx_remaining_byte_count == 1)
                    {
                        while (UCB0CTLW0 & UCTXSTT) {}
                        UCB0CTLW0 |= UCTXSTP;
                    }
                    break;

                case TX_STATE:
                    // if there are still bytes to transfer, increment index
                    // and keep sending, decrement bytes remaining
                    if (tx_remaining_byte_count > 0)
                    {
                        UCB0TXBUF = tx_data_buf[tx_data_buf_index]; // put next bit in Tx buffer
                        tx_data_buf_index++;  // increment to next byte in data
                        tx_remaining_byte_count--;
                    }

                    // if not, Tx done, send stop condition and clear flag for UCB0TXBUF being empty (UCTXIFG0)
                    // to avoid coming back into ISR repeatedly
                    else
                    {
                        UCB0CTLW0 |= UCTXSTP;
                        UCB0IE &= ~UCTXIE;
                        // UCB0IFG &= ~UCTXIFG0;
                        current_i2c_state = IDLE_STATE;
                        current_i2c_result = I2C_SUCCESS;
                        LPM0_EXIT;
                    }
                    break;

                default:
                    break;
            }
            break;

        default:
            __no_operation();
            break;
    }
}
