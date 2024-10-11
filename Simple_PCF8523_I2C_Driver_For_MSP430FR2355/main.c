#include <msp430.h> 
#include "tests/test_i2c.h"

/**
 * main.c
 */
int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer
    test_i2c();


	return 0;
}
