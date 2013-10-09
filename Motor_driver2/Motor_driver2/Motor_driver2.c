/*
 * Motor_driver2.c
 *
 * Created: 10/9/2013 1:29:00 PM
 *  Author: Cornelius Fudge
 */ 

# define F_CPU 1000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>


int main(void)
{
	DDRC  = 0b11111111;
    while(1)
    {
        PORTC |= _BV(0);
		PORTC |= _BV(1);
		PORTC |= _BV(2);
		PORTC |= _BV(3);
		PORTC |= _BV(4);
		PORTC |= _BV(5);
		PORTC |= _BV(6);
		PORTC |= _BV(7);
    }
}