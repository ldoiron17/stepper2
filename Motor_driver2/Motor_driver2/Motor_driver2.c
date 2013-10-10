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


#define PWM1A_on()  PORTA |= _BV(2)
#define PWM1A_off()  PORTA &= ~_BV(2)
#define PWM1B_on()  PORTA |= _BV(1)
#define PWM1B_off()  PORTA &= ~_BV(1)
#define DIS1_on()  PORTA |= _BV(0)
#define DIS1_off()  PORTA &= ~_BV(0)

/*
ISR (TIMER1_OVF_vect)
{
	PWM1A_on();
};

ISR (TIMER1_COMPB_vect)
{
	PWM1A_off();
};
*/
	ISR(TIMER1_COMPA_vect) //interrupt service routine for timer1 compare A flag
	{
		PWM1A_on();
		PWM1B_off();
	}

	ISR(TIMER1_COMPB_vect) //interrupt service routine for timer1 compare B flag
	{
		PWM1A_off();
		PWM1B_on();
	}

int main(void)
{
	//Set direction registers (1 -> output, 0 -> input)
	DDRA  = 0b00000111;
	DDRB  = 0b11110000; //set OC1A and OC1B to outputs for use with PWM
	PORTB = 0x00;

	TIMSK = (1 << TOIE1) | (1 << OCIE1A) | (1 << OCIE1B);
	
	TCCR1A = (0 << WGM01) | (1 << WGM00);
	TCCR1B = (0 << WGM13) | (1 << WGM12) | (1 << CS02) |(0 << CS01) | (0 << CS00);
	OCR1A = 1500;                       // 0.001024*1954 ~= 2 therefore SIG_OUTPUT_COMPARE1A will be triggered every 2 seconds
	OCR1B = 977;                       // 0.001024*977 = 1.0004480 therefore SIG_OUTPUT_COMPARE1B will be triggered every second
	sei();
	
	//Set up PWM timers
	
	//TIMSK = _BV(OCIE1A) | _BV(OCIE1B); // Enable Interrupt Timer/Counter1, Output Compare A & B (SIG_OUTPUT_COMPARE1A/SIG_OUTPUT_COMPARE1B)
	//TCCR1A = _BV(COM1A1) | _BV(COM1B1);
	//TCCR1B = _BV(CS12) | _BV(CS10) | _BV(WGM12);    // Clock/1024, 0.001024 seconds per tick, Mode=CTC

	
/*	
	TCCR2 |= _BV(WGM21); //set to CTC mode
	TCCR2 |= _BV(CS22);
	TCCR2 |= _BV(CS20);
	TIMSK |= _BV(OCIE2); //turn output of timer on
*/

	DIS1_on();
	PWM1A_on();
	PWM1B_on();
	DIS1_off();
	PWM1A_off();
	PWM1B_off();
	
	while(1){
		
	}

}
/*
	ISR(TIMER1_COMPA_vect) //interrupt service routine for timer1 compare A flag
	{
		PWM1A_on();
		PWM1B_off();
	}

	ISR(TIMER1_COMPB_vect) //interrupt service routine for timer1 compare B flag
	{
		PWM1A_off();
		PWM1B_on();
	}
*/
//////////////




