/*
 * Motor_driver2.c
 *
 * Created: 10/9/2013 1:29:00 PM
 *  Author: Cornelius Fudge
 */ 


# define F_CPU 1000000UL //1MHz clock speed, used for _delay_ms()

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>


#define PWM3A_on()  PORTG |= _BV(2)
#define PWM3A_off()  PORTG &= ~_BV(2)
#define PWM3B_on()  PORTA |= _BV(7)
#define PWM3B_off()  PORTA &= ~_BV(7)
#define DIS3_on()  PORTA |= _BV(6)     //Disable Motor Driver Chip 2
#define DIS3_off()  PORTA &= ~_BV(6)   //Enable Motor Driver Chip 2


#define PWM2A_on()  PORTA |= _BV(5)
#define PWM2A_off()  PORTA &= ~_BV(5)
#define PWM2B_on()  PORTA |= _BV(4)
#define PWM2B_off()  PORTA &= ~_BV(4)
#define DIS2_on()  PORTA |= _BV(3)     //Disable Motor Driver Chip 3
#define DIS2_off()  PORTA &= ~_BV(3)   //Enable Motor Driver Chip 3

int volatile count = 0;

	ISR(TIMER1_COMPA_vect) //interrupt service routine for timer1 compare A flag
	{
		//PWM2A_on();
		//PWM2B_off();
	}

	ISR(TIMER1_COMPB_vect) //interrupt service routine for timer1 compare B flag
	{
		//PWM2A_off();
		//PWM2B_on();
	}
	
	ISR(TIMER3_COMPA_vect) //interrupt service routine for timer3 compare A flag
	{
		if(count == 0){
			count += 1;
			PWM3A_on();
			PWM3B_off();
		}else if(count == 1){
			count += 1;
			PWM2A_on();
			PWM2B_off();
		}else if(count == 2){
		count += 1;
		PWM3A_off();
		PWM3B_on();
		} else{
			PWM2A_off();
			PWM2B_on();
			count = 0;
		}
	}

	ISR(TIMER3_COMPB_vect) //interrupt service routine for timer1 compare B flag
	{
		//PWM3A_off();
		//PWM3B_on();
	}
	
	
	
int main(void)
{
	//Set direction registers for GPIO (1 -> output, 0 -> input)
	DDRA  = 0b11111111;
	DDRG  = 0b00000100;
	DDRB  = 0b11110000; //set OC1A and OC1B to outputs for use with PWM
	PORTB = 0x00; //initialize PORTB to zero

/************************Set up PWM timers***************************************/
	TIMSK = //Timer/Counter Interrupt Mask Register
		(1 << TOIE1) |  //Overflow Interrupt Enable
		(1 << OCIE1A) | //Output Compare A Match Interrupt Enable
		(1 << OCIE1B);  //Output Compare B Match Interrupt Enable
	
	ETIMSK = (1 << TOIE3) | (1 << OCIE3A) | (1 << OCIE3B);	
	TCCR3A = (0 << WGM31) | (0 << WGM30);// | (1 << COM3A0) | (1 << COM3B0);
	TCCR3B = (0 << WGM33) | (1 << WGM32)| (1 << CS32) |(0 << CS31) | (1 << CS30);
	TCCR1A = (0 << WGM01) | (0 << WGM00);// | (1 << COM1A0) | (1 << COM1B0);
	TCCR1B = (0 << WGM13) | (1 << WGM12) | (1 << CS02) |(0 << CS01) | (1 << CS00);
	//Note OCR1B must be LESS THAN OCR1A
	OCR1A = 2000;  //set PWM frequency  1000*(1/8e6) =  12.5e-5 seconds  (4KHz)               
	OCR1B = 1000;   //set Duty Ratio     1000*(1/8e6) =  12.5e-5 seconds (4KHz)    
	OCR3A = 2000;  //set PWM frequency  100*(1/8e6) =  12.5e-5 seconds               
	OCR3B = 50;   //set Duty Ratio     100*(1/8e6) =  12.5e-5 seconds   
/************************Set up PWM timers***************************************/

/************************Set up ADCs***************************************/   
	//set Reference to internal 2.56 volts
	//Set ADC data register to be left aligned
	//Set ADC MUX to read from ADC Channel 3
	ADMUX = 
		(1 << REFS1) |
		(1 << REFS0) |
		(1 << ADLAR) |
		(0 << MUX4) |
		(0 << MUX3) |
		(0 << MUX2) |
		(1 << MUX1) |
		(1 << MUX0);
		
	ADCSRA = 
		(1 << ADEN) | //enable ADC
		(1 << ADSC) | //Start free-running mode conversions	
/************************Set up ADCs***************************************/    
	sei(); //Enable all interrupts
	

	count = 0;


	DIS3_off();
	PWM3A_off();
	PWM3B_off();
	PWM2A_off();
	PWM2B_off();
	DIS2_off();
	
	DIS2_on();
	DIS3_on();
	_delay_ms(1000); //delay 1 second before turning on gate driver	
	DIS2_off();
	DIS3_off();

	
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




