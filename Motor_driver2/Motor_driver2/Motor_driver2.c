/*
 * Motor_driver2.c
 *
 * Created: 10/9/2013 1:29:00 PM
 *  Author: Cornelius Fudge
 */ 


# define F_CPU 8000000UL //1MHz clock speed, used for _delay_ms()

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define LED1_on()  PORTD |= _BV(7)
#define LED1_off()  PORTD &= ~_BV(7)

#define PWM3A_on()  PORTG |= _BV(2)
#define PWM3A_off()  PORTG &= ~_BV(2)
#define PWM3B_on()  PORTA |= _BV(7)
#define PWM3B_off()  PORTA &= ~_BV(7)
#define DIS3_on()  PORTA |= _BV(6)     //Disable Motor Driver Chip 3
#define DIS3_off()  PORTA &= ~_BV(6)   //Enable Motor Driver Chip 3


#define PWM2A_on()  PORTA |= _BV(5)
#define PWM2A_off()  PORTA &= ~_BV(5)
#define PWM2B_on()  PORTA |= _BV(4)
#define PWM2B_off()  PORTA &= ~_BV(4)
#define DIS2_on()  PORTA |= _BV(3)     //Disable Motor Driver Chip 2
#define DIS2_off()  PORTA &= ~_BV(3)   //Enable Motor Driver Chip 2

int volatile count = 0;
int volatile state = 0;

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
	
	ISR(ADC_vect) 
	{
		
/*		//Multiplex through ADC readings
		switch(state){
			case 1:  //ADC 3
			ADMUX =
			(0 << MUX4) |
			(0 << MUX3) |
			(0 << MUX2) |
			(1 << MUX1) |
			(1 << MUX0);
			//state = 2;
			break;
			case 2:  //ADC 4
			ADMUX =
			(0 << MUX4) |
			(0 << MUX3) |
			(1 << MUX2) |
			(0 << MUX1) |
			(0 << MUX0);
			//state = 1;
			break;
			default:
			ADMUX =
			(0 << MUX4) |
			(0 << MUX3) |
			(0 << MUX2) |
			(1 << MUX1) |
			(1 << MUX0);
		}
		
		if(state == 1){
			if(ADC >= 130){  //approximately 1A
				DIS3_on();
				}else if ( ADC <= 90){
				DIS3_off();
			}
			state = 2;
			}else if(state == 2){
			if(ADC >= 130){  //approximately 1A
				DIS2_on();
				}else if ( ADC <= 90){
				DIS2_off();
			}
			state = 1;
		}

		if(ADC >= 130){  //approximately 1A
			DIS2_on();
			}else if ( ADC <= 90){
			DIS2_off();
		}
		if(ADC < 10){
			LED1_on();
			}else{
			LED1_off();
		}
		
		//Start new ADC conversion
		//ADCSRA = (1 << ADSC);
	*/

			LED1_on();	
	}
	
	
int main(void)
{
	//Set direction registers for GPIO (1 -> output, 0 -> input)
	DDRA  = 0b11111111;
	DDRD  = 0b10000000;
	DDRG  = 0b00000100;
	//DDRD  = 0b10000000;
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
	OCR1A = 10;  //set PWM frequency  1000*(1/8e6) =  12.5e-5 seconds  (4KHz)               
	OCR1B = 10;   //set Duty Ratio     1000*(1/8e6) =  12.5e-5 seconds (4KHz)    
	OCR3A = 2000;  //set PWM frequency  100*(1/8e6) =  12.5e-5 seconds               
	OCR3B = 50;   //set Duty Ratio     100*(1/8e6) =  12.5e-5 seconds   
/************************Set up PWM timers***************************************/

/************************Set up ADCs***************************************/   
	//set Reference to internal 2.56 volts
	//Set ADC data register to be right aligned
	//Set ADC MUX to read from ADC Channel 3
	ADMUX = 
		(0 << REFS1) |
		(1 << REFS0) |
		(0 << ADLAR) |
		(0 << MUX4) |
		(0 << MUX3) |
		(0 << MUX2) |
		(1 << MUX1) |
		(1 << MUX0);
		
	ADCSRA = 
		(1 << ADEN) | //enable ADC
		(1 << ADSC) | //Start free-running mode conversions	
		(1 << ADATE) | //Enable auto-trigger
		(1 << ADIE) | //Enable Interrupt Flag
		(1 << ADPS2) | //Setting these three bits to zero sets the ADC clk to the processor_clk/2
		(1 << ADPS1) |
		(1 << ADPS0);
		
	ADCSRB =
		(0 << ADTS2) | //setting these three bits to zero enables free running mode of the ADC
		(0 << ADTS1) |
		(0 << ADTS0);
		

	
	//ADC ref voltage is 2.56 volts at 10 bit resolution
	// 1 bit on the ADC is 2.5mV input at the pin
	//0.1Ohm sense resistor -> 4[A]*0.1[Ohm] = 400[mV]
	//400 mV/2.5 -> I_max_ADC = 160
/************************Set up ADCs***************************************/    
	
	
	state = 1;
	count = 0;
	
	

	DIS3_off();
	PWM3A_off();
	PWM3B_off();
	PWM2A_off();
	PWM2B_off();
	DIS2_off();
	
	DIS2_on();
	DIS3_on();
	LED1_on();
	_delay_ms(1000); //delay 1 second before turning on gate driver	
	//LED1_off();
	DIS2_off();
	DIS3_off();

	sei(); //Enable all interrupts
	
	while(1){
		
		
		LED1_off();
		
		
	}

}





