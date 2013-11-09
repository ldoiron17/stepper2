/*
 * Motor_driver2.c
 *
 * Created: 10/9/2013 1:29:00 PM
 *  Author: Cornelius Fudge
 */ 


# define F_CPU 8000000UL //8MHz clock speed, used for _delay_ms()

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h> 

//Define constants
#define ON 1
#define OFF 0
#define INIT 0
#define POS_CUR 1
#define NEG_CUR 2
#define DISABLED 1
#define ENABLED 0
#define VERIFIED 1
#define UNVERIFIED 0
#define NOT_UPDATED 0
#define NO 0
#define YES 1
#define UPDATED 1
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1

//LED functions for turning on and off status LED 1
#define LED1_on()  PORTD |= _BV(6)
#define LED1_off()  PORTD &= ~_BV(6)

//LED functions for turning on and off status LED 2
#define LED2_on()  PORTD |= _BV(4)
#define LED2_off()  PORTD &= ~_BV(4)

//Motor Driver Chip functions to control voltage outputs for H-Bridge 1
#define PWM1A_on()  PORTA |= _BV(2)
#define PWM1A_off()  PORTA &= ~_BV(2)
#define PWM1B_on()  PORTA |= _BV(1)
#define PWM1B_off()  PORTA &= ~_BV(1)
#define DIS1_on()  PORTA |= _BV(0)     //Disable Motor Driver Chip 1
#define DIS1_off()  PORTA &= ~_BV(0)   //Enable Motor Driver Chip 1

//Motor Driver Chip functions to control voltage outputs for H-Bridge 2
#define PWM2A_on()  PORTA |= _BV(5)
#define PWM2A_off()  PORTA &= ~_BV(5)
#define PWM2B_on()  PORTA |= _BV(4)
#define PWM2B_off()  PORTA &= ~_BV(4)
#define DIS2_on()  PORTA |= _BV(3)     //Disable Motor Driver Chip 2
#define DIS2_off()  PORTA &= ~_BV(3)   //Enable Motor Driver Chip 2

//Motor Driver Chip functions to control voltage outputs for H-Bridge 3
#define PWM3A_on()  PORTG |= _BV(2)
#define PWM3A_off()  PORTG &= ~_BV(2)
#define PWM3B_on()  PORTA |= _BV(7)
#define PWM3B_off()  PORTA &= ~_BV(7)
#define DIS3_on()  PORTA |= _BV(6)     //Disable Motor Driver Chip 3
#define DIS3_off()  PORTA &= ~_BV(6)   //Enable Motor Driver Chip 3

//Define volatile variables for use in ISRs
int volatile count = 0;
int volatile iter = 0;
int volatile state = 0;
int volatile Controller_state = 1;
int volatile Coordinates_updated = 0;
int volatile Coordinates_verified = 0;
int volatile Receiving_Coords = 0;
int volatile Coordinates_correct = 0;
int volatile parsed_coord=0;
signed int volatile Axis1_x_coord=0;
signed int volatile Axis1_y_coord=0;
signed int volatile Axis2_x_coord=0;
signed int volatile Axis2_y_coord=0;
signed int volatile Axis3_x_coord=0;
signed int volatile Axis3_y_coord=0;
char volatile data;
int volatile PWM1A = 0;
int volatile PWM1B = 0;
int volatile PWM2A = 0;
int volatile PWM2B = 0;
uint16_t volatile ADCval=0;
uint16_t volatile ADCval1=0;
uint16_t volatile ADCval2=0;
uint16_t volatile ADCval3=0;
uint16_t volatile ADCval4=0;
uint16_t volatile ADC_avg=0;
char volatile buffer[10];       //Used to Store ADC value readings from itoa() of ADC register
char volatile dummy_dummy;
char *clr_screen = "^\033[2J\033[0;0H";
char *Init_statement = "^\033[2J\033[0;0HWelcome to Crazy Numerical Control 3000\r\nWhere All Of Your Dreams Will Come True\r\n.......................................\r\n\r\n\r\n";
char *Coord_request_x = "Please Input The Desired X Coordinate:";
char *Coord_request_y = "Please Input The Desired Y Coordinate:";
char *Verify_coords1 = "You input the Coordinates (";
char *Verify_coords2 = ",";
char *Verify_coords3 = "). Is this correct? (Y/N)";
char *Verify_coords4 = "Moving To Desired Coordinates...";
char *Verify_coords5 = "Please Renter Coordinates:";
char volatile *newline = "\r\n";
char volatile *clearline = "\r";

void USART_Init( unsigned int ubrr )
{
	
	
	/* Set baud rate */
	UBRR1H = (unsigned char)(ubrr>>8);
	UBRR1L = (unsigned char)ubrr;
	/* Enable receiver and transmitter */
	UCSR1B = (1<<RXEN1)|(1<<TXEN1);//|(1<<RXCIE1)|(1<<UDRIE1);//|(1<<TXCIE1);
	/* Set frame format: 8data, 2stop bit */
	UCSR1C = (1<<USBS1)|(3<<UCSZ10)|(1<<UPM11); //even parity
	
	//UCSR1A = 0; //asynchronous normal mode
}



void USART_Transmit(unsigned char data_send )
{
	
	UCSR1B |= (1<<TXEN1);
	/* Wait for empty transmit buffer */
	while ( !( UCSR1A & (1<<UDRE1)) )
	;
	/* Put data into buffer, sends the data */
	UDR1 = data_send;
	
	UCSR1B &= ~(1<<TXEN1);
	
	unsigned char dummy;
	while ( UCSR1A & (1<<RXC1) ) {
		dummy = UDR1;
		dummy_dummy = dummy;
	}
	/* Wait for empty transmit buffer */
	//while ( !( UCSR1A & (1<<UDRE1)) )
	//;
	
	//UCSR1A = UCSR1A | (1 << TXC1);

	
}

unsigned char USART_Receive( void )
{
	UCSR1B &= ~(1<<TXEN1); //Disable USART TX pin
	unsigned char dummy, tmp;
	/* Wait for data to be received */
	while ( !(UCSR1A & (1<<RXC1)) );
	/* Get and return received data from buffer */
	tmp = UDR1;
	//Flush out the UDR1 buffer
	while ( UCSR1A & (1<<RXC1) ) {
		dummy = UDR1;
		dummy_dummy = dummy;
	}
	
	return tmp;
}	

void USART_Flush( void )
{
	unsigned char dummy;
	while ( UCSR1A & (1<<RXC1) ) {
		dummy = UDR1;
		dummy_dummy = dummy;
	}
	UCSR1A = 0;
}



void USART_putstring(char* StringPtr){
	
	//UCSR1B &= ~(1 << RXCIE1);  //Disable receive char interrupt
	
	while(*StringPtr != 0x00){    //Here we check if there is still more chars to send, this is done checking the actual char and see if it is different from the null char
		USART_Transmit(*StringPtr);    //Using the simple send function we send one char at a time
	StringPtr++;}        //We increment the pointer so we can read the next char
	
	//UCSR1A &= ~(1 << TXC1); 
	//UCSR1B |= (1 << TXCIE1);  //Enable send transmission complete interrupt
	
}

void Init_PWM(void){
	
	/************************Set up PWM timers***************************************/
	TIMSK = //Timer/Counter Interrupt Mask Register
	(1 << TOIE1) |  //Overflow Interrupt Enable
	(1 << OCIE1A) | //Output Compare A Match Interrupt Enable
	(1 << OCIE1B);  //Output Compare B Match Interrupt Enable
	
	ETIMSK = (1 << TOIE3) | (1 << OCIE3A) | (1 << OCIE3B);
	TCCR3A = (0 << WGM31) | (0 << WGM30);// | (1 << COM3A0) | (1 << COM3B0);
	TCCR3B = (0 << WGM33) | (1 << WGM32)| (1 << CS32) |(0 << CS31) | (1 << CS30);
	TCCR1A = (0 << WGM01) | (0 << WGM00);// | (1 << COM1A0) | (1 << COM1B0);
	TCCR1B = (0 << WGM13) | (1 << WGM12) | (0 << CS02) |(1 << CS01) | (1 << CS00); //clk_IO = clk/64
	//Note OCR1B must be LESS THAN OCR1A
	OCR1A = 100;  //set PWM frequency  1000*(1/8e6) =  12.5e-5 seconds  (4KHz)
	OCR1B = 2000;   //set Duty Ratio     1000*(1/8e6) =  12.5e-5 seconds (4KHz)
	OCR3A = 2000;  //set PWM frequency  100*(1/8e6) =  12.5e-5 seconds
	OCR3B = 50;   //set Duty Ratio     100*(1/8e6) =  12.5e-5 seconds
	/************************Set up PWM timers***************************************/
}

void Init_ADC( void ){
	/************************Set up ADCs***************************************/
	//set Reference to internal 2.56 volts
	//Set ADC data register to be right aligned
	//Set ADC MUX to read from ADC Channel 3
	ADMUX =
	(1 << REFS1) |
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
	(0 << ADPS2) | //Setting these three bits to zero sets the ADC clk to the processor_clk/2
	(0 << ADPS1) |
	(0 << ADPS0);
	
	ADCSRB =
	(0 << ADTS2) | //setting these three bits to zero enables free running mode of the ADC
	(0 << ADTS1) |
	(0 << ADTS0);
	

	
	//ADC ref voltage is 2.56 volts at 10 bit resolution
	// 1 bit on the ADC is 2.5mV input at the pin
	//0.1Ohm sense resistor -> 4[A]*0.1[Ohm] = 400[mV]
	//400 mV/2.5 -> I_max_ADC = 160
	/************************Set up ADCs***************************************/
	
}

void Motor_Disable (void){
	
	    //Set Global Motors Disable Flag
		Controller_state = DISABLED;
	
		DIS1_on();
		PWM1A_off();
		PWM1B_off();
		DIS2_on();
		PWM2A_off();
		PWM2B_off();
		DIS3_on();
		PWM3A_off();
		PWM3B_off();
		
		_delay_ms(1000); //delay 1 second before allowing anything else to happen
	
}

void Motor_Enable (void){
	
	//Set Global Motors Disable Flag
	Controller_state = DISABLED;
	
	DIS1_off();
	DIS2_off();
	//DIS3_off();

	

	
}
void Motor_Initialization( void ){
	
	
	//Set direction registers for GPIO (1 -> output, 0 -> input)
	DDRA  = 0b11111111;
	DDRD  = 0b01010000;
	DDRG  = 0b00000100;
	//DDRD  = 0b10000000;
	DDRB  = 0b11110100; //set OC1A and OC1B to outputs for use with PWM
	PORTB = 0x00; //initialize PORTB to zero
	
	//Startup in a Disabled State
	//Motor_Disable();
	

	
}


void Receive_Coords( void ){
	
	unsigned char tmp;
	//Ask for input Coordinate X
	Receiving_Coords = YES;
	Coordinates_updated = NOT_UPDATED;
	USART_putstring(Coord_request_x); 
	USART_putstring(newline);
	while( Coordinates_updated == NOT_UPDATED ){
		
		
		tmp = USART_Receive();
		//USART_Transmit(tmp);
		
		//LED1_off();
		if(Receiving_Coords == YES){
				

		if(Coordinates_updated == NOT_UPDATED){
			parsed_coord = atoi(&tmp);
			
			Coordinates_updated = UPDATED;
		}
				
		/*if(Coordinates_verified == UNVERIFIED){
			if( tmp == 'Y' | tmp == 'y'){
				Coordinates_correct = YES;
				} else{
				Coordinates_correct = NO;
				}	
			}
			*/	
				
		}
	}
	Axis1_x_coord = parsed_coord;
	
	//Ask for input Coordinate Y
	USART_putstring(Coord_request_y); 
	USART_putstring(newline);
	Coordinates_updated = NOT_UPDATED;
	while( Coordinates_updated == NOT_UPDATED ){
		
		tmp = USART_Receive();
		//USART_Transmit(tmp);
		if(Receiving_Coords == YES){
			

			if(Coordinates_updated == NOT_UPDATED){
				parsed_coord = atoi(&tmp);
				Coordinates_updated = UPDATED;
			}
			
			/*if(Coordinates_verified == UNVERIFIED){
				if( tmp == 'Y' | tmp == 'y'){
					Coordinates_correct = YES;
					} else{
					Coordinates_correct = NO;
				}
			}
			*/
			
		}
	}
	Axis1_y_coord = parsed_coord;
	
	//Verify that the input coordinates are correct
	Coordinates_verified = UNVERIFIED;
	Coordinates_correct = NO;
	USART_putstring(Verify_coords1);
	itoa(Axis1_x_coord, buffer, 10);
	USART_putstring(buffer);
	USART_putstring(Verify_coords2);
	itoa(Axis1_y_coord, buffer, 10);
	USART_putstring(buffer);
	USART_putstring(Verify_coords3);
	USART_putstring(newline);
	while( Coordinates_verified == UNVERIFIED ){
		
		tmp = USART_Receive();
		//USART_Transmit(tmp);
		//LED1_off();
		if(tmp == 'y'){
			Coordinates_correct == YES;
			Coordinates_verified = VERIFIED;
		}else{
			Receive_Coords();
		}
		//USART_putstring(clearline);

	}
	
	Receiving_Coords = NO;
	
}


	
int main(void)
{
	
	cli(); 
	//char data_test = 'a';
	//unsigned int ubrr;	//Used to Initial Baud Rate
	//ubrr = MYUBRR;
	//int state=0;
	count = 0;
	Init_PWM();
	//Init_ADC();
	
	Motor_Initialization();	
	//LED1_on();
	//LED2_on();
	USART_Init(416);
	sei(); //Enable all interrupts
	
	//PORTB &= ~(_BV(2));
	USART_putstring(clr_screen);
	//_delay_ms(500);
	USART_putstring(Init_statement); //Print Startup Message over USB
	//USART_putstring("Moving To Desired Coordinates...\r\n");
	//USART_putstring("Moving To Desired Coordinates...\r\n");
	LED1_off();
	LED2_off();
	Motor_Enable();
	
		//PORTB &= ~(_BV(2));
		//PORTB |= (_BV(2));
	while(1){
		
		LED1_on();
		Receive_Coords();
		USART_putstring("Moving To Desired Coordinates...\r\n");
		

		/*if( (ADCSRA & (1<<ADIF)) == 1){
			LED2_on();
			}else{
			LED2_off();

		*/
	}
}


	/*
ISR(USART1_RX_vect)
{

	data = UDR1;
	UCSR1B &= ~(1 << RXCIE1);  //Disable receive char interrupt
	//USCR1B |= (1<<UDRIE1);
	//USART_Transmit(data);    //Using the simple send function we send one char at a time
		
		
	UCSR1A &= ~(1 << TXC1);
	UCSR1B |= (1 << TXCIE1);  //Enable send transmission complete interrupt


	LED1_off();
	_delay_ms(1000);
	
	
	
	//LED1_on();
	if(Receiving_Coords == YES){
		
		unsigned char tmp;//, stat;
		tmp= UDR1; // get the data
		if(Coordinates_updated == NOT_UPDATED){
			parsed_coord = atoi(tmp);
			Coordinates_updated = UPDATED;
		}
	
		if(Coordinates_verified == UNVERIFIED){
			if( tmp == 'Y' | tmp == 'y'){
				Coordinates_correct = YES;
			} else{
				Coordinates_correct = NO;
			}
		}
	
	
	}
}


ISR(USART1_TX_vect)
{
	
	UDR1 = data;
	UCSR1B &= ~(1 << TXCIE1);  //Disable send transmission complete interrupt
	UCSR1A &= ~(1<<UDRE1);
	UCSR1B |= (1 << RXCIE1);  //Enable receive char interrupt
	//LED1_on();
	//UCSR1A = (0 << TXC1);
}
*/

ISR(TIMER1_COMPA_vect) //interrupt service routine for timer1 compare A flag
	{
		
		
		/* Full step code
		if(count == 0){
			count += 1;
			PWM1A = ON;
			PWM1B = OFF;
			
			PWM1A_on();
			PWM1B_off();
			}else if(count == 1){
			count += 1;
			PWM2A = ON;
			PWM2B = OFF;
			PWM2A_on();
			PWM2B_off();
			}else if(count == 2){
			count += 1;
			
			PWM1A = OFF;
			PWM1B = ON;
			PWM1A_off();
			PWM1B_on();
			} else{
			PWM2A = OFF;
			PWM2B = ON;
			PWM2A_off();
			PWM2B_on();
			count = 0;
		} */
		
	// Half step code
		if(count == 0){
			count += 1;
			PWM1A = ON;
			PWM1B = OFF;
			PWM1A_on();
			PWM1B_off();
			PWM2A = ON;
			PWM2B = OFF;
			PWM2A_on();
			PWM2B_off();
		}else if(count == 1){
			count += 1;
			PWM1A = OFF;
			PWM1B = OFF;
			PWM1A_off();
			PWM1B_off();
			PWM2A = ON;
			PWM2B = OFF;
			PWM2A_on();
			PWM2B_off();
		}else if(count == 2){
			count += 1;
			PWM1A = OFF;
			PWM1B = ON;
			PWM1A_off();
			PWM1B_on();
			PWM2A = ON;
			PWM2B = OFF;
			PWM2A_on();
			PWM2B_off();
		}else if(count == 3){
			count += 1;
			PWM1A = OFF;
			PWM1B = ON;
			PWM1A_off();
			PWM1B_on();
			PWM2A = OFF;
			PWM2B = OFF;
			PWM2A_off();
			PWM2B_off();
		}else if(count == 4){
			count += 1;
			PWM1A = OFF;
			PWM1B = ON;
			PWM1A_off();
			PWM1B_on();
			PWM2A = OFF;
			PWM2B = ON;
			PWM2A_off();
			PWM2B_on();
		}else if(count == 5){
			count += 1;
			PWM1A = OFF;
			PWM1B = OFF;
			PWM1A_off();
			PWM1B_off();
			PWM2A = OFF;
			PWM2B = ON;
			PWM2A_off();
			PWM2B_on();
		}else if(count == 6){
			count += 1;
			PWM1A = ON;
			PWM1B = OFF;
			PWM1A_on();
			PWM1B_off();
			PWM2A = OFF;
			PWM2B = ON;
			PWM2A_off();
			PWM2B_on();
		} else{
			count = 0;
			PWM1A = ON;
			PWM1B = OFF;
			PWM1A_on();
			PWM1B_off();
			PWM2A = OFF;
			PWM2B = OFF;
			PWM2A_off();
			PWM2B_off();
	}
	
	/*	if(iter == 100){
			iter = 0;
			if(OCR1A > 50){
				OCR1A -= 1;
			}
		}else{
			iter += 1;
		}
*/
	}

	ISR(TIMER1_COMPB_vect) //interrupt service routine for timer1 compare B flag
	{

	}
	
	ISR(TIMER3_COMPA_vect) //interrupt service routine for timer3 compare A flag
	{
		/*if(count == 0){
			count += 1;
			PWM1A = ON;
			PWM1B = OFF;
			
			PWM3A_on();
			PWM3B_off();
		}else if(count == 1){
			count += 1;
			PWM2A = ON;
			PWM2B = OFF;
			PWM2A_on();
			PWM2B_off();
		}else if(count == 2){
		count += 1;
		
			PWM1A = OFF;
			PWM1B = ON;
			PWM3A_off();
			PWM3B_on();
		} else{
			PWM2A = OFF;
			PWM2B = ON;
			PWM2A_off();
			PWM2B_on();
			count = 0;
		}*/
	}

	ISR(TIMER3_COMPB_vect) //interrupt service routine for timer1 compare B flag
	{
		//PWM3A_off();
		//PWM3B_on();
	}
	
	/*
	ISR(ADC_vect) 
	{
		
		//LED1_on();
		ADCval4 = ADCval3;
		ADCval3 = ADCval2;
		ADCval2 = ADCval1;
		ADCval1 = ADCval;
		ADCval = ADC;
		
		ADC_avg = (3*ADCval)/4 + ADCval1/4;
		
		
		if( (ADC_avg <= 120) ){  //approximately 1A
			state = POS_CUR;
			if( PWM2A == ON){
				//PWM2A_on();
				//PWM2B_off();
				DIS2_off();
				}else if ( PWM2B == ON){
				//PWM2A_off();
				//PWM2B_on();
				DIS2_off();
			}
			//}else{
			//	PWM2A_off();
			//	PWM2B_off();
			//}
			
			//DIS2_on();
			}else if ( (ADC_avg > 120) & (ADC_avg<155) ){
			if(state == POS_CUR){
				if( PWM2A == ON){
					//PWM2A_on();
					//PWM2B_off();
					DIS2_off();
					}else if ( PWM2B == ON){
					//PWM2A_off();
					//PWM2B_on();
					DIS2_off();
				}
				}else if (state == NEG_CUR){
				//PWM2A_off();
				//PWM2B_off();
				DIS2_on();
			}
		}
		else if (ADC_avg >=150){
			state = NEG_CUR;
			DIS2_on();
			//PWM2A_off();
			//PWM2B_off();
			//DIS2_off();
			//LED1_off();
			}
		
		//LED1_off();
		/*
		int ADCval;
		char buffer[10];       // somewhere to store the string
		
		ADCval = ADC;
		

		//PWM2A = ON;
		//PWM2B = OFF;
		//PWM2A_off();
		//PWM2B_on();

		if( (ADCval <= 120) ){  //approximately 1A
			LED1_on();
			if( PWM2A == ON){
				PWM2A_on();
				PWM2B_off();
				}else if ( PWM2B == ON){
				PWM2A_off();
			PWM2B_on();}
			//}else{
			//	PWM2A_off();
			//	PWM2B_off();
			//}
	
			//DIS2_on();
	
			}else if ( ADCval >=122){
			LED1_off();
			PWM2A_off();
			PWM2B_off();
			//DIS2_off();
			//LED1_off();
		}		
		
		//itoa(ADC_avg, buffer, 10);
		//USART_putstring(newline);
		//USART_putstring(buffer);

		//if(PWM2A == ON){
		//	LED2_on();
		//	}else{
		//	LED2_off();
		//}
















//Multiplex through ADC readings
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
	

			_delay_ms(200);
			LED1_on();	
			_delay_ms(200);
			LED1_off();
			
			
	}
	*/
