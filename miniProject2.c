/*
 * miniProject2.c
 *
 *  Created on: Sep 16, 2022
 *      Author: George
 */

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

unsigned char sec_0=0,sec_1=0,min_0=0,min_1=0,hr_0=0,hr_1=0;	//variables to hold clock time

ISR(TIMER1_COMPA_vect){ // Interrupt Service Routine for timer1 compare mode
	sec_0++;
	if(sec_0>9){
		sec_1++;
		sec_0=0;
		if(sec_1>5)
		{
			min_0++;
			sec_1=0;
			sec_0=0;
			if(min_0>9)
			{
				min_1++;
				min_0=0;
				sec_1=0;
				sec_0=0;
				if(min_1>5)
				{
					hr_0++;
					min_1=0;
					min_0=0;
					sec_1=0;
					sec_0=0;
					if(hr_0>9)
					{
						hr_1++;
						hr_0=0;
						min_1=0;
						min_0=0;
						sec_1=0;
						sec_0=0;
					}
				}
			}
		}
	}
}

// External INT0 Interrupt Service Routine
ISR(INT0_vect)
{
	sec_0=0,sec_1=0,min_0=0,min_1=0,hr_0=0,hr_1=0;
}

// External INT1 Interrupt Service Routine
ISR(INT1_vect)
{
	// Pause the stop watch by disable the timer
	TCCR1B&=~(1<<CS12);
	TCCR1B&=~(1<<CS10);
}

// External INT2 Interrupt Service Routine
ISR(INT2_vect)
{
	// resume the stop watch by enable the timer through the clock bits.
	TCCR1B|=(1<<CS12)|(1<<CS10);
}

void Timer1_Init(void)     //N=1024
{
	TCNT1=0; //timer initial value
	OCR1A= 1000; //compare value
	TIMSK |= (1<<OCIE1A);  //enable compare interrupt for channel A
	/* Configure timer1 control registers
	 * 1. Non PWM mode FOC1A=1 and FOC1B=1
	 * 2. No need for OC1A & OC1B in this example so COM1A0=0 & COM1A1=0 & COM1B0=0 & COM1B1=0
	 * 3. CTC Mode and compare value in OCR1A WGM10=0 & WGM11=0 & WGM12=1 & WGM13=0
	 */
	TCCR1A = (1<<FOC1A);
	/*
	 * 4. Clock = F_CPU/1024 CS10=1 CS11=0 CS12=1
	 */
	TCCR1B = (1<<WGM12) | (1<<CS12) | (1<<CS10);
}

// External INT1 Enable and Configuration function
void INT0_Init(void){
	DDRD &= ~(1<<PD2);      // Configure INT0/PD2 as input pin
	PORTD |= (1<<PD2);	 // Enable internal pull up resistor at INT0/PD2 pin
	// Trigger INT0 with the falling edge
	MCUCR |= (1<<ISC01);
	MCUCR &= ~(1<<ISC00);
	GICR |= (1<<INT0);     // Enable external interrupt pin INT0
}

// External INT1 Enable and Configuration function
void INT1_Init(void)
{
	DDRD &= ~(1<<PD3);      	// Configure INT1/PD3 as input pin
	// Trigger INT1 with the raising edge
	MCUCR |= (1<<ISC11) | (1<<ISC10);
	GICR |= (1<<INT1);			// Enable external interrupt pin INT1
}

// External INT2 Enable and Configuration function
void INT2_Init(void)
{
	DDRB&=~(1<<PB2);      	// Configure INT2/PB2 as input pin
	PORTB |= (1<<PB2);		// Enable internal pull up resistor at INT2/PB2 pin
	// Trigger INT2 with the falling edge
	MCUCSR&=~(1<<ISC2);
	GICR|=(1<<INT2);		// Enable external interrupt pin INT2
}

int main(void){
	DDRC |= 0x0F;      	//First 4 pins in port C as output pins
	DDRA |= 0x3F;		//First 6 pins in port A as output pins
	// Enable all the 7-Segments and initialize all of them with zero value
	PORTA |= 0x3F;
	PORTC &= 0xF0;

	SREG |= (1<<7);			// Enable global interrupts in MC.

	INT0_Init();
	INT1_Init();
	INT2_Init();
	Timer1_Init();



	while(1){
		PORTA = (PORTA & 0xC0) | 0x01;
		PORTC = (PORTC & 0xF0) | sec_0;
		_delay_ms(2);

		PORTA = (PORTA & 0xC0) | 0x02;
		PORTC = (PORTC & 0xF0) | sec_1;
		_delay_ms(2);

		PORTA = (PORTA & 0xC0) | 0x04;
		PORTC = (PORTC & 0xF0) | min_0;
		_delay_ms(2);

		PORTA = (PORTA & 0xC0) | 0x08;
		PORTC = (PORTC & 0xF0) | min_1;
		_delay_ms(2);

		PORTA = (PORTA & 0xC0) | 0x10;
		PORTC = (PORTC & 0xF0) | hr_0;
		_delay_ms(2);

		PORTA = (PORTA & 0xC0) | 0x20;
		PORTC = (PORTC & 0xF0) | hr_1;
		_delay_ms(2);

	}
	return 0;
}

