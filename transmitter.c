#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <util/delay.h>
#include "usart.h"

/* Pinout for DIP28 ATMega328P:

                           -------
     (PCINT14/RESET) PC6 -|1    28|- PC5 (ADC5/SCL/PCINT13)
       (PCINT16/RXD) PD0 -|2    27|- PC4 (ADC4/SDA/PCINT12)
       (PCINT17/TXD) PD1 -|3    26|- PC3 (ADC3/PCINT11)
      (PCINT18/INT0) PD2 -|4    25|- PC2 (ADC2/PCINT10)
 (PCINT19/OC2B/INT1) PD3 -|5    24|- PC1 (ADC1/PCINT9)
    (PCINT20/XCK/T0) PD4 -|6    23|- PC0 (ADC0/PCINT8)
                     VCC -|7    22|- GND
                     GND -|8    21|- AREF
(PCINT6/XTAL1/TOSC1) PB6 -|9    20|- AVCC
(PCINT7/XTAL2/TOSC2) PB7 -|10   19|- PB5 (SCK/PCINT5)
   (PCINT21/OC0B/T1) PD5 -|11   18|- PB4 (MISO/PCINT4)
 (PCINT22/OC0A/AIN0) PD6 -|12   17|- PB3 (MOSI/OC2A/PCINT3)
      (PCINT23/AIN1) PD7 -|13   16|- PB2 (SS/OC1B/PCINT2)
  (PCINT0/CLKO/ICP1) PB0 -|14   15|- PB1 (OC1A/PCINT1)
                           -------
*/

volatile unsigned int reload;
volatile unsigned int osc = 0;

// 'Timer 1 output compare A' Interrupt Service Routine
ISR(TIMER1_COMPA_vect)
{
	if(osc){
		OCR1A = OCR1A + reload;
		PORTB ^= 0b00000011; // Toggle PB0 and PB1
	}
}

void wait_1ms(void)
{
	unsigned int saved_TCNT1;
	
	saved_TCNT1=TCNT1;
	
	while((TCNT1-saved_TCNT1)<(F_CPU/1000L)); // Wait for 1 ms to pass
}

void waitms(int ms)
{
	while(ms--) wait_1ms();
}

#define PIN_PERIOD (PINB & 0b00000010)


void adc_init(void)
{
    ADMUX = (1<<REFS0);
    ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
}

uint16_t adc_read(int channel)
{
    channel &= 0x7;
    ADMUX = (ADMUX & 0xf8)|channel;
     
    ADCSRA |= (1<<ADSC);
     
    while(ADCSRA & (1<<ADSC)); //as long as ADSC pin is 1 just wait.
     
    return (ADCW);
}

void PrintNumber(long int N, int Base, int digits)
{ 
	char HexDigit[]="0123456789ABCDEF";
	int j;
	#define NBITS 32
	char buff[NBITS+1];
	buff[NBITS]=0;

	j=NBITS-1;
	while ( (N>0) | (digits>0) )
	{
		buff[j--]=HexDigit[N%Base];
		N/=Base;
		if(digits!=0) digits--;
	}
	usart_pstr(&buff[j+1]);
}

void ConfigurePins (void)
{
	//DDRB  &= 0b11111101; // Configure PB1 as input
	//PORTB |= 0b00000010; // Activate pull-up in PB1
	
	//DDRD  |= 0b11111100; // PD[7..2] configured as outputs
	//PORTD &= 0b00000011; // PD[7..2] = 0
	
	//DDRB  |= 0b00000001; // PB0 configured as output
	//PORTB &= 0x11111110; // PB0 = 0
	
	DDRD|=0b11111000; // PD3, PD4, PD5, PD6, and PD7 are outputs.
	
	DDRB=0b00000011; // PB1 (pin 15) and PB0 (pin 14) are our outputs
	PORTB |= 0x01; // PB0=NOT(PB1)
	TCCR1B |= _BV(CS10);   // set prescaler to Clock/1
	TIMSK1 |= _BV(OCIE1A); // output compare match interrupt for register A
	
}

#define UPPER_F 16500L
#define LOWER_F 16000L
#define TRACKER_F 15000L

// In order to keep this as nimble as possible, avoid
// using floating point or printf() on any of its forms!
int main (void)
{	
	char printString_x[16] ;
	char printString_y[16] ;
	char printString_z[16] ;
	char printString_t[16] ;
	
	const float PI = 3.141592;
	int x = 0, y = 0, z = 0;
	int theta = 0;
	int radius = 0;
	
	int trackermodeflag = 0;
	
	char buff[32];
	unsigned long newF;
	
	reload=0;
	
	sei(); // enable global interupt
	usart_init(); // configure the usart and baudrate
	adc_init();
	ConfigurePins();
	
	TCCR1B |= _BV(CS10); // Turn on Timer 1

	waitms(500); // Wait for putty to start

	while(1)
	{
		// Read Joystick X
		x = (adc_read(1) * 5000L) / 1023L - 2500L;
		sprintf(printString_x, "X: %5d ", x);
		usart_pstr(printString_x);
		
		// Read Joystick Y
		y = (adc_read(0) * 5000L) / 1023L - 2500L;
		sprintf(printString_y, "Y: %5d ", y);
		usart_pstr(printString_y);
		
		//Read Z
		z = (adc_read(2));
		sprintf(printString_z, "Z: %5d ", z);
		usart_pstr(printString_z);
		
		// Calculate Radius
		radius = sqrt(pow(y,2) + pow(x,2));
		
		// Only Compute if Joystick is not in Center
		if(radius > 500) {
			theta = (int)(atan2(y, x) * (UPPER_F-LOWER_F) / (2*PI));
		
			if(theta < 0) theta += (UPPER_F-LOWER_F);
		
			theta += LOWER_F;
			
			newF = theta;
			
		} else {
			
			newF = 0;
		
			if (!z) {
				_delay_ms(150);
				if(!z) {
					if(trackermodeflag){
						trackermodeflag = 0;
					} else {
						trackermodeflag = 1;
					}
				}
			}
		}
		
		if(trackermodeflag) {
			newF = TRACKER_F;
		}
		
		sprintf(printString_t, "F: %5d     ", newF);
		
		if(newF) {
			osc = 1;
		} else {
			osc = 0;
		}
		
		reload=(F_CPU/(2L*newF))+1;
		
		usart_pstr(printString_t);

		usart_pstr("\r");
		
		_delay_ms(50);
	}
}
