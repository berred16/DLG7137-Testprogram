/*
Copyright (C) 2015  Bernhard Redemann (ber.redemann@gmail.com)

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

Function:
BLDC motor works as an encoder to control a LED display
Application for this can be a electronic dice

Hardware setup, display is a DLG7137:

Atmega8/88/162/328 (THT package, 28 pin)

					PC6 |1------U-----28| PC5 
					PD0 |2            27| PC4
					PD1 |3            26| PC3
-> channel 1 INT0	PD2 |4            25| PC2
					PD3 |5            24| PC1
					PD4 |6            23| PC0
					VCC |7            22| GND
					GND |8            21| AREF
			D6		PB6 |9            20| AVCC
			    	PB7 |10           19| PB5 D5
-> channel 2    	PD5 |11           18| PB4 D4
					PD6 |12           17| PB3 D3
					PD7 |13           16| PB2 D2
			D0		PB0 |14           15| PB1 D1
						 ---------------
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/delay.h>
#define F_CPU 8000000UL

// DLG7137 defines
#define D0 0x01
#define D1 0x02
#define D2 0x04
#define D3 0x08
#define D4 0x10
#define D5 0x20
#define D6 0x40


#define SEG_PORT PORTB

unsigned char symbol_patterns[96]={		// only D5 "1"
										D5, 			// blank
										D0|D5, 			// !
										D1|D5, 			// "
										D0|D1|D5, 		// #
										D2|D5,			// $
										D0|D2|D5,   	// %
										D1|D2|D5,   	// &
										D0|D1|D2|D5,	// ´
										D3|D5,      	// (
										D0|D3|D5,   	// )
										D1|D3|D5,   	// *
										D0|D1|D3|D5,   	// +
										D2|D3|D5,   	// ,
										D0|D2|D3|D5,	// -
										D1|D2|D3|D5,	// .
										D0|D1|D2|D3|D5,	// /
										// D5+D4 "1"
										D5|D4, 				// 0
										D0|D5|D4, 			// 1
										D1|D5|D4, 			// 2
										D0|D1|D5|D4, 		// 3
										D2|D5|D4, 			// 4
										D0|D2|D5|D4,   		// 5
										D1|D2|D5|D4,   		// 6
										D0|D1|D2|D5|D4,		// 7
										D3|D5|D4,      		// 8
										D0|D3|D5|D4,   		// 9
										D1|D3|D5|D4,   		// :
										D0|D1|D3|D5|D4,   	// ;
										D2|D3|D5|D4,   		// <
										D0|D2|D3|D5|D4,		// =
										D1|D2|D3|D5|D4,		// >
										D0|D1|D2|D3|D5|D4,	// ?
										// D6 "1"
										D6, 			// @
										D0|D6, 			// A
										D1|D6, 			// B
										D0|D1|D6, 		// C
										D2|D6, 			// D
										D0|D2|D6,   	// E
										D1|D2|D6,   	// F
										D0|D1|D2|D6,	// G
										D3|D6,      	// H
										D0|D3|D6,   	// I
										D1|D3|D6,   	// J
										D0|D1|D3|D6,   	// K
										D2|D3|D6,   	// L
										D0|D2|D3|D6,	// M
										D1|D2|D3|D6,	// N
										D0|D1|D2|D3|D6,	// O
										// D6+D4 "1"
										D6|D4, 				// P
										D0|D6|D4, 			// Q
										D1|D6|D4, 			// R
										D0|D1|D6|D4, 		// S
										D2|D6|D4, 			// T
										D0|D2|D6|D4,   		// U
										D1|D2|D6|D4,   		// V
										D0|D1|D2|D6|D4,		// W
										D3|D6|D4,      		// X
										D0|D3|D6|D4,   		// Y
										D1|D3|D6|D4,   		// Z
										D0|D1|D3|D6|D4,   	// [
										D2|D3|D6|D4,   		// \
										D0|D2|D3|D6|D4,		// ]
										D1|D2|D3|D6|D4,		// ^
										D0|D1|D2|D3|D6|D4,	// _
										// D6+D5 "1"
										D6|D5, 				// `
										D0|D6|D5, 			// a
										D1|D6|D5, 			// b
										D0|D1|D6|D5, 		// c
										D2|D6|D5, 			// d
										D0|D2|D6|D5,   		// e
										D1|D2|D6|D5,   		// f
										D0|D1|D2|D6|D5,		// g
										D3|D6|D5,      		// h
										D0|D3|D6|D5,   		// i
										D1|D3|D6|D5,   		// j
										D0|D1|D3|D6|D5,   	// k
										D2|D3|D6|D5,   		// l
										D0|D2|D3|D6|D5,		// m
										D1|D2|D3|D6|D5,		// n
										D0|D1|D2|D3|D6|D5,	// o
										// D6+D5+D4 "1"
										D6|D5|D4, 				// p
										D0|D6|D5|D4, 			// q
										D1|D6|D5|D4, 			// r
										D0|D1|D6|D5|D4, 		// s
										D2|D6|D5|D4, 			// t
										D0|D2|D6|D5|D4,   		// u
										D1|D2|D6|D5|D4,   		// v
										D0|D1|D2|D6|D5|D4,		// w
										D3|D6|D5|D4,      		// x
										D0|D3|D6|D5|D4,   		// y
										D1|D3|D6|D5|D4,   		// z
										D0|D1|D3|D6|D5|D4,   	// {
										D2|D3|D6|D5|D4,   		// |
										D0|D2|D3|D6|D5|D4,		// }
										D1|D2|D3|D6|D5|D4,		// ~
										D0|D1|D2|D3|D6|D5|D4	// pattern
										
									};

int main(void)
{

int i=0;
long n;

DDRB = 0xff; // Output all segments
//PORTD |= 0x04; // Pullup PD2
//GICR |= (1 << INT0); // Enable IINT0
//MCUCR |= (1 << ISC01)|(1 << ISC00); // INTO: rising edge

	
//sei();

	while(1)
	{
	PORTB = symbol_patterns[i];
			
 	i++;
	if (i > 95)
		{
		i = 0;
		}
	
	_delay_ms(3000);
	// Thank you and good night...
	}
return 0;
}
