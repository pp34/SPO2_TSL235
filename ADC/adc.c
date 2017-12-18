/*
 * adc.c
 *
 *  Created on: 2016Äê7ÔÂ28ÈÕ
 *      Author: PP
 */

//               MSP430F552x
//             -----------------
//         /|\|                 |
//          | |                 |
//          --|RST              |
//            |                 |
//    Vin0 -->|P6.0/CB0/A0      |
//    Vin1 -->|P6.1/CB1/A1      |
//    Vin2 -->|P6.2/CB2/A2      |
//    Vin3 -->|P6.3/CB3/A3      |
//            |                 |
#include <msp430.h>
#include "adc.h"

volatile unsigned int results[4];           // Needs to be global in this example
														  // Otherwise, the compiler removes it
														  // because it is not used for anything.

void Adc_Init(void){

	P6SEL = 0x0F;                             // Enable A/D channel inputs
	ADC12CTL0 = ADC12ON + ADC12MSC + ADC12SHT0_2; // Turn on ADC12, set sampling time
	ADC12CTL1 = ADC12SHP + ADC12CONSEQ_1;       // Use sampling timer, single sequence
	ADC12MCTL0 = ADC12INCH_0;                 // ref+=AVcc, channel = A0
	ADC12MCTL1 = ADC12INCH_1;                 // ref+=AVcc, channel = A1
	ADC12MCTL2 = ADC12INCH_2;                 // ref+=AVcc, channel = A2
	ADC12MCTL3 = ADC12INCH_3 + ADC12EOS;        // ref+=AVcc, channel = A3, end seq.
	ADC12IE = 0x08;                           // Enable ADC12IFG.3

	ADC12CTL0 |= ADC12ENC;                    // Enable conversions
///////////////////////////////////////
	ADC12CTL0 |= ADC12SC;                   // Start convn - software trigger
///////////////////////////////////////
		//__bis_SR_register(LPM4_bits + GIE);     // Enter LPM4, Enable interrupts
	__no_operation();                       // For debugger

}
