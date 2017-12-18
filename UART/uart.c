/*
 * uart.c
 *
 *  Created on: 2016Äê7ÔÂ28ÈÕ
 *      Author: PP
 */
#include <msp430.h>
#include "uart.h"
//mode:
void Uart0_Init(void) {

	UCA0CTL1 |= UCSWRST;                      // **Put state machine in reset**

	UCA0CTL1 |= UCSSEL_1;                     // CLK = ACLK
	UCA0BR0 = 0x03;                           // 32kHz/9600=3.41 (see User's Guide)
	UCA0BR1 = 0x00;                           //
	UCA0MCTL = UCBRS_3 + UCBRF_0;               // Modulation UCBRSx=3, UCBRFx=0

	UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
	UCA0IE |= UCRXIE;                         // Enable USCI_A0 RX interrupt
	//_eint();
	//__bis_SR_register(LPM3_bits + GIE);       // Enter LPM3, interrupts enabled
	__no_operation();                         // For debugger
}

void Uart0SendByte(unsigned char data) {
	while ((UCA0IFG&UCTXIFG)==0);
	UCA0TXBUF = data;
}

void Uart0SendString(unsigned char *str) {
	while (*str != '\0') {
		Uart0SendByte(*str++);
		delay_ms(2);
	}
}
