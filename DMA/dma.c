/*
 * dma.c
 *
 *  Created on: 2016Äê7ÔÂ28ÈÕ
 *      Author: PP
 */
#include <msp430.h>
#include "dma.h"
#include "uart.h"

void DMA0_Init(void) {

	Uart0_Init();

	DMACTL0 = DMA0TSEL_17;        // DMA0 - UCA0TXIFG

}

void DMA0toUCA0(unsigned char string) {

	__data16_write_addr((unsigned short)&DMA0SA, (unsigned long)&string);
	// Source block address
	__data16_write_addr((unsigned short)&DMA0DA, (unsigned long)&UCA0TXBUF);
	// Destination single address
	DMA0SZ = sizeof(string)-1;                               // Block size
	DMA0CTL = DMASRCINCR_3 + DMASBDB + DMALEVEL;  // inc src
												  //DMA0CTL = DMADT_5 + DMASRCINCR_3 + DMADSTINCR_3; // Rpt, inc
	DMA0CTL |= DMAEN;                         // Enable DMA0
}



