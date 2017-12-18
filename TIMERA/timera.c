/*
 * timera.c
 *
 *  Created on: 2016年7月28日
 *      Author: PP
 */
#include <msp430.h>
#include "timera.h"

void TimerA0_Init(void) {

	TA0CTL = TACLR + TAIE;			 //开启中断并清零
	TA0CTL = TASSEL_1 + MC_1 +  TACLR;//选择SCLK32.768KHZ作为时钟，选用连续模式，并开启中断
	//中断频率200HZ
	TA0CCTL0 = CCIE;                          // CCR0 interrupt enabled
	TA0CCR0 = 69;
}

