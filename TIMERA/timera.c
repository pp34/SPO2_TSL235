/*
 * timera.c
 *
 *  Created on: 2016��7��28��
 *      Author: PP
 */
#include <msp430.h>
#include "timera.h"

void TimerA0_Init(void) {

	TA0CTL = TACLR + TAIE;			 //�����жϲ�����
	TA0CTL = TASSEL_1 + MC_1 +  TACLR;//ѡ��SCLK32.768KHZ��Ϊʱ�ӣ�ѡ������ģʽ���������ж�
	//�ж�Ƶ��200HZ
	TA0CCTL0 = CCIE;                          // CCR0 interrupt enabled
	TA0CCR0 = 69;
}

