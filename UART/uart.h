/*
 * uart.h
 *
 *  Created on: 2016��7��28��
 *      Author: PP
 */

#ifndef UART_UART_H_
#define UART_UART_H_

void Uart0_Init(void);
void Uart0SendByte(unsigned char data);
void Uart0SendString(unsigned char *str) ;

#endif /* UART_UART_H_ */
