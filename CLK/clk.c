/*
 * clk.c
 *
 *  Created on: 2016��7��28��
 *      Author: PP
 */
#include <msp430.h>
//#include "HAL_PMM.h"
#include "clk.h"

/*(1)XT1����LFģʽ��ΪXT1CLKʱ��Դ��ACLKѡͨΪXT1CLK��
**(2)MCLKѡͨΪDCOCLKDIV
**(3)SMCLKѡͨΪDCOCLKDIV
**(4)FLLʹ�ܣ��ҽ�XT1CLK��ΪFLL�ο�ʱ�ӡ�
**(5)XIN��XOUT������Ϊͨ��IO��XIN��XOUT����ΪXT1����ǰ��XT1���ֽ��á�
**(6)������õĻ���XT2IN��XT2OUT������Ϊͨ��IO�ұ��ֽ�ֹ״̬��
*/
/*REFOCLK��VLOCLK��DCOCLK��������ʱ��ô��Ϊ��Ĭ��״̬���ǿ��õģ�
 * ���ԣ��л���ʱ��ֻ��Ҫͨ��UCSCTL4������
 * ACLK��SMCLK��MCLK��ʱ��Դ���ɣ���XT1CLK��XT2CLK��Ҫ����Ӳ���ľ����������ȷ��
 * */
//CPUCLK->MCLK&SCLK 32.768KHZ
//ACLK			 default 	 32.768KHZ
void CLK_Init(void){

	Set_ACLKREF();
	Set_SMCLK8MHZ();
}

//DCOģ��������Ҫ�ο�ʱ��REFCLK��REFCLK��������REFOCLK��XT1CLK��XT2CLK��
//ͨ��UCSCTL3��SELREFѡ��Ĭ��ʹ�õ�XT1CLK�������XT1CLK��������ʹ��REFOCLK��
/*	DCO��Ƶ��ʽ
 * DCOCLK = D*(N+1)*(REFCLK/n)		default : 2097152	 * DCOCLKDIV = (N+1)*(REFCLK/n)		default : 1048576
 * D����ͨ��UCSCTL2�е�FLLD���趨��Ĭ��Ϊ1��Ҳ����2��Ƶ
 * N����ͨ��UCSCTL2�е�FLLN���趨��Ĭ��ֵΪ32
 */
//DCORSEL---��8Ƶ�ʶ�
//DCOX-------�ٷ�32Ƶ�ʽ�
/*******************************************************
 * DCORSEL = 0�ĵ��ڷ�ΧԼΪ0.20 ~ 0.70MHZ��
 * DCORSEL= 1�ĵ��ڷ�ΧԼΪ0.36  ~ 1.47MHZ��
 * DCORSEL = 2�ĵ��ڷ�ΧԼΪ0.75 ~ 3.17MHZ��
 * DCORSEL = 3�ĵ��ڷ�ΧԼΪ1.51 ~ 6.07MHZ��
 * DCORSEL = 4�ĵ��ڷ�ΧԼΪ3.2   ~ 12.3MHZ��
 * DCORSEL = 5�ĵ��ڷ�ΧԼΪ6.0   ~ 23.7MHZ��
 * DCORSEL = 6�ĵ��ڷ�ΧԼΪ10.7 ~ 39.7MHZ��
 * DCORSEL = 7�ĵ��ڷ�ΧԼΪ19.6 ~ 60   MHZ��
 *******************************************************/
void Set_SMCLK25MHZ(void){
	//   ACLK = REFO = 32kHz, MCLK = SMCLK = 25MHz
	 // Increase Vcore setting to level3 to support fsystem=25MHz
	  // NOTE: Change core voltage one level at a time..
	  SetVCore(3);

	  UCSCTL3 = SELREF_2;                       // Set DCO FLL reference = REFO
	  UCSCTL4 |= SELA_2;                        // Set ACLK = REFO

	  __bis_SR_register(SCG0);                  // Disable the FLL control loop
	  UCSCTL0 = 0x0000;                         // Set lowest possible DCOx, MODx
	  UCSCTL1 = DCORSEL_7;                      // Select DCO range 50MHz operation
	  UCSCTL2 = FLLD_0 + 762;                   // Set DCO Multiplier for 25MHz
	                                            // (N + 1) * FLLRef = Fdco
	                                            // (762 + 1) * 32768 = 25MHz
	                                            // Set FLL Div = fDCOCLK/2
	  __bic_SR_register(SCG0);                  // Enable the FLL control loop

	  // Worst-case settling time for the DCO when the DCO range bits have been
	  // changed is n x 32 x 32 x f_MCLK / f_FLL_reference. See UCS chapter in 5xx
	  // UG for optimization.
	  // 32 x 32 x 25 MHz / 32,768 Hz ~ 780k MCLK cycles for DCO to settle
	  __delay_cycles(782000);

	  // Loop until XT1,XT2 & DCO stabilizes - In this case only DCO has to stabilize
	  do
	  {
	    UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG);
	                                            // Clear XT2,XT1,DCO fault flags
	    SFRIFG1 &= ~OFIFG;                      // Clear fault flags
	  }while (SFRIFG1&OFIFG);                   // Test oscillator fault flag

}

void Set_SMCLK2_45MHZ(void){
	//  ACLK = REFO = 32768Hz, MCLK = SMCLK = DCO = (74+1) x REFO = 2457600Hz
	 P5SEL |= BIT4+BIT5;                       // Port select XT1
	  UCSCTL6 &= ~(XT1OFF);                     // XT1 On
	  UCSCTL6 |= XCAP_3;                        // Internal load cap

	  // Loop until XT1 fault flag is cleared
	  do
	  {
	    UCSCTL7 &= ~XT1LFOFFG;                  // Clear XT1 fault flags
	  }while (UCSCTL7&XT1LFOFFG);               // Test XT1 fault flag

	  // Initialize DCO to 2.45MHz
	  __bis_SR_register(SCG0);                  // Disable the FLL control loop
	  UCSCTL0 = 0x0000;                         // Set lowest possible DCOx, MODx
	  UCSCTL1 = DCORSEL_3;                      // Set RSELx for DCO = 4.9 MHz
	  UCSCTL2 = FLLD_1 + 74;                    // Set DCO Multiplier for 2.45MHz
	                                            // (N + 1) * FLLRef = Fdco
	                                            // (74 + 1) * 32768 = 2.45MHz
	                                            // Set FLL Div = fDCOCLK/2
	  __bic_SR_register(SCG0);                  // Enable the FLL control loop

	  // Worst-case settling time for the DCO when the DCO range bits have been
	  // changed is n x 32 x 32 x f_MCLK / f_FLL_reference. See UCS chapter in 5xx
	  // UG for optimization.
	  // 32 x 32 x 2.45 MHz / 32,768 Hz = 76563 = MCLK cycles for DCO to settle
	  __delay_cycles(76563);

	  // Loop until XT1,XT2 & DCO fault flag is cleared
	  do
	  {
	    UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG);
	                                            // Clear XT2,XT1,DCO fault flags
	    SFRIFG1 &= ~OFIFG;                      // Clear fault flags
	  }while (SFRIFG1&OFIFG);                   // Test oscillator fault flag
}

void Set_SMCLK8MHZ(void){
	//   ACLK = REFO = 32kHz, MCLK = SMCLK = 8MHz
	UCSCTL3 |= SELREF_2;                       // Set DCO FLL reference = REFO
	UCSCTL4 |= SELA_2;                        // Set ACLK = REFO
	UCSCTL0 = 0x0000;                         // Set lowest possible DCOx, MODx

	  // Loop until XT1,XT2 & DCO stabilizes - In this case only DCO has to stabilize
	while (SFRIFG1 & OFIFG){
	    UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG);
	    													// Clear XT2,XT1,DCO fault flags
	    SFRIFG1 &= ~OFIFG;                      // Clear fault flags
	  }                 									 // Test oscillator fault flag

	  __bis_SR_register(SCG0);                  // Disable the FLL control loop

	  UCSCTL1 = DCORSEL_5;                  // Select DCO range 16MHz operation
	  UCSCTL2 |= 249;                             // Set DCO Multiplier for 8MHz
	                                            		   // (N + 1) * FLLRef = Fdco
	                                            		   // (249 + 1) * 32768 = 8MHz
	  __bic_SR_register(SCG0);                  // Enable the FLL control loop
}

void Set_SMCLK12MHZ(void){
	//   ACLK = REFO = 32kHz, MCLK = SMCLK = 12MHz
	UCSCTL3 |= SELREF_2;                       // Set DCO FLL reference = REFO
	UCSCTL4 |= SELA_2;                        // Set ACLK = REFO
	UCSCTL0 = 0x0000;                         // Set lowest possible DCOx, MODx


	  __bis_SR_register(SCG0);                  // Disable the FLL control loop

	  UCSCTL0 = 0x0000;                         	  // Set lowest possible DCOx, MODx
	  UCSCTL1 = DCORSEL_5;                     // Select DCO range 24MHz operation
	  UCSCTL2 = FLLD_1 + 374;                   // Set DCO Multiplier for 12MHz
	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  // (N + 1) * FLLRef = Fdco
	                                              	  	  	  // (374 + 1) * 32768 = 12MHz
	                                              	  	  	  // Set FLL Div = fDCOCLK/2
	  __bic_SR_register(SCG0);                  	  // Enable the FLL control loop
	  __delay_cycles(375000);
	  // Loop until XT1,XT2 & DCO fault flag is cleared
	    do
	    {
	      UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG);
	                                              // Clear XT2,XT1,DCO fault flags
	      SFRIFG1 &= ~OFIFG;                      // Clear fault flags
	    }while (SFRIFG1&OFIFG);                   // Test oscillator fault flag
}

//VLOCLK:10KHZ->ACLK
//����ϸ
void Set_ACLKVLO(void){
	UCSCTL4 |= SELA_1;
}
//REFCLK:32.768KHZ->ACLK
//��/�ڲ���һ��
void Set_ACLKREF(void){
	UCSCTL4 = UCSCTL4&(~(SELA_7))|SELA_2;
}
//VLOCLK:10KHZ->ACLK
//����ϸ
void Set_ACLKXT1(void){
	 P5SEL |= BIT4+BIT5;                       // Select XT1

	  UCSCTL6 &= ~(XT1OFF);                     // XT1 On
	  UCSCTL6 |= XCAP_3;                        // Internal load cap
	  UCSCTL3 = 0;                              // FLL Reference Clock = XT1

	  // Loop until XT1,XT2 & DCO stabilizes - In this case loop until XT1 and DCo settle
	  do
	  {
	    UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG);
	                                            // Clear XT2,XT1,DCO fault flags
	    SFRIFG1 &= ~OFIFG;                      // Clear fault flags
	  }while (SFRIFG1&OFIFG);                   // Test oscillator fault flag

	  UCSCTL6 &= ~(XT1DRIVE_3);                 // Xtal is now stable, reduce drive strength

	  UCSCTL4 |= SELA_0;                        // ACLK = LFTX1 (by default)
}
void Set_SMCLKREF(void){
	UCSCTL4 = UCSCTL4&(~(SELS_7|SELM_7))|SELS_2|SELM_2; //��SMCLK��MCLK����ΪREFOCLK
}
void Set_SMCLKVLO(void){
	UCSCTL4 = UCSCTL4&(~(SELS_7|SELM_7))|SELS_1|SELM_1; //��SMCLK��MCLK����ΪVLOCLK
}

void Set_SMCLKXT1(void){

	P5SEL |= BIT4|BIT5; //��IO����ΪXT1����
	UCSCTL6 |= XCAP_3;  //���õ���Ϊ12pF
	UCSCTL6 &= ~XT1OFF; //ʹ��XT1
/*
	SetVcoreUp(1); //һ�����Vcore��ѹ�ȼ���������ο��ֲ�
	SetVcoreUp(2);
	SetVcoreUp(3);
*/
	while (SFRIFG1 & OFIFG){
		UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG);         // �������ʱ�ӱ�־λ
		   // ������Ҫ������ֱ�־λ����Ϊ�κ�һ��
		   // ��־λ���ὫOFIFG��λ
		SFRIFG1 &= ~OFIFG;                                  // ���ʱ�Ӵ����־λ
	}
	UCSCTL4 = UCSCTL4&(~(SELS_7|SELM_7))|SELS_0|SELM_0;     //��SMCLK��MCLKʱ��Դ����ΪXT1
}
void Set_SMCLKXT2(void){
	//XT2 sources MCLK & SMCLK = HF XTAL (455kHz - 16MHz)
	P5SEL |= BIT2+BIT3;                       // Port select XT2

	  UCSCTL6 &= ~XT2OFF;                       // Enable XT2
	  UCSCTL3 |= SELREF_2;                      // FLLref = REFO
	                                            // Since LFXT1 is not used,
	                                            // sourcing FLL with LFXT1 can cause
	                                            // XT1OFFG flag to set
	  UCSCTL4 |= SELA_2;                        // ACLK=REFO,SMCLK=DCO,MCLK=DCO

	  // Loop until XT1,XT2 & DCO stabilizes - in this case loop until XT2 settles
	  do
	  {
	    UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG);
	                                            // Clear XT2,XT1,DCO fault flags
	    SFRIFG1 &= ~OFIFG;                      // Clear fault flags
	  }while (SFRIFG1&OFIFG);                   // Test oscillator fault flag

	  UCSCTL6 &= ~XT2DRIVE0;                    // Decrease XT2 Drive according to
	                                            // expected frequency
	  UCSCTL4 |= SELS_5 + SELM_5;               // SMCLK=MCLK=XT2

}
