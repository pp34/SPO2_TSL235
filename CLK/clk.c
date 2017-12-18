/*
 * clk.c
 *
 *  Created on: 2016年7月28日
 *      Author: PP
 */
#include <msp430.h>
//#include "HAL_PMM.h"
#include "clk.h"

/*(1)XT1处于LF模式作为XT1CLK时钟源。ACLK选通为XT1CLK。
**(2)MCLK选通为DCOCLKDIV
**(3)SMCLK选通为DCOCLKDIV
**(4)FLL使能，且将XT1CLK作为FLL参考时钟。
**(5)XIN和XOUT脚设置为通用IO，XIN和XOUT配置为XT1功能前，XT1保持禁用。
**(6)如果可用的话，XT2IN和XT2OUT被设置为通用IO且保持禁止状态。
*/
/*REFOCLK、VLOCLK、DCOCLK（这里暂时这么认为）默认状态下是可用的，
 * 所以，切换的时候只需要通过UCSCTL4来配置
 * ACLK、SMCLK和MCLK的时钟源即可，而XT1CLK和XT2CLK需要根据硬件的具体配置情况确定
 * */
//CPUCLK->MCLK&SCLK 32.768KHZ
//ACLK			 default 	 32.768KHZ
void CLK_Init(void){

	Set_ACLKREF();
	Set_SMCLK8MHZ();
}

//DCO模块运行需要参考时钟REFCLK，REFCLK可以来自REFOCLK、XT1CLK和XT2CLK，
//通过UCSCTL3的SELREF选择，默认使用的XT1CLK，但如果XT1CLK不可用则使用REFOCLK。
/*	DCO倍频公式
 * DCOCLK = D*(N+1)*(REFCLK/n)		default : 2097152	 * DCOCLKDIV = (N+1)*(REFCLK/n)		default : 1048576
 * D可以通过UCSCTL2中的FLLD来设定，默认为1，也就是2分频
 * N可以通过UCSCTL2中的FLLN来设定，默认值为32
 */
//DCORSEL---分8频率段
//DCOX-------再分32频率阶
/*******************************************************
 * DCORSEL = 0的调节范围约为0.20 ~ 0.70MHZ；
 * DCORSEL= 1的调节范围约为0.36  ~ 1.47MHZ；
 * DCORSEL = 2的调节范围约为0.75 ~ 3.17MHZ；
 * DCORSEL = 3的调节范围约为1.51 ~ 6.07MHZ；
 * DCORSEL = 4的调节范围约为3.2   ~ 12.3MHZ；
 * DCORSEL = 5的调节范围约为6.0   ~ 23.7MHZ；
 * DCORSEL = 6的调节范围约为10.7 ~ 39.7MHZ；
 * DCORSEL = 7的调节范围约为19.6 ~ 60   MHZ。
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
//不精细
void Set_ACLKVLO(void){
	UCSCTL4 |= SELA_1;
}
//REFCLK:32.768KHZ->ACLK
//低/内部但一般
void Set_ACLKREF(void){
	UCSCTL4 = UCSCTL4&(~(SELA_7))|SELA_2;
}
//VLOCLK:10KHZ->ACLK
//不精细
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
	UCSCTL4 = UCSCTL4&(~(SELS_7|SELM_7))|SELS_2|SELM_2; //将SMCLK和MCLK配置为REFOCLK
}
void Set_SMCLKVLO(void){
	UCSCTL4 = UCSCTL4&(~(SELS_7|SELM_7))|SELS_1|SELM_1; //将SMCLK和MCLK配置为VLOCLK
}

void Set_SMCLKXT1(void){

	P5SEL |= BIT4|BIT5; //将IO配置为XT1功能
	UCSCTL6 |= XCAP_3;  //配置电容为12pF
	UCSCTL6 &= ~XT1OFF; //使能XT1
/*
	SetVcoreUp(1); //一次提高Vcore电压等级，具体请参考手册
	SetVcoreUp(2);
	SetVcoreUp(3);
*/
	while (SFRIFG1 & OFIFG){
		UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG);         // 清除三类时钟标志位
		   // 这里需要清除三种标志位，因为任何一种
		   // 标志位都会将OFIFG置位
		SFRIFG1 &= ~OFIFG;                                  // 清除时钟错误标志位
	}
	UCSCTL4 = UCSCTL4&(~(SELS_7|SELM_7))|SELS_0|SELM_0;     //将SMCLK和MCLK时钟源配置为XT1
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
