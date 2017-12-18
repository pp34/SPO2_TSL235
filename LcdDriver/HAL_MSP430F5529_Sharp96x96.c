//*****************************************************************************
//
// HAL_MSP-EXP430F5529_Sharp96x96.c
//
//*****************************************************************************
//
//! \addtogroup display_api
//! @{
//
//*****************************************************************************

#include "grlib.h"
#include "HAL_MSP430F5529_Sharp96x96.h"
#include "driverlib.h"


//*****************************************************************************
//
//! Initializes the display driver.
//!
//! This function initializes the Sharp96x96 display. This function
//! configures the GPIO pins used to control the LCD display when the basic
//! GPIO interface is in use. On exit, the LCD has been reset and is ready to
//! receive command and data writes.
//!
//! \return None.
//
//*****************************************************************************
void HAL_LCD_initDisplay(void){
	//CLK--3.2; MOSI--3.0; MISO--3.1
	P3SEL |= BIT0 + BIT2;
	P3DIR |= BIT0 + BIT2;
	//DISP--1.6; LPC--6.5; ECI--2.0
	P1DIR |= BIT6;
	P6DIR |= BIT5;
	P1OUT &=~ BIT6;
	P6OUT |= BIT5;
	//CS--6.6
	P6DIR |= BIT6;

	HAL_LCD_clearCS();

	USCI_B_SPI_initMasterParam  param ={0};
	param.selectClockSource = USCI_B_SPI_CLOCKSOURCE_SMCLK ;
	param.clockSourceFrequency = UCS_getSMCLK();
	param.desiredSpiClock = 1000000;
	param.msbFirst = USCI_B_SPI_MSB_FIRST;
	param.clockPhase = USCI_B_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT;
	param.clockPolarity =USCI_B_SPI_CLOCKPOLARITY_INACTIVITY_LOW;
//	param.spiMode = USCI_B_SPI_3PIN;
/*
	{
		EUSCI_B_SPI_CLOCKSOURCE_SMCLK,
		8000000,
		1000000,
		EUSCI_B_SPI_MSB_FIRST,
		EUSCI_B_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT,
		EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_LOW,
		EUSCI_B_SPI_3PIN
	};
*/
	USCI_B_SPI_initMaster(USCI_B0_BASE,&param);

	USCI_B_SPI_enable(USCI_B0_BASE);
}



//*****************************************************************************
//
// Writes command or data to the LCD Driver
//
// \param ucCmdData is the 8 or 16 bit command to send to the LCD driver
// Uses the SET_LCD_DATA macro
//
// \return None
//
//*****************************************************************************
void HAL_LCD_writeCommandOrData(unsigned int command){
	UCB0TXBUF = command;
	while (UCB0STAT & UCBUSY);
}

//*****************************************************************************
//
// Clears CS line
//
// This macro allows to clear the Chip Select (CS) line
//
// \return None
//
//*****************************************************************************
void HAL_LCD_clearCS(void) {
	P6OUT &= ~BIT6;
}

//*****************************************************************************
//
// Set CS line
//
// This macro allows to set the Chip Select (CS) line
//
// \return None
//
//*****************************************************************************
void HAL_LCD_setCS(void) {
	P6OUT |= BIT6;
}

//*****************************************************************************
//
// Waits until the SPI communication with the LCD is finished a command to
// the LCD Driver
//
// \param None
//
// \return None
//*****************************************************************************
void HAL_LCD_waitUntilLcdWriteFinish(void)
{
	while (USCI_B_SPI_isBusy(LCD_EUSCI_BASE));
}

//*****************************************************************************
//
// Disables Shapr96x96 LCD
//
// \param None
//
// \return None
//*****************************************************************************
void HAL_LCD_disableDisplay(void)
{
	P1OUT &= ~BIT6;
	P6OUT &= ~BIT5;
}

//*****************************************************************************
//
// Enables Shapr96x96 LCD
//
// \param None
//
// \return None
//*****************************************************************************
void HAL_LCD_enableDisplay(void)
{
	P1OUT |= BIT6;
	P6OUT |= BIT5;

}


//*****************************************************************************
//
// Prepare to write memory
//
// This macro unlocks flash memory controller and
// sets access right on flash controller
//
// \return None
//
//*****************************************************************************
void HAL_LCD_prepareMemoryWrite()
{
}


//*****************************************************************************
//
// Finish memory writing
//
// This macro removes access rights on flash controller and
// locks flash memory controller.
//
// \return None
//
//*****************************************************************************
void HAL_LCD_finishMemoryWrite()
{
	__no_operation();
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
