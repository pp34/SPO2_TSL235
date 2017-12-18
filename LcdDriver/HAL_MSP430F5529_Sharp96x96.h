#ifndef __HAL_MSP430F5529_SHARP96x96_H__
#define __HAL_MSP430F5529_SHARP96x96_H__


//*****************************************************************************
//
// User Configuration for the LCD Driver
//
//*****************************************************************************

// Ports from MSP430 connected to LCD
#define LCD_SPI_SI_PORT                       GPIO_PORT_P3	//3.0
#define LCD_SPI_CLK_PORT                    GPIO_PORT_P3	//3.2
#define LCD_DISP_PORT                        GPIO_PORT_P1	//1.6
#define LCD_POWER_PORT                    GPIO_PORT_P6	//6.5
#define LCD_SPI_CS_PORT	                    GPIO_PORT_P6	//6.6


// Pins from MSP430 connected to LCD
#define LCD_SPI_SI_PIN                      GPIO_PIN0
//#define LCD_SPI_SI_PIN_FUNCTION             GPIO_SECONDARY_MODULE_FUNCTION
#define LCD_SPI_CLK_PIN                     GPIO_PIN2
//#define LCD_SPI_CLK_PIN_FUNCTION            GPIO_SECONDARY_MODULE_FUNCTION
#define LCD_DISP_PIN                        GPIO_PIN6
#define LCD_POWER_PIN                       GPIO_PIN5
#define LCD_SPI_CS_PIN                      GPIO_PIN6


// Definition of USCI base address to be used for SPI communication
#define LCD_EUSCI_BASE		      USCI_B0_BASE


// Non-volatile Memory used to store DisplayBuffer
#define NON_VOLATILE_MEMORY_BUFFER
#ifdef NON_VOLATILE_MEMORY_BUFFER
//#define USE_FLASH_BUFFER
#define NON_VOLATILE_MEMORY_ADDRESS			0xf400
#endif //NON_VOLATILE_MEMORY_BUFFER


//*****************************************************************************
//
// Prototypes for the globals exported by this driver.
//
//*****************************************************************************
extern void HAL_LCD_initDisplay(void);
extern void HAL_LCD_writeCommandOrData(unsigned int command);
extern void HAL_LCD_clearCS(void);
extern void HAL_LCD_setCS(void);
extern void HAL_LCD_prepareMemoryWrite(void);
extern void HAL_LCD_finishMemoryWrite(void);
extern void HAL_LCD_waitUntilLcdWriteFinish(void);
extern void HAL_LCD_disableDisplay(void);
extern void HAL_LCD_enableDisplay(void);


#endif // __HAL_MSP_EXP430F5529_SHARPLCD_H__
