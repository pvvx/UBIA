/********************************************************************************************************
 * @file     board_et104_bt05.h
 *
 * @brief    board configuration for
 *
 * @author 	 pvvx
 *******************************************************************************************************/
#ifndef _BOARD_ET104_BT05_H_
#define _BOARD_ET104_BT05_H_
#pragma once

/* Enable C linkage for C++ Compilers: */
#if defined(__cplusplus)
extern "C" {
#endif

/*
 * ------------- E104-BT05-TB -------------
 * *** PortA
 * ANA_A0:  SWS		i/o
 * ANA_A1:  PWM3(1)	BLUE LED
 * ANA_A5:  PWM4(2) BLUE LED
 * *** PortB
 * ANA_B0:  PWM5(3) output
 * ANA_B5:  USB DM	i/o
 * ANA_B6:  USB DP	i/o
 * *** PortC
 * ANA_C0:  MOD		input	// KEY_MOD
 * ANA_C1:  DISC	output? // ADC?
 *-ANA_C2:  PWM1_N			// не выведен!
 * ANA_C4:  PWM2(0)	output
 * ANA_C6:  UART_TX	output
 * ANA_C7:  UART_RX input
 * *** PortD
 * ANA_D4:  IO0		KEY_SW0
 * ANA_D5:  IO1		input	// KEY_SW1
 * *** PortE
 * ANA_E4:  LINK	YELLOW LED
 * ANA_E5:	WKP		input   // KEY_WKP
 * ANA_E6:  IO2		output	// DEBUG
 * ANA_E7:  DATA	YELLOW LED	// I2C_SDA
 * *** PortF
 * ANA_F0:  IO3		output // ERROR/SCL?
 *-ANA_F1:  SPI_CK/I2C_SCL
 * -----------------------------------------
 * *
 * ---------SoC TLSR8266F512ET/AT32---------
 * *** PortA
 * ANA_A0:  SWS
 * ANA_A1:  PWM3
 * ANA_A5:  PWM4
 * *** PortB
 * ANA_B0:  PWM5
 * ANA_B5:  USB DM
 * ANA_B6:  USB DP
 * *** PortC
 * ANA_C0:  PWM0
 * ANA_C1:  GP1/PWM1_N
 * ANA_C2:  PWM1_N			// не выведен!
 * ANA_C4:  PWM2
 * ANA_C6:  GP4/UART_TX
 * ANA_C7:  GP5/UART_RX
 * *** PortD
 * ANA_D4:  GP10
 * ANA_D5:  GP11
 * *** PortE
 * ANA_E4:  GP16
 * ANA_E5:	GP17
 * ANA_E6:  SPI_CN/UART_RTS
 * ANA_E7:  SPI_DI/I2C_SDA
 * *** PortF
 * ANA_F0:  SPI_DO/UART_CTS
 * ANA_F1:  SPI_CK/I2C_SCL	// не выведен!
 * -----------------------------------------
 *
*/

/* TLSR8266F512ET/AT32: Embedded 32-bit high performance MCU with clock up to 48MHz. */
#define CHIP_TYPE           CHIP_TYPE_8266      // 8866-24, 8566-32
#define CHIP_PACKAGE 		CHIP_PACKAGE_QFN32
#define MCU_CORE_TYPE		MCU_CORE_8266
/* Running chip flash size select. If '1' - 1M, otherwise (or undefined) - 512K. */
#define FLASH_SIZE_1M		0
#define USE_EXT_32K_CRYSTAL 0

//#ifndef CLOCK_SYS_TYPE
/*--------------- System clock config -----------------------
There are four selectable clock sources for MCU system clock:
* 	32MHz RC clock,
* 	HS divider clock (divided from a High speed clock),
* 	Pad clock (12MHz/16MHz,32.768KHz).
The high speed clock (FHS) is selectable via address
{0x70[0], 0x66[7?]} from the following sources:
* 	192MHz clock from PLL,
* 	32MHz RC clock,
* 	12MHz/16MHz Pad clock.*/
#define	SYS_CLK_HS_DIV	0 // Analog Telink SDK: CLOCK_TYPE_PLL (48,32,..MHz)
#define	SYS_CLK_RC		1 // Analog Telink SDK: CLOCK_TYPE_OSC (32,16,..MHz)
#define	SYS_CLK_QUARTZ	2 // Analog Telink SDK: CLOCK_TYPE_PAD (16,8,..MHz)

#define	QUARTZ_16MHZ	16
#define	QUARTZ_12MHZ	12
#define SET_PLL 		QUARTZ_12MHZ	// QUARTZ_16MHZ, QUARTZ_12MHZ

#if (SET_PLL == QUARTZ_16MHZ)
#define CLK_QUARTZ 16000000
#elif (SET_PLL == QUARTZ_12MHZ)
#define CLK_QUARTZ 12000000
#else
#error Set SET_PLL QUARTZ_16MHZ or QUARTZ_12MHZ!
#endif

#define CLOCK_SYS_TYPE  SYS_CLK_HS_DIV  // SYS_CLK_HS_DIV, SYS_CLK_RC, SYS_CLK_QUARTZ

#if (CLOCK_SYS_TYPE == SYS_CLK_HS_DIV)
#define CLOCK_SYS_CLOCK_HZ  	16000000	// 192000000/n, n = 4..15 : 48000000, ..
#elif (CLOCK_SYS_TYPE == SYS_CLK_RC)
#define CLOCK_SYS_CLOCK_HZ  	32000000	// 32000000/n, n = 1..15
#elif (CLOCK_SYS_TYPE == SYS_CLK_QUARTZ)
#define CLOCK_SYS_CLOCK_HZ  	(CLK_QUARTZ/1)	// 16000000/n or 12000000/n, n = 1..15
#else
#error Set CLOCK_SYS_CLOCK_HZ SYS_CLK_HS_DIV, SYS_CLK_RC, SYS_CLK_QUARTZ!
#endif
//#endif // #ifndef CLOCK_SYS_TYPE
// PE6 CN/RTS	OUT2 - DEBUG
// PE7 DI/SDA	OUT3
// PF0 DO/CTS	OUT4
// PF1 CK/SCL	K5
//-------------------------- KEYs
/*
#define	KEY_PWRC						GPIO_PD4 // PWRC
#define PD4_FUNC						AS_GPIO
#define PD4_OUTPUT_ENABLE				0
#define PD4_INPUT_ENABLE				1
#define	PULL_WAKEUP_SRC_PD4				PM_PIN_PULLUP_10K
*/

//#define PD5_OUTPUT_ENABLE				0
//#define PD5_INPUT_ENABLE				0
//#define PD5_DATA_STRENGTH				0
//#define PD5_DATA_OUT					0

#if 0
#define LED_PWM0						GPIO_PC0 // PWM0(R)
#define PC0_FUNC						AS_GPIO // AS_PWM
#define PC0_OUTPUT_ENABLE				1
//#define PC0_INPUT_ENABLE				0
#endif
#if 0
#define LED_PWM1						GPIO_PC2 // PWM1(B)
#define PC2_FUNC						AS_GPIO // AS_PWM
#define PC2_OUTPUT_ENABLE				1
//#define PC2_INPUT_ENABLE				0
#endif
#if 1
#define GPIO_VBAT						GPIO_PC4 // PWM2(G)
//#define PC4_FUNC						AS_ADC
#else
#define LED_PWM2						GPIO_PC4 // PWM2(G)
#define PC4_FUNC						AS_GPIO // AS_PWM
//#define PC4_OUTPUT_ENABLE				1
//#define PC4_INPUT_ENABLE				0
#endif

//#define LED_PWM3						GPIO_PA1 // PWM3(W)
//#define PA1_FUNC						AS_GPIO // AS_PWM
//#define PA1_OUTPUT_ENABLE				1
//#define PA1_INPUT_ENABLE				0
/*
#define LED_PWM4						GPIO_PA5 // STAT
#define PA5_FUNC						AS_PWM
#define PA5_OUTPUT_ENABLE				1
#define PA5_INPUT_ENABLE				0

#define LED_PWM5						GPIO_PB0 // K1
#define PB0_FUNC						AS_PWM
#define PB0_OUTPUT_ENABLE				1
#define PB0_INPUT_ENABLE				0
*/
//#define LED_POWER						LED_ADV
//#define LED_POWER_TOGLE()		do {reg_gpio_out(LED_POWER) ^=  LED_POWER & 0xff;} while(0)
//#define LED_POWER_ON()			do {reg_gpio_out(LED_POWER) |=  LED_POWER & 0xff;} while(0)
//#define LED_POWER_OFF()			do {reg_gpio_out(LED_POWER) &=  ~(LED_POWER & 0xff);} while(0)

#define ON            						1
#define OFF           						0

//-------------------------- SWIRE
#define PA0_FUNC						AS_SWIRE // SWS
#define PULL_WAKEUP_SRC_PA0           	PM_PIN_PULLUP_1M  // SWS, should be pulled up, otherwise single wire would be triggered

//-------------------------- UART
#if	USE_UART
	#define UART_TX_PIN         		GPIO_PC6	// TX
	#define PC6_FUNC                	AS_UART
	#define PC6_INPUT_ENABLE        	0
	#define PC6_OUTPUT_ENABLE       	1
	#define PC6_DATA_STRENGTH       	0

	#define UART_RX_PIN         		GPIO_PC7	// RX
	#define PC7_FUNC                	AS_UART
	#define PC7_INPUT_ENABLE        	1
	#define PC7_OUTPUT_ENABLE       	0
	#define PC7_DATA_STRENGTH       	0
	#define PULL_WAKEUP_SRC_PC7     	GPIO_PULL_UP_10K

	#define UART_PIN_CFG				UART_GPIO_CFG_PC6_PC7()
#endif

//-------------------------- USB
#if USE_USB
#define PB5_FUNC	AS_USB
#define PB6_FUNC	AS_USB
#define PB5_INPUT_ENABLE	1
#define PB6_INPUT_ENABLE	1
#define PULL_WAKEUP_SRC_PB5           	PM_PIN_PULLDOWN_100K // PM_PIN_PULLUP_1M  // USB DM //K2
#define PULL_WAKEUP_SRC_PB6           	PM_PIN_PULLDOWN_100K // PM_PIN_PULLUP_1M  // USB DP //K3
#endif

//--------------------- DEBUG PRINTF
#if (UART_PRINTF_MODE || PRINT_DEBUG_INFO)
	#define PRINT_BAUD_RATE             1000000 //1M baud rate,should Not bigger than 1M, when system clock is 16M.
	//defination debug printf pin
	#define	DEBUG_INFO_TX_PIN	    	GPIO_PE6	//	print / RTS / OUT2
	#define PE6_OUTPUT_ENABLE			1
	#define PE6_INPUT_ENABLE			0
	#define PULL_WAKEUP_SRC_PE1    		PM_PIN_PULLUP_1M
#endif
//------------------------- WDT

/* Disable C linkage for C++ Compilers: */
#if defined(__cplusplus)
}
#endif

#endif // _BOARD_ET104_BT05_H_
