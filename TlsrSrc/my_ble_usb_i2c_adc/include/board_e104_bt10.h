/********************************************************************************************************
 * @file     board_et104_bt10.h
 *
 * @brief    board configuration for
 *
 * @author 	 pvvx
 *******************************************************************************************************/
#ifndef _BOARD_ET104_BT10_H_
#define _BOARD_ET104_BT10_H_
#pragma once

/* Enable C linkage for C++ Compilers: */
#if defined(__cplusplus)
extern "C" {
#endif

/*
 * ------------- E104_BT10-N/G -------------
 * Default GPIO Settings:
 * PD3: Led White (Led Power) (digital Output)
 * PE1: Led Red   (Led Tx) (digital Output / PWM1) ? SDM_P(SDMDAC)
 * PE0: Led Green (Led Rx) (digital Output / PWM0) ? SDM_N(SDMDAC)
 * PB1: Led Blue  (Led Power2) (digital Output / PWM2)
 * PD2: Key SW0 /
 * PC5: Key SW1 / UART_CTS (digital Input)
 * PE2: USB DM
 * PE3: USB DP
 * PB0: SWS
 * PA7: SWM
 * ----------?
 * PB4: SPI_CN (Output)
 * PB5: SPI_DO (Output)
 * PB6: SPI_DI (Input)
 * PB7: SPI_CK (Output) (R2R_ADC_BAT?)
 * PC2: UART_TX (Output)
 * PC3: UART_RX (Input)
 * PC4: UAR_RTS (Output) (Led Debug?/PWM5?)
 * PC5: UART_CTS (Input)
 * PA3: I2C_SDA (Input/Output)
 * PA4: I2C_SCK (Input/Output)
 * PA0: DMIC_DI(digital microphone)
 * PA1: DMIC_CLK(digital microphone)
 * -----------------------------------------
 * *
 * ---------SoC TLSR8269F512ET/AT32---------
 * *** PortA
 * ANA_A0:  DMIC_DI(digital microphone)/PWM0
 * ANA_A1:  DMIC_CLK(digital microphone)
 * ANA_A2:  SPI_DO/PWM0_N
 * ANA_A3:  SPI_DI/I2C_SDA/PWM1
 * ANA_A4:  SPI_CK/I2C_SCK/PWM1_N
 * ANA_A7:  UART_RX/SWM
 * ANA_B0:  PWM2/SWS
 * ANA_B1:  PWM2_N
 * ANA_B4:  SPI_CN/PWM4
 * ANA_B5:  SPI_DO/PWM4_N
 * ANA_B6:  SPI_DI/I2C_SDA/PWM5
 * ANA_B7:  SPI_CK/I2C_SCK/PWM5_N/R2R_ADC_BAT
 * *** PortC
 * ANA_C2:  UART_TX(digital Output)/PWM2/32KHz crystal output
 * ANA_C3:  UART_RX(digital Input)/PWM3/32KHz crystal input
 * ANA_C4:  UAR_RTS(digital Output)/PWM4
 * ANA_C5:  UART_CTS(digital Input)/PWM5
 * *** PortD
 * ANA_D2:  GPIO4
 * ANA_D3:  GPIO5
 * *** PortE
 * ANA_E0:  PWM0/SDM_P(SDMDAC)
 * ANA_E1:  PWM1/SDM_N(SDMDAC)
 * ANA_E2:  USB_DM
 * ANA_E3:  USB_DP
 * -----------------------------------------
 *
*/

/* TLSR8269F512ET/AT32: Embedded 32-bit high performance MCU with clock up to 48MHz. */
#define CHIP_TYPE           CHIP_TYPE_8269      // 8866-24, 8566-32
#define CHIP_PACKAGE 		CHIP_PACKAGE_QFN32
#define MCU_CORE_TYPE		MCU_CORE_8269
/* Running chip flash size select. If '1' - 1M, otherwise (or undefined) - 512K. */
#define FLASH_SIZE_1M		0
#define USE_EXT_32K_CRYSTAL 0

#ifndef CLOCK_SYS_TYPE
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
#define SET_PLL 		QUARTZ_16MHZ	// QUARTZ_16MHZ, QUARTZ_12MHZ

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
#endif // #ifndef CLOCK_SYS_TYPE
/*
//-------------------------- KEYs
#define	KEY_K1							GPIO_PD2	// GPIO4
#define PD2_FUNC						AS_GPIO		// AS_ADC, AS_GPIO
#define PD2_OUTPUT_ENABLE				0
#define PD2_INPUT_ENABLE				1
#define	PULL_WAKEUP_SRC_PD2				GPIO_PULL_UP_10K

#define	KEY_K2	 						GPIO_PC4	//
#define PC4_FUNC						AS_GPIO		// AS_ADC, AS_GPIO, AS_SPI
#define PC4_OUTPUT_ENABLE				0
#define PC4_INPUT_ENABLE				1
#define	PULL_WAKEUP_SRC_PC4				GPIO_PULL_UP_10K

#define	KEY_K2	 						GPIO_PC5	// UART_CTS/PWM5
#define PC5_FUNC						AS_GPIO		// AS_ADC, AS_GPIO, AS_SPI
#define PC5_OUTPUT_ENABLE				0
#define PC5_INPUT_ENABLE				1
#define	PULL_WAKEUP_SRC_PC5				PM_PIN_PULLUP_10K

enum{
	VK_SW1 = 0x01,
	VK_SW2 = 0x02
};

#define	KB_MAP_NORMAL	{\
		{VK_SW1,}, \
		{VK_SW2,}, }

#define	KB_MAP_NUM		KB_MAP_NORMAL
#define	KB_MAP_FN		KB_MAP_NORMAL

#define KB_DRIVE_PINS  {NULL }
#define KB_SCAN_PINS   {BUTTON1,  BUTTON2}
*/
//-------------------------- LEDs
/*
#define LED_W							GPIO_PB6
#define PB6_FUNC						AS_GPIO		// AS_ADC, AS_GPIO
#define PB6_OUTPUT_ENABLE				1
#define PB6_INPUT_ENABLE				0

#define LED_R							GPIO_PE1
#define PE1_FUNC						AS_GPIO // AS_PWM 		// AS_ADC, AS_GPIO, AS_PWM(1), AS_SDM
#define PE1_OUTPUT_ENABLE				1
#define PE1_INPUT_ENABLE				0

#define LED_G							GPIO_PE0
#define PE0_FUNC						AS_GPIO // AS_PWM 		// AS_ADC, AS_GPIO, AS_PWM(0), AS_SDM
#define PE0_OUTPUT_ENABLE				1
#define PE0_INPUT_ENABLE				0

#define LED_B							GPIO_PB1
#define PB1_FUNC						AS_GPIO // AS_PWM		// AS_ADC, AS_GPIO, AS_PWM(2N)
#define PB1_OUTPUT_ENABLE				1
#define PB1_INPUT_ENABLE				0

#define LED_POWER						LED_W
#define LED_POWER_TOGLE()		do {reg_gpio_out(LED_POWER) ^=  LED_POWER & 0xff;} while(0)
#define LED_POWER_ON()			do {reg_gpio_out(LED_POWER) |=  LED_POWER & 0xff;} while(0)
#define LED_POWER_OFF()			do {reg_gpio_out(LED_POWER) &=  ~(LED_POWER & 0xff);} while(0)
*/
#define ON            						1
#define OFF           						0

//-------------------------- SWIRE
#define PA7_FUNC 						AS_SWIRE // SWM
#define PA7_INPUT_ENABLE				1		 // SWM input enable

#define PB0_FUNC						AS_SWIRE // SWS
#define PULL_WAKEUP_SRC_PB0           	PM_PIN_PULLUP_1M  // SWS, should be pulled up, otherwise single wire would be triggered

//-------------------------- UART
#if 0 //	(!BLE_AUDIO_ENABLE && USE_UART)
	#define UART_TX_PIN         		GPIO_PC2
	#define PC2_FUNC                	AS_UART
	#define PC2_INPUT_ENABLE        	0
	#define PC2_OUTPUT_ENABLE       	1
	#define PC2_DATA_STRENGTH       	0

	#define UART_RX_PIN         		GPIO_PC3
	#define PC3_FUNC                	AS_UART
	#define PC3_INPUT_ENABLE        	1
	#define PC3_OUTPUT_ENABLE       	0
	#define PC3_DATA_STRENGTH       	0
	#define PULL_WAKEUP_SRC_PC3     	GPIO_PULL_UP_10K

	#define UART_PIN_CFG				UART_GPIO_CFG_PC2_PC3()
#endif

//-------------------------- USB
#if USE_USB
#define PE2_FUNC	AS_USB
#define PE3_FUNC	AS_USB
#define PE2_INPUT_ENABLE	1
#define PE3_INPUT_ENABLE	1
#define PULL_WAKEUP_SRC_PE2           	PM_PIN_PULLDOWN_100K // PM_PIN_PULLUP_1M  // USB DM
#define PULL_WAKEUP_SRC_PE3           	PM_PIN_PULLDOWN_100K // PM_PIN_PULLUP_1M  // USB DP
#endif

// DEBUG PRINTF?
#if (UART_PRINTF_MODE || PRINT_DEBUG_INFO)
	#define PRINT_BAUD_RATE             1000000 //1M baud rate,should Not bigger than 1M, when system clock is 16M.
	//defination debug printf pin
	#define	DEBUG_INFO_TX_PIN	    	GPIO_PB5	//	print
	#define PB5_OUTPUT_ENABLE			1
	#define PB5_INPUT_ENABLE			0
	#define PULL_WAKEUP_SRC_PB5    		PM_PIN_PULLUP_1M
#endif
//------------------------- WDT

/* Disable C linkage for C++ Compilers: */
#if defined(__cplusplus)
}
#endif

#endif // _BOARD_ET104_BT10_H_
