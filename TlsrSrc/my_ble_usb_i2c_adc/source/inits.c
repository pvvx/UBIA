/*
 * inits.c
 *
 *  Created on: 11.10.2019
 *      Author: pvvx
 */
#include "proj/tl_common.h"
#include "inits.h"

/*
 * Обработка таблиц инициализации
 */
void LoadTbl32Set(const TBLCMDSET32 * pt, int size) {
	unsigned char ccmd;
	unsigned int cadr;
	unsigned int cdat;
	int i;
	for( i = 0; i < size; i++ ) {
		ccmd = pt[i].cmd;
		cadr = pt[i].adr | 0x800000;
		cdat = pt[i].dat;
		if (ccmd == TCMD_SWR32) {
			write_reg32(cadr, cdat);
		}
		else
		if (ccmd == TCMD_SWR16) {
			write_reg16(cadr , cdat);
		}
		else
		if (ccmd == TCMD_SWR8) {
			write_reg8(cadr, cdat);
		}
		else
		if (ccmd == TCMD_SOR8) {
			write_reg8(cadr, (read_reg8(cadr) & (cdat >> 16)) | cdat);
		}
		else
		if (ccmd == TCMD_SOR16) {
			write_reg16(cadr, (read_reg16(cadr) & (cdat >> 16)) | cdat);
		}
		else
		if (ccmd == TCMD_SSET32) {
			write_reg32(cadr, read_reg32(cadr) | cdat);
		}
		else
		if (ccmd == TCMD_AWR) {
			analog_write(cadr, cdat);
		}
		else
		if (ccmd == TCMD_AOR) {
			analog_write(cadr, (analog_read(cadr) & (cdat >> 16)) | cdat);
		}
	}
}
/*
 * Таблица стартовой инициализации регистров SoC
 */
static const TBLCMDSET32 tbl_soc_ini[] = {

	// reg_rst_clk0 // 0xDF010020
	{ TCMD_SWR32, 0x060, 0
#if USE_SPI
		| FLD_CLK_SPI_EN
#endif
#if USE_I2C
//		| FLD_CLK_I2C_EN // включается в I2CBusInit()
#endif
#if USE_USB
		| FLD_CLK_USB_EN
		| FLD_CLK_USB_PHY_EN
#endif
		| FLD_CLK_MCU_EN
		| FLD_CLK_MAC_EN
#if USE_ADC
		| FLD_CLK_ADC_EN
#endif
//		| FLD_CLK_ZB_EN
	},
	// reg_clk_en: 0x002637FF
	{ TCMD_SWR16, 0x064, 0
		| FLD_CLK_GPIO_EN	// system timer
		| FLD_CLK_ALGM_EN
#if USE_USB | USE_DMA
		| FLD_CLK_DMA_EN
#endif
#if USE_UART
		| FLD_CLK_UART_EN
#endif
#if USE_PWM
		| FLD_CLK_PWM_EN
#endif
#if USE_AES
		| FLD_CLK_AES_EN
#endif
//		| FLD_CLK_32K_TIMER_EN	// clk32k for system timer
		| FLD_CLK_PLL_EN
#if (PA7_FUNC==AS_SWIRE) || (PB0_FUNC==AS_SWIRE)
		| FLD_CLK_SWIRE_EN
#endif
//		| FLD_CLK_32K_QDEC_EN	// 32k for qdec
#if USE_AUD
		| FLD_CLK_AUD_EN
#endif
#if USE_DFIFO
		| FLD_CLK_DIFIO_EN
#endif
#if USE_KEYSCAN
		| FLD_CLK_KEYSCAN_EN
#endif
		| FLD_CLK_MCIC_EN
#if	USE_QDEC
		| FLD_CLK_QDEC_EN
#endif
	},
    // rega_dcdc_ctrl:
	{ TCMD_AWR,  0x0088, 0x0f }, // rega_dcdc_ctrl
	// rega_pwdn_setting1:
	{ TCMD_AWR,  0x0005, 0x62 }, // FLDA_32K_XTAL_PWDN, FLDA_BGIREF_3V_PWDN, FLDA_COMP_PWDN
#if (CLOCK_SYS_TYPE == SYS_CLK_HS_DIV)
	{ TCMD_SWR32, 0x070, 0 // = 0x04000400
	/* reg_fhs_sel [0x70], After reset = 0x00 */
		| (0) // bit1 FHS sel: 192M clock from pll | 32M clock from rc osc
	/* reg_dcdc_clk [0x71], After reset [0x71] = 0x04 */
		| ((1<<2)<<8)
	/* reg_?? [0x72], After reset [0x72] = 0x00 */
		| (0<<16) // watchdog reset status bit 0x72[0] = 1, manually clear - write '1'
	/* reg_clk_mux_cel [0x73], After reset  = 0x14
	* [0] clk32k select; 0: sel 32k osc 1: 32k pad
	* [1] dmic clock select, 1: select 32k (refer to bit[0] to decide which 32k ; 0: dmic clk div
	* [2] usb phy clock select, 1 : 192M divider 0: 48M pll
	* [7:4] r_lpr_div, decide system clock speed in low power mode	 */
		| ((1<<2)<<24)
	},
	// reg_clk_sel [0x66], After reset = 0x06
	{ TCMD_SWR8, 0x066, MASK_VAL(FLD_CLK_SEL_DIV, CLOCK_PLL_CLOCK/CLOCK_SYS_CLOCK_HZ, FLD_CLK_SEL_SRC, CLOCK_SEL_HS_DIV)},
#elif (CLOCK_SYS_TYPE == SYS_CLK_RC)
#if(CLOCK_SYS_CLOCK_HZ != 32000000)
#error Set CLOCK_SYS_CLOCK_HZ 32000000!
#endif
	{ TCMD_SWR32, 0x070, 0
	/* reg_fhs_sel [0x70], After reset = 0x00 */
		| (0) // bit1 FHS sel: 192M clock from pll | 32M clock from rc osc
	/* reg_dcdc_clk [0x71], After reset [0x71] = 0x04 */
		| ((1<<2)<<8)
	/* reg_?? [0x72], After reset [0x72] = 0x00 */
		| (0<<16) // watchdog reset status bit 0x72[0] = 1, manually clear - write '1'
	/* reg_clk_mux_cel [0x73], After reset  = 0x14
	* [0] clk32k select; 0: sel 32k osc 1: 32k pad
	* [1] dmic clock select, 1: select 32k (refer to bit[0] to decide which 32k ; 0: dmic clk div
	* [2] usb phy clock select, 1 : 192M divider 0: 48M pll
	* [7:4] r_lpr_div, decide system clock speed in low power mode	 */
		| ((1<<2)<<24)
	},
	// reg_clk_sel [0x66], After reset = 0x06
	// reg_clk_sel must be zero
	{ TCMD_SWR8, 0x066, MASK_VAL(FLD_CLK_SEL_DIV, 0, FLD_CLK_SEL_SRC, CLOCK_SEL_32M_RC)},
#elif (CLOCK_SYS_TYPE == SYS_CLK_QUARTZ)
#if(CLOCK_SYS_CLOCK_HZ == CLK_QUARTZ)
	{ TCMD_SWR32, 0x070, 0
	/* reg_fhs_sel [0x70], After reset = 0x00 */
		| (1) // bit1 FHS sel: 192M clock from pll | 32M clock from rc osc
	/* reg_dcdc_clk [0x71], After reset [0x71] = 0x04 */
		| ((1<<2)<<8)
	/* reg_?? [0x72], After reset [0x72] = 0x00 */
		| (0<<16) // watchdog reset status bit 0x72[0] = 1, manually clear - write '1'
	/* reg_clk_mux_cel [0x73], After reset  = 0x14
	* [0] clk32k select; 0: sel 32k osc 1: 32k pad
	* [1] dmic clock select, 1: select 32k (refer to bit[0] to decide which 32k ; 0: dmic clk div
	* [2] usb phy clock select, 1 : 192M divider 0: 48M pll
	* [7:4] r_lpr_div, decide system clock speed in low power mode	 */
		| ((1<<2)<<24)
	},
	// reg_clk_sel [0x66], After reset = 0x06
	{ TCMD_SWR8, 0x066, MASK_VAL(FLD_CLK_SEL_DIV, 0, FLD_CLK_SEL_SRC, CLOCK_SEL_16M_PAD) },
#elif (CLOCK_SYS_CLOCK_HZ < CLK_QUARTZ) && (CLOCK_SYS_CLOCK_HZ >= (CLK_QUARTZ/15))
	/* reg_fhs_sel [0x70], After reset = 0x00 */
	{ TCMD_SWR32, 0x070, 0
		| (1) // bit1 FHS sel: 32M clock from rc osc
	/* reg_dcdc_clk [0x71], After reset [0x71] = 0x04 */
		| ((1<<2)<<8)
	/* reg_?? [0x72], After reset [0x72] = 0x00 */
		| (0<<16) // watchdog reset status bit 0x72[0] = 1, manually clear - write '1'
	/* reg_clk_mux_cel [0x73], After reset  = 0x14
	* [0] clk32k select; 0: sel 32k osc 1: 32k pad
	* [1] dmic clock select, 1: select 32k (refer to bit[0] to decide which 32k ; 0: dmic clk div
	* [2] usb phy clock select, 1 : 192M divider 0: 48M pll
	* [7:4] r_lpr_div, decide system clock speed in low power mode	 */
		| ((1<<2)<<24)
	},
	// reg_clk_sel [0x66], After reset = 0x06
	{ TCMD_SWR8, 0x066, MASK_VAL(FLD_CLK_SEL_DIV, CLK_QUARTZ/CLOCK_SYS_CLOCK_HZ, FLD_CLK_SEL_SRC, CLOCK_SEL_HS_DIV) },
#endif
#else
	#error Set CLOCK_SYS_CLOCK_HZ (CLK_QUARTZ/n), n = 1..15!
#endif
	// 0x68 = 0x00020002 { TCMD_SWR32, 0x068, 0x00020002 },
	// enable system tick clock_time(): reg_system_tick_ctrl = FLD_SYSTEM_TICK_START;
	{ TCMD_SWR8, 0x74f, FLD_SYSTEM_TICK_START },
#if (SET_PLL == QUARTZ_16MHZ)	// 16 Mhz crystal
	// reg_pll_ctrl_a = 0x20
	{ TCMD_SWR8, 0x04eb, FLD_PLL_A_CAL_DONE_EN },
	{ TCMD_AWR, 0x099, 0x31 },
	{ TCMD_AWR, 0x082, 0x34 },
	{ TCMD_AWR, 0x09e, 0x82 },
#elif (SET_PLL == QUARTZ_12MHZ) // 12 MHz crystal
	// reg_pll_ctrl_a = 0xa0
	{ TCMD_SWR8, 0x04eb,  FLD_PLL_A_CAL_DONE_EN | 0x80},
	{ TCMD_AWR, 0x099, 0xb1 },
	{ TCMD_AWR, 0x082, 0x20 },
	{ TCMD_AWR, 0x09e, 0xad },
#else
	#error Set SET_PLL QUARTZ_16MHZ or QUARTZ_12MHZ!
#endif // SET_PLL
	// Timers & WDT
#if (USE_TIMER0) || (USE_TIMER1) || (USE_TIMER2) || (USE_WATCHDOG)
#if (USE_TIMER0)
	// Timer0: Clear count
	// reg_tmr0_tick = 0;
	{TCMD_SWR32, 0x630,	0},
	// Timer0: Set timeout USE_TIMER0 us
	// reg_tmr0_capt = USE_TIMER0 * CLOCK_SYS_CLOCK_1US;
	{TCMD_SWR32, 0x624,	USE_TIMER0 * CLOCK_SYS_CLOCK_1US},
#endif
#if (USE_TIMER1)
	// Timer1: Clear count
	// reg_tmr1_tick = 0;
	{TCMD_SWR32, 0x634,	0},
	// Timer1: Set timeout USE_TIMER1 us
	// reg_tmr1_capt = USE_TIMER1 * CLOCK_SYS_CLOCK_1US;
	{TCMD_SWR32, 0x628,	USE_TIMER1 * CLOCK_SYS_CLOCK_1US},
#endif
#if (USE_TIMER2)
	// Timer2: Clear count
	// reg_tmr2_tick = 0;
	{TCMD_SWR32, 0x638,	0},
	// Timer2: Set timeout USE_TIMER2 us
	// reg_tmr2_capt = USE_TIMER2 * CLOCK_SYS_CLOCK_1US;
	{TCMD_SWR32, 0x62c,	USE_TIMER0 * CLOCK_SYS_CLOCK_1US},
#endif
	// reg_tmr_ctrl
#if (USE_WATCHDOG)
#define WDT_COUNT (USE_WATCHDOG * CLOCK_SYS_CLOCK_1US)
#if (WDT_COUNT < (1<<WATCHDOG_TIMEOUT_COEFF))
#error Watchdog Capture low! Min (Step WDT) USE_WATCHDOG = 8192(us)
#endif
#if ((WDT_COUNT >> WATCHDOG_TIMEOUT_COEFF) > 0x3fff)
#error Watchdog Capture has only 14bits! Max USE_WATCHDOG = 134209536 (us) (16383*8192)
#endif
#if (USE_TIMER2)
#error Timer2 or Watchdog!
#else
	// Timer2: Clear count
	{TCMD_SWR32, 0x638,	0},
#endif
#endif
	{ TCMD_SWR32, 0x620, 0
#if (USE_WATCHDOG)
	// WDT set capture (FLD_TMR_WD_CAPT -> reg_tmr2_capt[31:18]), WDT Enable
	| ((WDT_COUNT >> 9) & FLD_TMR_WD_CAPT)
	| FLD_TMR_WD_EN
	| FLD_CLR_WD
	// Timer2: Mode 0, Enable
	| FLD_TMR2_STA // clear irq status
	| FLD_TMR2_EN
#endif
	// Timers: Mode 0, Enable
#if (USE_TIMER0)
		| FLD_TMR0_STA // clear irq status
		| FLD_TMR0_EN
#endif
#if (USE_TIMER1)
		| FLD_TMR1_STA // clear irq status
//		| FLD_TMR1_EN
#endif
#if (USE_TIMER2)
		| FLD_TMR2_STA // clear irq status
		| FLD_TMR2_EN
#endif
	},
	// Timers: Enable Interrupt
	// reg_irq_mask1 |= FLD_IRQ_TMR0_EN | ...
#if (USE_TIMER0) || (USE_TIMER1) || (USE_TIMER2)
	{TCMD_SOR8, 0x640, (0xfff<<16)
#if (USE_TIMER0)
			| FLD_IRQ_TMR0_EN
#endif
#if (USE_TIMER1)
			| FLD_IRQ_TMR1_EN
#endif
#if (USE_TIMER2)
			| FLD_IRQ_TMR2_EN
#endif
	},
#endif // (USE_TIMER0) || (USE_TIMER1) || (USE_TIMER2)
#endif // (USE_TIMER0) || (USE_TIMER1) || (USE_TIMER2) || (USE_WATCHDOG)
	// enable software irq [0x640]=0x01004000
//	{ TCMD_SOR8, 0x641, (0xFFFF<<16) | 0x40 }, // FLD_IRQ_SW_EN ? irq_software | irq_pwm
	// ???
//	{ TCMD_AWR,  0x0080, 0x71 },
//	{ TCMD_AWR,  0x0020, 0xc1 },
//	{ TCMD_AWR,  0x002d, 0x0f },

};

/*
 * Таблица инициализации всех GPIO
 */
static const TBLCMDSET32 tbl_gpio_ini[] = {
// ---------- gpio_init
	{TCMD_SWR32, 0x0580, // reg_gpio_pa_setting1
		(PA0_INPUT_ENABLE<<8) | (PA1_INPUT_ENABLE<<9)	| (PA2_INPUT_ENABLE<<10) | (PA3_INPUT_ENABLE<<11) |
		(PA4_INPUT_ENABLE<<12)	| (PA5_INPUT_ENABLE<<13) | (PA6_INPUT_ENABLE<<14) | (PA7_INPUT_ENABLE<<15) |
		((PA0_OUTPUT_ENABLE?0:1)<<16) | ((PA1_OUTPUT_ENABLE?0:1)<<17) | ((PA2_OUTPUT_ENABLE?0:1)<<18) | ((PA3_OUTPUT_ENABLE?0:1)<<19) |
		((PA4_OUTPUT_ENABLE?0:1)<<20) | ((PA5_OUTPUT_ENABLE?0:1)<<21) | ((PA6_OUTPUT_ENABLE?0:1)<<22) | ((PA7_OUTPUT_ENABLE?0:1)<<23) |
		(PA0_DATA_OUT<<24) | (PA1_DATA_OUT<<25) | (PA2_DATA_OUT<<26) | (PA3_DATA_OUT<<27) |
		(PA4_DATA_OUT<<28) | (PA5_DATA_OUT<<29) | (PA6_DATA_OUT<<30) | (PA7_DATA_OUT<<31)},
	{TCMD_SWR32, 0x584, // reg_gpio_pa_setting2
		(PA0_DATA_STRENGTH<<8) | (PA1_DATA_STRENGTH<<9) | (PA2_DATA_STRENGTH<<10) | (PA3_DATA_STRENGTH<<11) |
		(PA4_DATA_STRENGTH<<12)	| (PA5_DATA_STRENGTH<<13) | (PA6_DATA_STRENGTH<<14) | (PA7_DATA_STRENGTH<<15) |
		(PA0_FUNC==AS_GPIO ? BIT(16):0)	| (PA1_FUNC==AS_GPIO ? BIT(17):0)| (PA2_FUNC==AS_GPIO ? BIT(18):0)| (PA3_FUNC==AS_GPIO ? BIT(19):0) |
		(PA4_FUNC==AS_GPIO ? BIT(20):0)	| (PA5_FUNC==AS_GPIO ? BIT(21):0)| (PA6_FUNC==AS_GPIO ? BIT(22):0)| (PA7_FUNC==AS_GPIO ? BIT(23):0)},
	{TCMD_SWR32, 0x588, // reg_gpio_pb_setting1
		(PB0_INPUT_ENABLE<<8) 	| (PB1_INPUT_ENABLE<<9)	| (PB2_INPUT_ENABLE<<10)	| (PB3_INPUT_ENABLE<<11) |
		(PB4_INPUT_ENABLE<<12)	| (PB5_INPUT_ENABLE<<13)| (PB6_INPUT_ENABLE<<14)	| (PB7_INPUT_ENABLE<<15) |
		((PB0_OUTPUT_ENABLE?0:1)<<16)	| ((PB1_OUTPUT_ENABLE?0:1)<<17) | ((PB2_OUTPUT_ENABLE?0:1)<<18)	| ((PB3_OUTPUT_ENABLE?0:1)<<19) |
		((PB4_OUTPUT_ENABLE?0:1)<<20)	| ((PB5_OUTPUT_ENABLE?0:1)<<21) | ((PB6_OUTPUT_ENABLE?0:1)<<22)	| ((PB7_OUTPUT_ENABLE?0:1)<<23) |
		(PB0_DATA_OUT<<24)	| (PB1_DATA_OUT<<25)	| (PB2_DATA_OUT<<26)	| (PB3_DATA_OUT<<27) |
		(PB4_DATA_OUT<<28)	| (PB5_DATA_OUT<<29)	| (PB6_DATA_OUT<<30)	| (PB7_DATA_OUT<<31)},
	{TCMD_SWR32, 0x58c, // reg_gpio_pb_setting2
		(PB0_DATA_STRENGTH<<8)	| (PB1_DATA_STRENGTH<<9)	| (PB2_DATA_STRENGTH<<10)	| (PB3_DATA_STRENGTH<<11) |
		(PB4_DATA_STRENGTH<<12)	| (PB5_DATA_STRENGTH<<13)	| (PB6_DATA_STRENGTH<<14)	| (PB7_DATA_STRENGTH<<15) |
		(PB0_FUNC==AS_GPIO ? BIT(16):0)	| (PB1_FUNC==AS_GPIO ? BIT(17):0)| (PB2_FUNC==AS_GPIO ? BIT(18):0)| (PB3_FUNC==AS_GPIO ? BIT(19):0) |
		(PB4_FUNC==AS_GPIO ? BIT(20):0)	| (PB5_FUNC==AS_GPIO ? BIT(21):0)| (PB6_FUNC==AS_GPIO ? BIT(22):0)| (PB7_FUNC==AS_GPIO ? BIT(23):0)},
	{TCMD_SWR32, 0x590, // reg_gpio_pc_setting1
		(PC0_INPUT_ENABLE<<8) 	| (PC1_INPUT_ENABLE<<9)	| (PC2_INPUT_ENABLE<<10)	| (PC3_INPUT_ENABLE<<11) |
		(PC4_INPUT_ENABLE<<12)	| (PC5_INPUT_ENABLE<<13)| (PC6_INPUT_ENABLE<<14)	| (PC7_INPUT_ENABLE<<15) |
		((PC0_OUTPUT_ENABLE?0:1)<<16)	| ((PC1_OUTPUT_ENABLE?0:1)<<17) | ((PC2_OUTPUT_ENABLE?0:1)<<18)	| ((PC3_OUTPUT_ENABLE?0:1)<<19) |
		((PC4_OUTPUT_ENABLE?0:1)<<20)	| ((PC5_OUTPUT_ENABLE?0:1)<<21) | ((PC6_OUTPUT_ENABLE?0:1)<<22)	| ((PC7_OUTPUT_ENABLE?0:1)<<23) |
		(PC0_DATA_OUT<<24)	| (PC1_DATA_OUT<<25)	| (PC2_DATA_OUT<<26)	| (PC3_DATA_OUT<<27) |
		(PC4_DATA_OUT<<28)	| (PC5_DATA_OUT<<29)	| (PC6_DATA_OUT<<30)	| (PC7_DATA_OUT<<31) },
	{TCMD_SWR32, 0x594, // reg_gpio_pc_setting2
		(PC0_DATA_STRENGTH<<8)	| (PC1_DATA_STRENGTH<<9)	| (PC2_DATA_STRENGTH<<10)	| (PC3_DATA_STRENGTH<<11) |
		(PC4_DATA_STRENGTH<<12)	| (PC5_DATA_STRENGTH<<13)	| (PC6_DATA_STRENGTH<<14)	| (PC7_DATA_STRENGTH<<15) |
		(PC0_FUNC==AS_GPIO ? BIT(16):0)	| (PC1_FUNC==AS_GPIO ? BIT(17):0)| (PC2_FUNC==AS_GPIO ? BIT(18):0)| (PC3_FUNC==AS_GPIO ? BIT(19):0) |
		(PC4_FUNC==AS_GPIO ? BIT(20):0)	| (PC5_FUNC==AS_GPIO ? BIT(21):0)| (PC6_FUNC==AS_GPIO ? BIT(22):0)| (PC7_FUNC==AS_GPIO ? BIT(23):0)},
	{TCMD_SWR32, 0x0598, // reg_gpio_pd_setting1
		(PD0_INPUT_ENABLE<<8) 	| (PD1_INPUT_ENABLE<<9)	| (PD2_INPUT_ENABLE<<10)	| (PD3_INPUT_ENABLE<<11) |
		(PD4_INPUT_ENABLE<<12)	| (PD5_INPUT_ENABLE<<13)| (PD6_INPUT_ENABLE<<14)	| (PD7_INPUT_ENABLE<<15) |
		((PD0_OUTPUT_ENABLE?0:1)<<16)	| ((PD1_OUTPUT_ENABLE?0:1)<<17) | ((PD2_OUTPUT_ENABLE?0:1)<<18)	| ((PD3_OUTPUT_ENABLE?0:1)<<19) |
		((PD4_OUTPUT_ENABLE?0:1)<<20)	| ((PD5_OUTPUT_ENABLE?0:1)<<21) | ((PD6_OUTPUT_ENABLE?0:1)<<22)	| ((PD7_OUTPUT_ENABLE?0:1)<<23) |
		(PD0_DATA_OUT<<24)	| (PD1_DATA_OUT<<25)	| (PD2_DATA_OUT<<26)	| (PD3_DATA_OUT<<27) |
		(PD4_DATA_OUT<<28)	| (PD5_DATA_OUT<<29)	| (PD6_DATA_OUT<<30)	| (PD7_DATA_OUT<<31) },
	{TCMD_SWR32, 0x59c, // reg_gpio_pd_setting2
		(PD0_DATA_STRENGTH<<8)	| (PD1_DATA_STRENGTH<<9)	| (PD2_DATA_STRENGTH<<10)	| (PD3_DATA_STRENGTH<<11) |
		(PD4_DATA_STRENGTH<<12)	| (PD5_DATA_STRENGTH<<13)	| (PD6_DATA_STRENGTH<<14)	| (PD7_DATA_STRENGTH<<15) |
		(PD0_FUNC==AS_GPIO ? BIT(16):0)	| (PD1_FUNC==AS_GPIO ? BIT(17):0)| (PD2_FUNC==AS_GPIO ? BIT(18):0)| (PD3_FUNC==AS_GPIO ? BIT(19):0) |
		(PD4_FUNC==AS_GPIO ? BIT(20):0)	| (PD5_FUNC==AS_GPIO ? BIT(21):0)| (PD6_FUNC==AS_GPIO ? BIT(22):0)| (PD7_FUNC==AS_GPIO ? BIT(23):0)},
	{TCMD_SWR32, 0x05a0, // reg_gpio_pe_setting1
		(PE0_INPUT_ENABLE<<8) 	| (PE1_INPUT_ENABLE<<9)	| (PE2_INPUT_ENABLE<<10)	| (PE3_INPUT_ENABLE<<11) |
		(PE4_INPUT_ENABLE<<12)	| (PE5_INPUT_ENABLE<<13)| (PE6_INPUT_ENABLE<<14)	| (PE7_INPUT_ENABLE<<15) |
		((PE0_OUTPUT_ENABLE?0:1)<<16)	| ((PE1_OUTPUT_ENABLE?0:1)<<17) | ((PE2_OUTPUT_ENABLE?0:1)<<18)	| ((PE3_OUTPUT_ENABLE?0:1)<<19) |
		((PE4_OUTPUT_ENABLE?0:1)<<20)	| ((PE5_OUTPUT_ENABLE?0:1)<<21) | ((PE6_OUTPUT_ENABLE?0:1)<<22)	| ((PE7_OUTPUT_ENABLE?0:1)<<23) |
		(PE0_DATA_OUT<<24)	| (PE1_DATA_OUT<<25)	| (PE2_DATA_OUT<<26)	| (PE3_DATA_OUT<<27) |
		(PE4_DATA_OUT<<28)	| (PE5_DATA_OUT<<29)	| (PE6_DATA_OUT<<30)	| (PE7_DATA_OUT<<31) },
	{TCMD_SWR32, 0x05a4, // reg_gpio_pe_setting2
		(PE0_DATA_STRENGTH<<8)	| (PE1_DATA_STRENGTH<<9)	| (PE2_DATA_STRENGTH<<10)	| (PE3_DATA_STRENGTH<<11) |
		(PE4_DATA_STRENGTH<<12)	| (PE5_DATA_STRENGTH<<13)	| (PE6_DATA_STRENGTH<<14)	| (PE7_DATA_STRENGTH<<15) |
		(PE0_FUNC==AS_GPIO ? BIT(16):0)	| (PE1_FUNC==AS_GPIO ? BIT(17):0)| (PE2_FUNC==AS_GPIO ? BIT(18):0)| (PE3_FUNC==AS_GPIO ? BIT(19):0) |
		(PE4_FUNC==AS_GPIO ? BIT(20):0)	| (PE5_FUNC==AS_GPIO ? BIT(21):0)| (PE6_FUNC==AS_GPIO ? BIT(22):0)| (PE7_FUNC==AS_GPIO ? BIT(23):0) },
#if (CHIP_TYPE==CHIP_TYPE_8266)
		{TCMD_SWR32, 0x05a8, // reg_gpio_pe_setting1
		(PF0_INPUT_ENABLE<<8) 	| (PF1_INPUT_ENABLE<<9)	 | ((PF0_OUTPUT_ENABLE?0:1)<<16) | ((PF1_OUTPUT_ENABLE?0:1)<<17) |
		(PF0_DATA_OUT<<24)	| (PF1_DATA_OUT<<25) },
		{TCMD_SWR32, 0x05ac, // reg_gpio_pe_setting2
		(PF0_DATA_STRENGTH<<8)	| (PF1_DATA_STRENGTH<<9) | (PF0_FUNC==AS_GPIO ? BIT(16):0)	| (PF1_FUNC==AS_GPIO ? BIT(17):0) },
#endif
// -------------- PULL WAKEUP
	{TCMD_AOR, 0x0a,
		(0x0F<<16) |
		(PULL_WAKEUP_SRC_PA0<<4) | (PULL_WAKEUP_SRC_PA1<<6)	},
	{TCMD_AWR, 0x0b,
		PULL_WAKEUP_SRC_PA2 | (PULL_WAKEUP_SRC_PA3<<2) |
		(PULL_WAKEUP_SRC_PA4<<4) | (PULL_WAKEUP_SRC_PA5<<6) },
	{TCMD_AWR, 0x0c,
		PULL_WAKEUP_SRC_PA6 | (PULL_WAKEUP_SRC_PA7<<2) |
		(PULL_WAKEUP_SRC_PB0<<4) | (PULL_WAKEUP_SRC_PB1<<6) },
	{TCMD_AWR, 0x0d,
		PULL_WAKEUP_SRC_PB2 | (PULL_WAKEUP_SRC_PB3<<2) |
		(PULL_WAKEUP_SRC_PB4<<4) |	(PULL_WAKEUP_SRC_PB5<<6) },
	{TCMD_AWR, 0x0e,
		PULL_WAKEUP_SRC_PB6 | (PULL_WAKEUP_SRC_PB7<<2) |
		(PULL_WAKEUP_SRC_PC0<<4) | (PULL_WAKEUP_SRC_PC1<<6) },
	{TCMD_AWR, 0x0f,
		PULL_WAKEUP_SRC_PC2 | (PULL_WAKEUP_SRC_PC3<<2) |
		(PULL_WAKEUP_SRC_PC4<<4) | (PULL_WAKEUP_SRC_PC5<<6) },
	{TCMD_AWR, 0x10,
		PULL_WAKEUP_SRC_PC6 | (PULL_WAKEUP_SRC_PC7<<2) |
		(PULL_WAKEUP_SRC_PD0<<4) | (PULL_WAKEUP_SRC_PD1<<6) },
	{TCMD_AWR, 0x11,
		PULL_WAKEUP_SRC_PD2 | (PULL_WAKEUP_SRC_PD3<<2) |
		(PULL_WAKEUP_SRC_PD4<<4) | (PULL_WAKEUP_SRC_PD5<<6) },
	{TCMD_AWR, 0x12,  PULL_WAKEUP_SRC_PD6 |
		(PULL_WAKEUP_SRC_PD7<<2) | (PULL_WAKEUP_SRC_PE0<<4) |
		(PULL_WAKEUP_SRC_PE1<<6) },
#if (CHIP_TYPE==CHIP_TYPE_8269)

	{TCMD_AOR, 0x08, (0x0f << 16) |
		PULL_WAKEUP_SRC_PE2<<4 | (PULL_WAKEUP_SRC_PE3<<6) },
	// ---------- Pad Function Mux
	{TCMD_SWR32, 0x5b0, 0
		|	((PA0_FUNC==AS_DMIC) << 0)
//		|	((PA1_FUNC==AS_?) << ?)
		|	((PA2_FUNC==AS_SPI) << 1)
		|	((PA3_FUNC==AS_SPI) << 2)
		|	((PA4_FUNC==AS_SPI) << 4)
		|	((PA5_FUNC==AS_SPI) << 5)
//		|	((PA6_FUNC==AS_?) << 6)
		|	((PA7_FUNC==AS_UART) << 7)
		|	((PB0_FUNC==AS_PWM) << 8)
//		|	((PB1_FUNC==AS_?) << 9)
		|	((PB2_FUNC==AS_UART) << 10)
		|	((PB3_FUNC==AS_UART) << 11)
		|	((PB4_FUNC==AS_SPI) << 12)
		|	((PB5_FUNC==AS_SPI) << 13)
		|	((PB6_FUNC==AS_SPI) << 14)
		|	((PB7_FUNC==AS_SPI) << 15)

		|	((PC0_FUNC==AS_I2C) << 16)
		|	((PC1_FUNC==AS_I2C) << 17)
		|	((PC2_FUNC==AS_UART) << 18)
		|	((PC3_FUNC==AS_UART) << 19)
		|	((PC4_FUNC==AS_UART) << 20)
		|	((PC5_FUNC==AS_UART) << 21)
//		|	((PC6_FUNC==AS_?) << 22)
//		|	((PC7_FUNC==AS_?) << 23)
//		|	((PD0_FUNC==AS_?) << 24)
//		|	((PD1_FUNC==AS_?) << 25)
//		|	((PD2_FUNC==AS_?) << 26)
//		|	((PD3_FUNC==AS_?) << 27)
//		|	((PD4_FUNC==AS_?) << 28)
//		|	((PD5_FUNC==AS_?) << 29)
//		|	((PD6_FUNC==AS_?) << 30)
//		|	((PD7_FUNC==AS_?) << 31)
	},
	{TCMD_SWR8, 0x5b4, 0
		|	((PE0_FUNC==AS_PWM) << 0)
		|	((PE1_FUNC==AS_PWM) << 1)
//		|	((PE2_FUNC==AS_?) << 2)
//		|	((PE3_FUNC==AS_?) << 3)
	},
#elif (CHIP_TYPE==CHIP_TYPE_8266)
	{TCMD_AWR, 0x13,  PULL_WAKEUP_SRC_PE2 |
			(PULL_WAKEUP_SRC_PE3<<2) |
			(PULL_WAKEUP_SRC_PE4<<4) |
			(PULL_WAKEUP_SRC_PE5<<6) },
	{TCMD_AWR, 0x14,  PULL_WAKEUP_SRC_PE6 |
			(PULL_WAKEUP_SRC_PE7<<2) |
			(PULL_WAKEUP_SRC_PF0<<4) |
			(PULL_WAKEUP_SRC_PF1<<6) },
	// ---------- Pad Function Mux
	{TCMD_SWR8, 0x5b4, (PE6_FUNC==AS_UART || PF0_FUNC==AS_UART)  << 5 },
#endif
};

/*
 * Стартовая инициализация:
 *  1. SoC регистров
 *  2. GPIO
 */
void StartUpInits(void) {
//	LoadTbl32Set (tbl_cpu_wakeup_init1, sizeof (tbl_cpu_wakeup_init1)/sizeof (TBLCMDSET32));
	LoadTbl32Set (tbl_soc_ini, sizeof (tbl_soc_ini)/sizeof (TBLCMDSET32));
	LoadTbl32Set (tbl_gpio_ini, sizeof (tbl_gpio_ini)/sizeof (TBLCMDSET32));
}
