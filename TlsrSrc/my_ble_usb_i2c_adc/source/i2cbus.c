/*
 * ina2xx.c
 *
 *	Created on: 07.01.2020
 *		Author: pvvx
 */
#include "proj/tl_common.h"
#include "i2cbus.h"

#define USE_I2C_BUS_BUSY	0
/* Universal I2C/SMBUS read-write transaction
 * wrlen = 1..127 ! */
int I2CBusUtr(void * outdata, i2c_utr_t *tr, unsigned int wrlen) {
	unsigned char * pwrdata = (unsigned char *) &tr->wrdata;
	unsigned char * poutdata = (unsigned char *) outdata;
	unsigned int cntstart = wrlen - (tr->mode & 0x7f);
	unsigned int rdlen = tr->rdlen & 0x7f;

	unsigned char r = irq_disable();
	reg_i2c_id = *pwrdata++;
	reg_i2c_ctrl = FLD_I2C_CMD_START | FLD_I2C_CMD_ID;
	while(reg_i2c_status & FLD_I2C_CMD_BUSY);
#if USE_I2C_BUS_BUSY
	int ret = (reg_i2c_status & (FLD_I2C_NAK | FLD_I2C_BUS_BUSY)) == 0;
#else
	int ret = (reg_i2c_status & FLD_I2C_NAK) == 0;
#endif
	while(ret && wrlen--) {
		// write data
		reg_i2c_do = *pwrdata++;
		reg_i2c_ctrl = FLD_I2C_CMD_DO;
		while(reg_i2c_status & FLD_I2C_CMD_BUSY);
		ret = (reg_i2c_status & FLD_I2C_NAK) == 0;
		if(wrlen == cntstart && ret) { // + send start & id
			if(rdlen)
				reg_i2c_id |=  FLD_I2C_WRITE_READ_BIT;
			if(tr->mode & 0x80) {
				reg_i2c_ctrl = FLD_I2C_CMD_STOP;
				while(reg_i2c_status & FLD_I2C_CMD_BUSY);
			}
			else {
#if 1	// optimization
				reg_rst0 = FLD_RST_I2C;
				reg_rst0 = 0;
#else
				unsigned char tmp = reg_rst0 &(~FLD_RST_I2C);
				reg_rst0 = FLD_RST_I2C;
				reg_rst0 = tmp;
#endif
			}
			// start + id
			reg_i2c_ctrl = FLD_I2C_CMD_START | FLD_I2C_CMD_ID;
			// hw reset I2C
			while(reg_i2c_status & FLD_I2C_CMD_BUSY);
			ret = (reg_i2c_status & FLD_I2C_NAK) == 0;
		}
	}
	while(ret && rdlen--) {
		if(rdlen)
			reg_i2c_ctrl = FLD_I2C_CMD_DI | FLD_I2C_CMD_READ_ID;
		else {
			if(tr->rdlen & 0x80)
				reg_i2c_ctrl = FLD_I2C_CMD_DI | FLD_I2C_CMD_READ_ID | FLD_I2C_CMD_NAK;
			else
				reg_i2c_ctrl = FLD_I2C_CMD_DI | FLD_I2C_CMD_READ_ID;
		}
		while(reg_i2c_status & FLD_I2C_CMD_BUSY);
#if USE_I2C_BUS_BUSY
		ret = (reg_i2c_status & FLD_I2C_BUS_BUSY) == 0;
#endif
		*poutdata++ = reg_i2c_di;
	}
	reg_i2c_ctrl = FLD_I2C_CMD_STOP; // launch start/stop cycle
	while(reg_i2c_status & FLD_I2C_CMD_BUSY);
	irq_restore(r);
	return ret;
}

#if 0
/*
 *  rdlen: 0 -> start-addr-stop, bit7 =1 -> end byte send ACK
 *  wrlen:  addr len - 1
 */
int I2CBusRead(unsigned char i2c_addr, void *data, int rdlen, int wrlen)
{
	int i = rdlen & 0x7f, ret = 0;
	unsigned char * p = (unsigned char *) data;
	u8 r = irq_disable();
	if(i) {
		u8 tmp = reg_rst0 &(~FLD_RST_I2C);
		reg_rst0 = tmp | FLD_RST_I2C;
		reg_rst0 = tmp;
		// Start By Master Read, Write Slave Address, Read data MSByte
		reg_i2c_id = i2c_addr | FLD_I2C_WRITE_READ_BIT;  //SlaveID & 0xfe,.i.e write data. Read:High  Write:Low
		//write addr
		reg_i2c_adr = p[0];
		if(wrlen == 0) {
			reg_i2c_ctrl = FLD_I2C_CMD_START | FLD_I2C_CMD_ID;
		}
		else if(wrlen == 1) {
			reg_i2c_ctrl = FLD_I2C_CMD_START | FLD_I2C_CMD_ID | FLD_I2C_CMD_ADR;
		} else {
			//write data
			reg_i2c_do = p[1];
			reg_i2c_ctrl = FLD_I2C_CMD_START | FLD_I2C_CMD_ID | FLD_I2C_CMD_ADR | FLD_I2C_CMD_DO;
		}
		while(reg_i2c_status & FLD_I2C_CMD_BUSY);
		if((reg_i2c_status & FLD_I2C_NAK) == 0) {
			while(i--) {
				if(i) reg_i2c_ctrl = FLD_I2C_CMD_DI | FLD_I2C_CMD_READ_ID;
				else {
					if(rdlen & 0x80)
						reg_i2c_ctrl = FLD_I2C_CMD_DI | FLD_I2C_CMD_READ_ID | FLD_I2C_CMD_STOP;
					else
						reg_i2c_ctrl = FLD_I2C_CMD_DI | FLD_I2C_CMD_READ_ID | FLD_I2C_CMD_STOP | FLD_I2C_CMD_NAK;
				}
				while(reg_i2c_status & FLD_I2C_CMD_BUSY);
				*p++ = reg_i2c_di;
			}
			ret = 1;
		} else {
			reg_i2c_ctrl = FLD_I2C_CMD_STOP; // launch stop cycle
			while(reg_i2c_status & FLD_I2C_CMD_BUSY	);
		}
	} else {
		reg_i2c_ctrl = FLD_I2C_CMD_START | FLD_I2C_CMD_STOP; // launch start/stop cycle
		while(reg_i2c_status & FLD_I2C_CMD_BUSY	);
	}
	irq_restore(r);
	return ret;
}
/*
 * stop: =1 -> send STOP, = 0 -> restart I2C
 */
int I2CBusWrite(unsigned char i2c_addr, void *data, int wrlen , unsigned char stop)
{
	int i = wrlen, ret = 0;
	if (stop) stop = FLD_I2C_CMD_ADR | FLD_I2C_CMD_STOP;
	unsigned char * p = (unsigned char *) data;
	u8 r = irq_disable();
//		reg_i2c_id &= (~FLD_I2C_WRITE_READ_BIT); // SlaveID & 0xfe,.i.e write data. R:High  W:Low
	reg_i2c_id = i2c_addr;

	// write data
	reg_i2c_adr = *p++;
	reg_i2c_do = *p++;
	reg_i2c_di = *p++;
	if(!i)
		reg_i2c_ctrl = stop | FLD_I2C_CMD_START | FLD_I2C_CMD_ID;
	else if(!--i)
		reg_i2c_ctrl = stop | FLD_I2C_CMD_START | FLD_I2C_CMD_ID | FLD_I2C_CMD_ADR;
	else if(!--i)
		reg_i2c_ctrl = stop | FLD_I2C_CMD_START | FLD_I2C_CMD_ID | FLD_I2C_CMD_ADR | FLD_I2C_CMD_DO;
	else if(!--i)
		reg_i2c_ctrl = stop | FLD_I2C_CMD_START | FLD_I2C_CMD_ID | FLD_I2C_CMD_ADR | FLD_I2C_CMD_DO | FLD_I2C_CMD_DI;
	else // i > 3
		reg_i2c_ctrl = FLD_I2C_CMD_START | FLD_I2C_CMD_ID | FLD_I2C_CMD_ADR | FLD_I2C_CMD_DO | FLD_I2C_CMD_DI;
	while(reg_i2c_status & FLD_I2C_CMD_BUSY);
	ret = (reg_i2c_status & FLD_I2C_NAK) == 0;
	while(ret && i) {
		//write data
		reg_i2c_adr = *p++;
		reg_i2c_do = *p++;
		reg_i2c_di = *p++;
		if(!--i) {
			reg_i2c_ctrl = stop | FLD_I2C_CMD_ADR;
			break;
		} else if(!--i) {
			reg_i2c_ctrl = stop | FLD_I2C_CMD_ADR;
			break;
		} else if(!--i) {
			reg_i2c_ctrl = stop | FLD_I2C_CMD_ADR;
			break;
		} else // i > 3
			reg_i2c_ctrl = FLD_I2C_CMD_ADR;
		while(reg_i2c_status & FLD_I2C_CMD_BUSY);
		ret = (reg_i2c_status & FLD_I2C_NAK) == 0;
	}
	if(!ret) {
		reg_i2c_ctrl = FLD_I2C_CMD_STOP; // launch stop cycle
		while(reg_i2c_status & FLD_I2C_CMD_BUSY	);
	}
	irq_restore(r);
	return ret;
}
#endif

int I2CBusWriteWord(unsigned char i2c_addr, unsigned char reg_addr, unsigned short reg_data)
{
	int ret = 0;
	u8 r = irq_disable();
	//Start By	Master, write Slave Address & Register Pointer
	reg_i2c_id = i2c_addr; // &= (~FLD_I2C_WRITE_READ_BIT); // SlaveID & 0xfe,.i.e write data. R:High  W:Low
   	reg_i2c_adr = reg_addr;
	//write data MSByte
	reg_i2c_do = (unsigned char)(reg_data >> 8);
	//write data LSByte
	reg_i2c_di = (unsigned char)(reg_data);
   	reg_i2c_ctrl = FLD_I2C_CMD_START | FLD_I2C_CMD_ID | FLD_I2C_CMD_ADR | FLD_I2C_CMD_DO | FLD_I2C_CMD_DI | FLD_I2C_CMD_STOP; // 0x3f
    while(reg_i2c_status & FLD_I2C_CMD_BUSY	);
	ret = (reg_i2c_status & FLD_I2C_NAK) == 0;
	irq_restore(r);
	return ret;
}

_attribute_ram_code_ int I2CBusReadWord(unsigned char i2c_addr, unsigned char reg_addr, void *preg_data)
{
	int ret = 0;

	unsigned char * p = (unsigned char *) preg_data;
//	u8 r = irq_disable();
	//Start By	Master Write, write Slave Address & Register Pointer
	reg_i2c_id = i2c_addr; // &= (~FLD_I2C_WRITE_READ_BIT); // SlaveID & 0xfe,.i.e write data. R:High  W:Low
   	reg_i2c_adr = reg_addr; //address
   	reg_i2c_ctrl = FLD_I2C_CMD_START | FLD_I2C_CMD_ID | FLD_I2C_CMD_ADR;// | FLD_I2C_CMD_STOP; // 0x33
    while(reg_i2c_status & FLD_I2C_CMD_BUSY	);
	if ((reg_i2c_status & FLD_I2C_NAK) == 0) {
		// Start By Master Read, Write Slave Address, Read data MSByte
		reg_i2c_id |= FLD_I2C_WRITE_READ_BIT;  //SlaveID & 0xfe,.i.e write data. Read:High  Write:Low

#if 1	// optimization
		reg_rst0 = FLD_RST_I2C;
		reg_rst0 = 0;
#else
		unsigned char tmp = reg_rst0 &(~FLD_RST_I2C);
		reg_rst0 = FLD_RST_I2C;
		reg_rst0 = tmp;
#endif

		reg_i2c_ctrl = FLD_I2C_CMD_START | FLD_I2C_CMD_ID | FLD_I2C_CMD_READ_ID | FLD_I2C_CMD_DI; // 0x59
	    while(reg_i2c_status & FLD_I2C_CMD_BUSY);
		p[1] = reg_i2c_di;
		// Read data LSByte, Stop By Master
	    reg_i2c_ctrl = FLD_I2C_CMD_READ_ID | FLD_I2C_CMD_DI | FLD_I2C_CMD_NAK | FLD_I2C_CMD_STOP; // 0xE8
	    while(reg_i2c_status & FLD_I2C_CMD_BUSY);
		p[0] = reg_i2c_di;
		ret = 1;
	}
//	irq_restore(r);
	return ret;
}

void I2CBusDeInit(void) {
#if (MCU_CORE_TYPE == MCU_CORE_8266)
	// SDA
	BM_SET(reg_gpio_oen(GPIO_PE7), GPIO_PE7 & 0xff); // disable output
	BM_SET(reg_gpio_gpio_func(GPIO_PE7), GPIO_PE7 & 0xff); // is gpio
	BM_SET(reg_gpio_config_func(GPIO_PE7), GPIO_PE7 & 0xff); // is gpio
//	BM_CLR(reg_gpio_ie(GPIO_PE7), GPIO_PE7 & 0xff);	// disable input
	// SCL
	BM_SET(reg_gpio_oen(GPIO_PF1), GPIO_PF1 & 0xff); // disable output
	BM_SET(reg_gpio_gpio_func(GPIO_PF1), GPIO_PF1 & 0xff); // is gpio
//	BM_CLR(reg_gpio_ie(GPIO_PF1), GPIO_PF1 & 0xff);	// disable input
	// up_down_resistor
	analog_write(0x14, (analog_read(0x14) & (~((3 << 2) | (3 <<6))))
				|  (GPIO_PULL_UP_0 << 2) // PE7
				|  (GPIO_PULL_UP_0 << 6) // PF1
				);
#else
/*
	u32 gpio_sda = GPIO_PA3;
	u32 gpio_scl = GPIO_PA4;
	gpio_setup_up_down_resistor(gpio_sda, PM_PIN_PULLUP_1M);       // 10k pull_up resistor
	gpio_setup_up_down_resistor(gpio_scl, PM_PIN_PULLUP_1M);       // 10k pull_up resistor
*/
#endif
	BM_CLR(reg_rst_clk0, FLD_CLK_I2C_EN);
}

void I2CBusInit(unsigned int clk_khz) {
//	int ret = 0;
#if (MCU_CORE_TYPE == MCU_CORE_8266)
#if 0
	u32 gpio_sda = GPIO_PE7;
	u32 gpio_scl = GPIO_PF1;

	gpio_set_func(gpio_sda, AS_I2C);  //disable gpio function
	gpio_set_func(gpio_scl, AS_I2C);  //disable gpio function

	gpio_setup_up_down_resistor(gpio_sda, PM_PIN_PULLUP_10K);
	gpio_setup_up_down_resistor(gpio_scl, PM_PIN_PULLUP_10K);

	gpio_set_input_en(gpio_sda, 1);
    gpio_set_input_en(gpio_scl, 1);
#else
    // SCL
	BM_CLR(reg_gpio_gpio_func(GPIO_PF1), GPIO_PF1 & 0xff);
	BM_SET(reg_gpio_ie(GPIO_PF1), BIT(1));  //enable input
	// SDA
	BM_CLR(reg_gpio_gpio_func(GPIO_PE7), GPIO_PE7 & 0xff);
    BM_CLR(reg_gpio_config_func(GPIO_PE7), GPIO_PE7 & 0xff);
	BM_SET(reg_gpio_ie(GPIO_PE7), BIT(7));  //enable input
	BM_CLR(reg_gpio_oen(GPIO_PE7), BIT(7)); //enable output
	// up_down_resistor
	analog_write(0x14, (analog_read(0x14) & (~((3 << 2) | (3 <<6))))
				|  (GPIO_PULL_UP_10K << 2) // PE7
				|  (GPIO_PULL_UP_10K << 6) // PF1
				);
#endif

#else
	u32 gpio_sda = GPIO_PA3;
	u32 gpio_scl = GPIO_PA4;
	gpio_setup_up_down_resistor(gpio_sda, PM_PIN_PULLUP_10K);       // 10k pull_up resistor
	gpio_setup_up_down_resistor(gpio_scl, PM_PIN_PULLUP_10K);       // 10k pull_up resistor

	gpio_set_func(gpio_sda, AS_I2C);  // disable gpio function
	gpio_set_func(gpio_scl, AS_I2C);  // disable gpio function
#endif
	// FI2C = (System Clock/(address 0x73[7:4]+1)) / (4 *clock speed configured in address 0x00)
	// System Clock / (8 * address 0x00)
	// default reg_i2c_speed = 0x13
	reg_i2c_speed = (CLOCK_SYS_CLOCK_HZ/4000)/clk_khz; //=5 if set i2c clock 400 kHz

//	reg_i2c_id = cfg->i2c_addr;	//set the id of i2c module.

	BM_SET(reg_i2c_mode, FLD_I2C_MODE_MASTER);  // enable master mode.
	BM_SET(reg_rst_clk0, FLD_CLK_I2C_EN);       // enable i2c clock
	BM_CLR(reg_spi_sp, FLD_SPI_ENABLE);        // force PADs act as I2C; i2c and spi share the hardware of IC

	//Start/Stop By Master, SDA & SCL = "1"
    reg_i2c_ctrl = FLD_I2C_CMD_START | FLD_I2C_CMD_STOP; // launch stop cycle
    while(reg_i2c_status & FLD_I2C_CMD_BUSY);
//	return ret;
}
