
#pragma once

/////////////////////////////////////////////////////
//  compatibility for old api, 	 no so good but keep them for the moment

#define WriteAnalogReg(addr,v)	analog_write(addr,v)
#define ReadAnalogReg(addr)		analog_read(addr)

extern void sleep_us (unsigned int us);

#define WaitUs(t)			sleep_us(t)
#define WaitMs(t)			sleep_us((t)*1000)

#ifdef WIN32
#define write_reg8(addr,v)	U8_SET((addr - 0x800000 + REG_BASE_ADDR),v)
#define write_reg16(addr,v)	U16_SET((addr - 0x800000 + REG_BASE_ADDR),v)
#define write_reg32(addr,v)	U32_SET((addr - 0x800000 + REG_BASE_ADDR),v)
#define read_reg8(addr)		U8_GET((addr - 0x800000 + REG_BASE_ADDR))
#define read_reg16(addr)	U16_GET((addr - 0x800000 + REG_BASE_ADDR))
#define read_reg32(addr)	U32_GET((addr - 0x800000 + REG_BASE_ADDR))
#else
#define write_reg8(addr,v)	U8_SET((addr + REG_BASE_ADDR),v)
#define write_reg16(addr,v)	U16_SET((addr + REG_BASE_ADDR),v)
#define write_reg32(addr,v)	U32_SET((addr + REG_BASE_ADDR),v)
#define read_reg8(addr)		U8_GET((addr + REG_BASE_ADDR))
#define read_reg16(addr)	U16_GET((addr + REG_BASE_ADDR))
#define read_reg32(addr)	U32_GET((addr + REG_BASE_ADDR))
#define set_reg8_bit(addr,v)    write_reg8(addr, read_reg8(addr)|(v));
#define reset_reg8_bit(addr,v)  write_reg8(addr, read_reg8(addr)&(~(v)));

#endif

#define TCMD_UNDER_RD		0x80
#define TCMD_UNDER_WR		0x40
#define TCMD_UNDER_BOTH		0xc0
#define TCMD_MASK			0x3f

#define TCMD_WRITE			0x3
#define TCMD_WAIT			0x7
#define TCMD_WAREG			0x8

typedef struct TBLCMDSET {
	unsigned short	adr;
	unsigned char	dat;
	unsigned char	cmd;
} TBLCMDSET;

int LoadTblCmdSet (	const TBLCMDSET * pt, int size);



