/*
 * inits.h
 *
 *  Created on: 14.10.2019
 *      Author: pvvx
 */

#ifndef _INITS_H_
#define _INITS_H_

#pragma once

typedef struct TBLCMDSET32 {
	unsigned short	cmd;
	unsigned short	adr;
	unsigned int	dat;
} TBLCMDSET32;

// TBLCMDSET32 cmd :
enum{
	TCMD_SWR32 = 1,
	TCMD_SWR16,
	TCMD_SWR8,
	TCMD_SSET32,
	TCMD_SOR16,
	TCMD_SOR8,
	TCMD_AWR,
	TCMD_AOR
};

void LoadTbl32Set(const TBLCMDSET32 * pt,int size);
/*
 * Стартовая инициализация:
 *  1. SoC регистров
 *  2. GPIO
 */
void StartUpInits(void);

#endif /* _INITS_H_ */
