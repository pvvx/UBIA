/*
 * INA219 util - part of PowerCape by AndiceLabs
 *
 * Copyright (C) 2014  AndiceLabs admin@andicelabs.com  http://andicelabs.com
 * Copyright (C) 2014  Zig Fisher flyrouter@gmail.com   http://zftlab.org
 * 2020 pvvx for USB adapter UBIA (TLSR8266)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *
 * Example run and output:
 *
 * OpenWRT:~# uina219 -b 0 -i 100
 *
 *       12168mV  134.02mA
 *       12168mV  239.92mA
 *       12168mV  134.73mA
 *
 */

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <endian.h>
#include <string.h>
#include <signal.h>
#include <time.h>
#include <getopt.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <termios.h>
#include "ubia_blk.h"

#define CONFIG_REG          0
#define SHUNT_REG           1
#define BUS_REG             2
#define POWER_REG           3
#define CURRENT_REG         4
#define CALIBRATION_REG     5

#define INA_ADDRESS         0x80

#define SET_TTY_OPT	1

#if 0
#include <netinet/in.h>
#define swap_ushort(a) htos(a)
#else
#define swap_ushort(a) htole16(a)
#endif

typedef enum {
	OP_DUMP, OP_VOLTAGE, OP_CURRENT, OP_MONITOR, OP_NONE
} op_type;

op_type operation = OP_DUMP;

#define CONV_STEP 	(586+568) //  typ 532 us, max 586 us
#define MIN_INTERVAL 4 // 2? ms
#define DEF_INTERVAL 1000 // 1 s

blk_tx_pkt_t read_pkt;
blk_rx_pkt_t send_pkt;

int samplescount = 100;
int interval = DEF_INTERVAL; // in ms
int i2c_bus = 0;
int i2c_address = INA_ADDRESS;
int i2c_clk_khz = 1200;
int handle;
int whole_numbers = 0;
unsigned short config = 0x399F;
int setcfg = 0;

dev_get_ver_t dev_ver;

void msleep(int msecs) {
	usleep(msecs * 1000);
}

int ubia_read_blk(uint8_t cmd) {
	int cnt_err = 7;
	do {
		read_pkt.head.size = 0;
		read_pkt.head.cmd = 0xff;
		int rc = read(handle, &read_pkt, sizeof(read_pkt));

		if (rc >= sizeof(read_pkt.head) + read_pkt.head.size
				&& (read_pkt.head.cmd & 0x7f) == cmd) {
			return (read_pkt.head.cmd & 0x80) != 0;
		} else if (rc < 0) {
			if (errno == EAGAIN || errno == EWOULDBLOCK) {
				// Resource temporarily unavailable
				usleep(500);
				continue;
			} else
				break;
		} else {
			usleep(1000);
		}
	} while (cnt_err--);
	printf("UBIA read failed: %s\n", strerror( errno));
	return -1;
}

int ubia_write_blk(void) {
	int rc = 0;

	if (write(handle, &send_pkt, send_pkt.head.size + sizeof(send_pkt.head))
			!= send_pkt.head.size + sizeof(send_pkt.head)) {
		rc = -1;
	}

	return rc;
}

int ubia_write_cmd(uint8_t cmd, uint8_t size) {
	send_pkt.head.cmd = cmd;
	send_pkt.head.size = size;
	int rc = ubia_write_blk();
	if (rc == 0) {
		msleep(1);
		rc = ubia_read_blk(cmd);
	}
	return rc;
}

int ubia_get_version(dev_get_ver_t * p) {
	int rc = -1;
	if (ubia_write_cmd(CMD_DEV_VER, 0) == 0) {
		memcpy(p, &read_pkt.data.ver, sizeof(dev_get_ver_t));
		rc = 0;
	}
	return rc;
}

int ubia_i2c_clk(int clk_khz) {
	send_pkt.data.ci2c.pktcnt = 0;
	send_pkt.data.ci2c.multiplier = 0;
	send_pkt.data.ci2c.time = 1000;
	send_pkt.data.ci2c.clk_khz = clk_khz;
	send_pkt.data.ci2c.init[0].dev_addr = 0;
	return ubia_write_cmd(CMD_DEV_CFG, 7);
}

int register_read(unsigned char reg, unsigned short *data) {
	send_pkt.data.rdreg.dev_addr = (uint8_t) i2c_address;
	send_pkt.data.rdreg.reg_addr = reg;
	int rc = ubia_write_cmd(CMD_DEV_GRG, sizeof(reg_rd_t));
	if (rc == 0)
		*data = swap_ushort(read_pkt.data.wrreg.data);
	return rc;
}

int register_write(unsigned char reg, unsigned short data) {
	send_pkt.data.wrreg.dev_addr = (uint8_t) i2c_address;
	send_pkt.data.wrreg.reg_addr = reg;
	send_pkt.data.wrreg.data = swap_ushort(data);
	return ubia_write_cmd(CMD_DEV_SRG, sizeof(reg_wr_t));
}

void show_usage(char *progname) {
	fprintf( stderr, "Usage: %s <mode> \n", progname);
	fprintf( stderr, "   Mode (required):\n");
	fprintf( stderr, "      -h --help           Show usage.\n");
	fprintf( stderr,
			"      -i --interval       Set interval (ms) for monitor mode.\n");
	fprintf( stderr,
			"      -w --whole          Show whole numbers only. Useful for scripts.\n");
	fprintf( stderr, "      -v --voltage        Show voltage in mV.\n");
	fprintf( stderr, "      -c --current        Show current in mA.\n");
	fprintf( stderr,
			"      -a --address <addr> Override I2C address of INA219 from default of 0x%02X.\n",
			i2c_address);
	fprintf( stderr,
			"      -b --bus <ttyACMx>  Override ttyACM device from default of %d.\n",
			i2c_bus);
	fprintf( stderr, "      -s --setcfg <value> Set Configuration Register (default: 0x%04X)\n", config);
	fprintf( stderr, "      -l --length <value> Set count read (default: %u)\n", samplescount);
	exit(1);
}

void parse(int argc, char *argv[]) {
	while (1) {
		static const struct option lopts[] = { { "address", 0, 0, 'a' }, {
				"bus", 0, 0, 'b' }, { "current", 0, 0, 'c' }, { "help", 0, 0,
				'h' }, { "interval", 0, 0, 'i' }, { "voltage", 0, 0, 'v' }, {
				"whole", 0, 0, 'w' }, { "setcfg", 0, 0, 's' }, { "length", 0, 0,
				'l' }, { NULL, 0, 0, 0 }, };
		int c;

		c = getopt_long(argc, argv, "a:b:chi:vws:l:", lopts, NULL);

		if (c == -1)
			break;

		switch (c) {
		case 'a': {
			errno = 0;
			i2c_address = (int) strtol(optarg, NULL, 0);
			if ( errno != 0) {
				fprintf( stderr, "Unknown address parameter %s.\n", optarg);
				exit(1);
			}
			break;
		}

		case 'b': {
			errno = 0;
			i2c_bus = (int) strtol(optarg, NULL, 0);
			if ( errno != 0) {
				fprintf( stderr, "Unknown bus parameter %s.\n", optarg);
				exit(1);
			}
			break;
		}

		case 'c': {
			operation = OP_CURRENT;
			break;
		}

		default:
		case 'h': {
			operation = OP_NONE;
			show_usage(argv[0]);
			break;
		}

		case 'i': {
			operation = OP_MONITOR;
			interval = atoi(optarg);
			if ( errno != 0 || interval < MIN_INTERVAL) {
				fprintf( stderr, "Invalid interval value, minimal: %u ms\n", MIN_INTERVAL);
				exit(1);
			}
			break;
		}

		case 'v': {
			operation = OP_VOLTAGE;
			break;
		}

		case 'w': {
			whole_numbers = 1;
			break;
		}
		case 's': {
			config = (unsigned short) strtol(optarg, NULL, 0);
			if ( errno != 0) {
				fprintf( stderr, "Invalid config value\n");
				exit(1);
			}
			setcfg = 1;
			break;
		}
		case 'l': {
			samplescount = strtol(optarg, NULL, 0);
			if ( errno != 0) {
				fprintf( stderr, "Invalid count value\n");
				exit(1);
			}
			if (!samplescount)
				samplescount = 1;
			break;
		}
		}
	}
}

int get_voltage(float *mv) {
	short bus;

	if (register_read( BUS_REG, (unsigned short*) &bus) < 0) {
		return -1;
	}

	*mv = (float) ((bus & 0xFFF8) >> 1);
	if ((bus & 1) != 0)
		fprintf( stderr,
				"INA219: Power or Current calculations are out of range\n");
	if ((bus & 2) == 0)
		fprintf( stderr, "INA219: Conversion Ready!\n");

	return 0;
}

int get_current(float *ma) {
	short shunt;

	if (register_read( SHUNT_REG, (unsigned short *) &shunt) < 0) {
		return -1;
	}

	*ma = (float) shunt / 100.0;
	return 0;
}

void show_current(void) {
	float ma;
	if (get_current(&ma)) {
		fprintf( stderr, "Error reading current\n");
		return;
	}

	if (whole_numbers) {
		printf("%4.0f\n", ma);
	} else {
		printf("%04.1f\n", ma);
	}
}

void show_voltage(void) {
	float mv;

	if (get_voltage(&mv)) {
		fprintf( stderr, "Error reading voltage\n");
		return;
	}
	printf("%4.0f\n", mv);
}

void show_voltage_current(void) {
	float mv, ma;

	if (get_voltage(&mv) || get_current(&ma)) {
		fprintf( stderr, "Error reading voltage/current\n");
		return;
	}

	if (whole_numbers) {
		printf("%04.0f;%.2f\n", mv, ma);
	} else {
		printf("%04.0fmV  %.2fmA\n", mv, ma);
	}
}

#include <signal.h>
#include <time.h>

#define errExit(msg)    do { perror(msg); return EXIT_FAILURE; } while (0)
#define CLOCKID CLOCK_REALTIME
#define SIG SIGRTMIN

sigset_t mask;
timer_t timerid;
volatile int expire = 0;
int overrun_cnt = 0;
void timer_handler(int signo, siginfo_t *info, void *context) {
	if (info->si_code == SI_TIMER) {
		expire++;
#if 1
		int overrun;
		timer_t *tidp;
		tidp = info->si_value.sival_ptr;
		/*
		 * Спецификация говорит, что в один момент времени
		 * для данного таймера только один экземпляр сигнала
		 * помещается в очередь процесса. Если таймер, сигнал
		 * которого всё еще ожидает доставки, заканчивает
		 * работу, сигнал не помещается в очередь и происходит
		 * ситуация переполнения таймера. timer_getoverrun
		 * возвращает дополнительное число окончаний работы
		 * таймера, которые произошли между моментом, когда
		 * был сгенерирован сигнал (помещён в очередь) и
		 * моментом, когда он был доставлен или принят
		 */
		if ((overrun = timer_getoverrun(*tidp)) != -1 && overrun != 0) {
//			printf("timer overrun %d\n", overrun);
			overrun_cnt += overrun;
		}
#endif

		if (expire >= samplescount) {
			sigprocmask(SIG_SETMASK, &mask, NULL);
			timer_delete(timerid);
		}
		show_voltage_current();
	}
}

#define ADD_CNV 0 // (CONV_STEP*2)
int setspeed(void) {
	unsigned short speed = 3;
	long long interval_us = interval * 1000;
	if (register_read( CONFIG_REG, &config) < 0) {
		return -1;
	}
	if (interval_us > CONV_STEP * 128 + ADD_CNV)
		speed = 0xFF;
	else if (interval_us > CONV_STEP * 64 + ADD_CNV)
		speed = 0xee;
	else if (interval_us > CONV_STEP * 32 + ADD_CNV)
		speed = 0xdd;
	else if (interval_us > CONV_STEP * 16 + ADD_CNV)
		speed = 0xcc;
	else if (interval_us > CONV_STEP * 8 + ADD_CNV)
		speed = 0xbb;
	else if (interval_us > CONV_STEP * 4 + ADD_CNV)
		speed = 0xaa;
	else if (interval_us > CONV_STEP * 2 + ADD_CNV)
		speed = 0x99;
	else if (interval_us > CONV_STEP + ADD_CNV)
		speed = 0x88;
	config = (config & 0x3807) | (speed << 3);
	if (register_write(CONFIG_REG, config) < 0) {
//		fprintf( stderr, "Error writing config register 0x%04x\n", config );
		return -1;
	}
	else if (!whole_numbers)
		printf("Set config register 0x%04X\n", config);
	return 0;
}

int monitor(void) {
	struct sigevent sev;
	struct itimerspec its;
	long long freq_nanosecs;
	struct sigaction sa;

	/* Establish handler for timer signal */
	sa.sa_flags = (unsigned long) SA_SIGINFO;
	sigemptyset(&sa.sa_mask);
	sa.sa_sigaction = timer_handler;
	if (sigaction(SIG, &sa, NULL) == -1)
		errExit("sigaction");
	/* Block timer signal temporarily */
	sigemptyset(&mask);
	sigaddset(&mask, SIG);
	if (sigprocmask(SIG_SETMASK, &mask, NULL) == -1)
		errExit("sigprocmask");
	/* Create the timer */
	sev.sigev_notify = SIGEV_SIGNAL;
	sev.sigev_signo = SIG;
	sev.sigev_value.sival_ptr = &timerid;
	if (timer_create(CLOCKID, &sev, &timerid) == -1)
		errExit("timer_create");
	if (sigprocmask(SIG_UNBLOCK, &mask, NULL) == -1)
		errExit("sigprocmask");

	if (whole_numbers) {
		printf("mV;mA;step=%dms\n", interval);
	}
	/* Start the timer */
	freq_nanosecs = interval * 1000000; // interval in ms -> ns
	its.it_value.tv_sec = freq_nanosecs / 1000000000;
	its.it_value.tv_nsec = freq_nanosecs % 1000000000;
	its.it_interval.tv_sec = its.it_value.tv_sec;
	its.it_interval.tv_nsec = its.it_value.tv_nsec;
	if (timer_settime(timerid, 0, &its, NULL) == -1)
		errExit("timer_settime");
	/* Спим и ждём сигнал */
	for (;;) {
		sleep(0);
		if (expire >= samplescount) {
			printf("Ok, %d samples\n", expire);
			if(overrun_cnt)
				printf("Warning: Overrun count %d!\n", overrun_cnt);
			return EXIT_SUCCESS;
		}
	}
	return (EXIT_FAILURE);
}

#ifdef SET_TTY_OPT
// WIN32: _setmode(handle, _O_BINARY)
int set_tty_options(void) {
	struct termios options;
	int rc = tcgetattr(handle, &options);
	if (rc >= 0) {
		options.c_iflag &= ~(INLCR | IGNCR | ICRNL | IXON | IXOFF);
		options.c_oflag &= ~(ONLCR | OCRNL);
		options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
		rc = tcsetattr(handle, TCSANOW, &options);
	}
	return rc;
}
#endif

int main(int argc, char *argv[]) {
	char filename[20];
	int rc = EXIT_SUCCESS;

	parse(argc, argv);

	snprintf(filename, sizeof(filename) - 1, "/dev/ttyACM%d", i2c_bus);
	handle = open(filename, O_RDWR | O_NONBLOCK | O_NOCTTY);
	if (handle < 0) {
		fprintf( stderr, "Error opening %s: %s\n", filename, strerror( errno));
		exit(1);
	}
#ifdef SET_TTY_OPT
	if (set_tty_options() < 0) {
		fprintf( stderr, "Error set option handle %s: %s\n", filename,
				strerror( errno));
		exit(1);
	}
#endif
	if (ubia_get_version(&dev_ver) != 0) {
		fprintf( stderr, "Error get device version: %s\n", strerror( errno));
		exit(1);
	}

	if (!whole_numbers)
		printf("UBIA DevID: %04x, ver: %04x\n", dev_ver.id, dev_ver.ver);

	if (ubia_i2c_clk(i2c_clk_khz) != 0) {
		fprintf( stderr, "Error set speed i2c %d kHz: %s\n", i2c_clk_khz,
				strerror( errno));
		exit(1);
	}

	if (setcfg && register_write(CONFIG_REG, config) < 0) {
		fprintf( stderr, "Error writing config register\n");
		exit(1);
	}

	switch (operation) {
	case OP_DUMP: {
		show_voltage_current();
		break;
	}

	case OP_VOLTAGE: {
		show_voltage();
		break;
	}

	case OP_CURRENT: {
		show_current();
		break;
	}

	case OP_MONITOR: {
		if (!setcfg)
			setspeed();
		rc = monitor();
		break;
	}

	default:
	case OP_NONE: {
		break;
	}
	}

	close(handle);
	return rc;
}

