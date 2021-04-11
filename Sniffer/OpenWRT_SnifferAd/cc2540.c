/*
 * Create 11.04.2021 pvvx
 *
 * Based on https://github.com/andrebdo/wireshark-cc2531 :
 * Copyright (C) 2019 Andre B. Oliveira
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <dirent.h>
#include <fcntl.h>
#include <linux/usbdevice_fs.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <unistd.h>

typedef int cc2540_handle;

#ifndef HAVE_DAEMON
#include <fcntl.h>
#include <unistd.h>
/*
 * System function daemon() replacement based on FreeBSD implementation.
 * Original source file CVS tag:
 * $FreeBSD: src/lib/libc/gen/daemon.c,v 1.3 2000/01/27 23:06:14 jasone Exp $
 */
int daemon(nochdir, noclose)
	int nochdir, noclose; {
	int fd;

	switch (fork()) {
	case -1:
		return (-1);
	case 0:
		break;
	default:
		_exit(0);
	}

	if (setsid() == -1)
		return (-1);

	if (!nochdir)
		(void) chdir("/");

	if (!noclose && (fd = open("/dev/null", O_RDWR, 0)) != -1) {
		(void) dup2(fd, STDIN_FILENO);
		(void) dup2(fd, STDOUT_FILENO);
		(void) dup2(fd, STDERR_FILENO);
		if (fd > 2)
			(void) close(fd);
	}
	return (0);
}
#endif

/*
 * Open CC2540 USB device
 */
#if 0
const unsigned char cc2540_usb_device_descriptor[] = {
	0x12, /* bLength */
	0x01, /* bDescriptorType */
	0x00, 0x02, /* bcdUSB */
	0x00, /* bDeviceClass */
	0x00, /* bDeviceSubClass */
	0x00, /* bDeviceProtocol */
	0x??, /* bMaxPacketSize0 */
	0x51, 0x04, /* idVendor */
	0xb3, 0x16, /* idProduct */
};
#endif
static cc2540_handle cc2540_open(void) {
	char path[21] = "/dev/bus/usb/"; /* "/dev/bus/usb/XXX/YYY" */
	DIR *dir1 = opendir(path);
	if (dir1) {
		struct dirent e, *r;
		while (readdir_r(dir1, &e, &r) == 0 && r != NULL) {
			if (strlen(e.d_name) == 3) {
				path[13] = e.d_name[0];
				path[14] = e.d_name[1];
				path[15] = e.d_name[2];
				path[16] = '/';
				path[17] = 0;
				DIR *dir2 = opendir(path);
				if (dir2) {
					while (readdir_r(dir2, &e, &r) == 0 && r != NULL) {
						if (strlen(e.d_name) == 3) {
							path[17] = e.d_name[0];
							path[18] = e.d_name[1];
							path[19] = e.d_name[2];
							int fd = open(path, O_RDWR);
							if (fd >= 0) {
								char descriptor[12];
								int
										n = read(fd, descriptor,
												sizeof(descriptor));
								if (n == sizeof(descriptor) && descriptor[0]
										== 0x12 && descriptor[1] == 0x01
										&& descriptor[2] == 0x00
										&& descriptor[3] == 0x02
										&& descriptor[8] == 0x51
										&& descriptor[9] == 0x04
										&& descriptor[10] == 0xb3
										&& descriptor[11] == 0x16) {
									closedir(dir2);
									closedir(dir1);
									return fd;
								}
								close(fd);
							}
						}
					}
					closedir(dir2);
				}
			}
		}
		closedir(dir1);
	}
	return -1;
}

/*
 * Executes a USB control transfer on the CC2540.
 * Returns the number of data bytes transferred, or -1 in case of error.
 */
static int cc2540_control(cc2540_handle handle, int request, int index,
		int length, const unsigned char *data_byte) {
	const int request_type = 64;
	struct usbdevfs_ctrltransfer ctrl;
	ctrl.bRequestType = request_type;
	ctrl.bRequest = request;
	ctrl.wValue = 0;
	ctrl.wIndex = index;
	ctrl.wLength = length;
	ctrl.data = (void *) data_byte;
	ctrl.timeout = 5000; /* milliseconds */
	return ioctl(handle, USBDEVFS_CONTROL, &ctrl);
}

/*
 * Sends the commands to the CC2540 to start capturing
 */
static void cc2540_start(cc2540_handle handle, unsigned char channel,
		const unsigned char * mac) {
	/* Set power */
	cc2540_control(handle, 197, 4, 0, 0);

	/* Wait until powered up */
	sleep(1);

	cc2540_control(handle, 201, 0, 0, 0);

	/* Set channel */
	cc2540_control(handle, 210, 0, 1, &channel);
	if (mac != NULL)
		cc2540_control(handle, 210, 6, 6, mac);
	else
		cc2540_control(handle, 210, 6, 0, 0);

	cc2540_control(handle, 210, 1, 1, 0);

	/* Start capture */
	cc2540_control(handle, 208, 0, 0, 0);
}

static void cc2540_stop(cc2540_handle handle) {
	cc2540_control(handle, 209, 0, 0, 0);
}

/*
 * Reads the captured packet bytes from the CC2540.
 */
static int cc2540_read(cc2540_handle handle, void *data, int length) {
	const int endpoint = 0x83;
	struct usbdevfs_bulktransfer bulk;
	bulk.ep = endpoint;
	bulk.len = length;
	bulk.data = data;
	bulk.timeout = 1000; /* milliseconds */
	return ioctl(handle, USBDEVFS_BULK, &bulk);
}

/*
 * Gets a CC2540 capture packet from a buffer.
 * The format of the CC2540 capture packets is:
 *
 * Offset  Bytes  Description
 * --------------------------
 * 0       1      ?
 * 1       1      Number of bytes from offset 3 to the end of this packet
 * 2       1      ?
 * --------------------------
 * 3       4      Some kind of timestamp
 * 7       1      Payload length (N)
 * -------------------------------------
 * 8       N-2    Payload
 * 8+N-2   1      RSSI
 * 8+N-1   1      CRC OK
 *
 * The CC2540 also sends the following packet periodically when idle:
 * 01 01 00 XX  (where XX is a counter that increments by 4)
 */
static int cc2540_get_packet(unsigned char *buffer, int *head, int *tail,
		int *length) {
	for (;;) {
		int available = *head - *tail;
		if (*tail > 0) {
			memmove(buffer, buffer + *tail, available);
			*head = available;
			*tail = 0;
		}
		if (available < 4) {
			return 0;
		}
		if (buffer[0] == 1 && buffer[1] == 1 && buffer[2] == 0) {
			/* alive packet */
			*tail = 4;
			continue;
		}
		if (available < 8) {
			return 0;
		}
		int phy_payload_length = buffer[7];
		if (available < 8 + phy_payload_length) {
			return 0;
		}
		*head -= 8;
		memmove(buffer, buffer + 8, *head);
		*tail = phy_payload_length;
		*length = phy_payload_length;
		return 1;
	}
}

/*
 * Writes the header of a capture packet.
 */
static void cc2540_write_pcap_packet(FILE *file, const void *packet, int length) {
	int i = 0;
	unsigned char buf[1024];
	unsigned char * ptr = (unsigned char *) packet;
	struct timeval t;
	gettimeofday(&t, NULL);
	struct pcap_packet_header {
		int ts_sec;
		int ts_usec;
		int incl_len;
	} header = { t.tv_sec, /* timestamp seconds */
	t.tv_usec, /* timestamp microseconds */
	length, /* number of bytes of packet data */
	};
	if (file == stdout) {
		if (length < sizeof(buf) / 2)
			for (; i < length; i++) {
				unsigned char c = (ptr[i] >> 4) & 0x0f;
				if (c > 9)
					c += 'a' - 10;
				else
					c += '0';
				buf[i * 2] = c;
				c = ptr[i] & 0x0f;
				if (c > 9)
					c += 'a' - 10;
				else
					c += '0';
				buf[i * 2 + 1] = c;
			}
		buf[i * 2] = 0;
		fprintf(stdout, "%08lx-%08lx:%s\n", t.tv_sec, t.tv_usec, buf);
	} else {

		fwrite(&header, sizeof(header), 1, file);
		fwrite(packet, length, 1, file);
	}
	fflush(file);
}

void show_usage(void) {
	puts(
			"Usage:\n"
				" cc2540 --channel <37-39> [--mac xx:xx:xx:xx:xx:xx] [--fifo <path>]"
#ifndef HAVE_DAEMON
				" [--daemon]"
#endif
				"\n");
}

/*
 * Starts capturing packets and writes them in PCAP format.
 */
static int cc2540_capture(int channel, const unsigned char * mac,
		const char *path) {
	cc2540_handle handle;
	FILE *file;
	unsigned char buffer[512];
	int tail = 0;
	int head = 0;

	if (path == NULL)
		file = stdout; // stdout
	else
		file = fopen(path, "wb");
	if (file) {

		handle = cc2540_open();
		if (handle == -1) {
			fprintf(stderr, "Device not found!\n");
			exit(3);
		}

		cc2540_stop(handle);

		while (cc2540_read(handle, buffer + head, sizeof(buffer) - head) > 0)
			;

		cc2540_start(handle, channel, mac);

		for (;;) {
			int n = cc2540_read(handle, buffer + head, sizeof(buffer) - head);
			if (n < 0) {
				if (errno == ETIMEDOUT)
					continue;
				else
					break;
			}
			head += n;

			int length;
			while (cc2540_get_packet(buffer, &head, &tail, &length)) {
				if (length > 127) {
					cc2540_stop(handle);
					return 1;
				}
				cc2540_write_pcap_packet(file, buffer, length);
			}
		}

		cc2540_stop(handle);
	} else {
		fprintf(stderr, "Can't open file '%s' (%s)\n", path, strerror(errno));
	}

	return 1;
}

/*
 * cc2540 --channel N --fifo \\.\pipe\cc2540.log
 */
int main(int argc, char *argv[]) {
	int channel = 0;
#ifndef HAVE_DAEMON
	int isdaemon =0;
#endif
	unsigned char bin_mac[6] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
	char *fifo = NULL;
	int mac_ok = 0;
	int i;

	for (i = 1; i < argc; i++) {
		if (strcmp(argv[i], "--channel") == 0) {
			channel = atoi(argv[++i]);
		} else if (strcmp(argv[i], "--fifo") == 0) {
			fifo = argv[++i];
		} else if (strcmp(argv[i], "--mac") == 0) {
			int last = -1;
			int rc = sscanf(argv[++i], "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx%n",
					bin_mac + 5, bin_mac + 4, bin_mac + 3, bin_mac + 2,
					bin_mac + 1, bin_mac + 0, &last);
			if (rc != 6 || strlen(argv[i]) != last) {
				fprintf(stderr, "Error string MAC '%s'!\n", argv[i]);
				exit(1);
			} else
				mac_ok = 1;
#ifndef HAVE_DAEMON
		} else if (strcmp(argv[i], "--daemon") == 0) {
			isdaemon = 1;
#endif
		} else {
			show_usage();
			exit(1);
		}
	}

	if (channel) {
#ifndef HAVE_DAEMON
		/* go or not to daemon mode? */
		int rc;
		if (isdaemon && (rc = daemon(1, 1))) {
			fprintf(stderr, "Can't be daemonized (%s), exiting...",
					strerror(errno));
			exit(rc);
		}
#endif
		exit(cc2540_capture(channel, ((mac_ok != 0) ? bin_mac : NULL), fifo));
	}
	show_usage();
	exit(1);
}
