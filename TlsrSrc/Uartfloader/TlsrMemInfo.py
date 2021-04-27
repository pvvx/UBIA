#!/usr/bin/env python
 
### TlsrMemInfo.py ###
###  Autor: pvvx   ###
 
import sys
import signal
import struct
import platform
import time
import argparse
import subprocess
import os
import io

__progname__ = "TLSR826x MemInfo"
__filename__ = "TlsrMemInfo"
__version__ = "0.1beta"

#tool_nm = "e:/Telink/SDK/opt/tc32/bin/"

class FatalError(RuntimeError):
	def __init__(self, message):
		RuntimeError.__init__(self, message)

	@staticmethod
	def WithResult(message, result):
		message += " (result was %s)" % hexify(result)
		return FatalError(message)

def signal_handler(signal, frame):
	print()
	print('Keyboard Break!')
	sys.exit(0)
	
def arg_auto_int(x):
	return int(x, 0)
	
class ELFFile:

	def __init__(self, name):
		self.name = name
		self.symbols = {}
		try:
			tool_nm = "tc32-elf-nm"
			#if sys.platform == 'linux2':
			#	tool_nm = "tc32-elf-nm"
			proc = subprocess.Popen([tool_nm, self.name], stdout=subprocess.PIPE)
		except OSError:
			print("Error calling " + tool_nm + ", do you have Xtensa toolchain in PATH?")
			sys.exit(1)
		for l in proc.stdout:
			fields = l.strip().split()
			try:
				if fields[0] == b"U":
					print("Warning: Undefined symbol '%s'!" %(fields[1].decode('ASCII')))
					continue
				if fields[0] == b"w":
					continue  # can skip weak symbols
				self.symbols[fields[2]] = int(fields[0], 16)
			except ValueError:
				raise FatalError("Failed to strip symbol output from nm: %s" % fields)

	def get_symbol_addr(self, sym, default = 0):
		try:
			x = self.symbols[sym]
		except:
			return default
		return x
'''
	def load_section(self, section):
		tool_objcopy = "e:/Telink/SDK/opt/tc32/bin/tc32-elf-objcopy.exe"
		if sys.platform == 'linux2':
			tool_nm = "tc32-elf-objcopy"
		tmpsection = tempfile.mktemp(suffix=".section")
		try:
			subprocess.check_call([tool_objcopy, "--only-section", section, "-Obinary", self.name, tmpsection])
			with open(tmpsection, "rb") as f:
				data = f.read()
		finally:
			os.remove(tmpsection)
		return data
'''

def main():

	signal.signal(signal.SIGINT, signal_handler)
	parser = argparse.ArgumentParser(description='%s version %s' % (__progname__, __version__), prog=__filename__)
	parser.add_argument(
		"--size", "-s",
		help="Chip SRAM Size (default: 16384)",
		type=arg_auto_int, 
		default=16384)
	parser.add_argument('elffname', help='Name of elf file')		
	args = parser.parse_args()
	#tool_nm = args.tc32

	print('%s version %s' % (__progname__, __version__))

	sec_name = ["ramcode", "ictag","icdata", "data", "bss", "stack", "flash"]
	sec_des = ["Resident Code SRAM", "Cache Table SRAM", "Cache Data SRAM", "Data SRAM", "BSS Data SRAM", "CPU Stack SRAM", "Bin Size FLASH"]
	sec_start = [b"__start", b"_ictag_start_", b"_ictag_end_", b"_start_data_", b"_start_bss_", b"_end_bss_", b"__start"]
	sec_start_def = [0, 0, 0, -1, -1, 0, 0]
	sec_start_add = [0x808000, 0, 0, 0, 0, 0, 0]
	sec_end = [b"_start_data_", b"_ictag_end_", b"_ictag_end_", b"_end_data_", b"_end_bss_", b"__chip_ram", b"_bin_size_"]
	sec_end_def = [0, 0, 0, 0, 0, 0, 0]
	sec_end_add = [0, 0, 0, 0, 0, 0x808000 + args.size, 0]
	sec_size = []

	e = ELFFile(args.elffname)
	load_ram = e.get_symbol_addr(b"__LOAD_RAM", 0)*0x100
	if load_ram != 0:
		sec_start_def[1] = 0x808900+load_ram-0x900 
		sec_start_def[2] = 0x808900+load_ram-0x800
		sec_end[0] = b"_ictag_start_"
		sec_end_def[0] = 0x808900+load_ram-0x900
		sec_end_def[1] = 0x808900+load_ram-0x800
		sec_end_add[2] = 0x800
	print("=================================================================")
	print("{0:>8}|{1:>19}|{2:>12}|{3:>12}|{4:>8}".format("Section", "Description", "Start (hex)", "End (hex)", "Used space"))
	print("-----------------------------------------------------------------")

	for i in range(len(sec_name)):
		ss = e.get_symbol_addr(sec_start[i], sec_start_def[i]) + sec_start_add[i]
		se = e.get_symbol_addr(sec_end[i], sec_end_def[i]) + sec_end_add[i]
		sec_size.append(int(se - ss))
		print("{0:>8}|{1:>19}|{2:>12X}|{3:>12X}|{4:>8d}".format(sec_name[i], sec_des[i], ss, se, sec_size[i]))
	print("-----------------------------------------------------------------")

	ram_used = sec_size[0] + sec_size[1] + sec_size[2] + sec_size[3] + sec_size[4]
	total_free = sec_size[5] - 256
	print("{0} : {1:d}".format("Total Used SRAM", ram_used))
	print("{0} : {1:d}".format("Total Free SRAM", total_free))
	sys.exit(0);
	

if __name__ == '__main__':
	main()
