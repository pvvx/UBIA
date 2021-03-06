# Makefile

TARGET  = floader

# This assumes that you install Telink toolchain for Windows, MacOS or Linux
# Under Windows you will be using MingW/Msys environment
#
GCCPATH = /opt/tc32/bin/tc32
AS      = $(GCCPATH)-elf-as
CC      = $(GCCPATH)-elf-gcc
LD      = $(GCCPATH)-elf-ld
NM      = $(GCCPATH)-elf-nm
OBJCOPY = $(GCCPATH)-elf-objcopy
OBJDUMP = $(GCCPATH)-elf-objdump
ARCH	= $(GCCPATH)-elf-ar
SIZE	= $(GCCPATH)-elf-size


CRT0=./mcu/cstartup_8266.S

WARNINGS=-Wall -Wextra -Wshadow -Wpointer-arith -Wbad-function-cast -Wcast-align -Wsign-compare \
		-Waggregate-return -Wunused

OTHER_CFLAGS= -fpack-struct -fshort-enums -finline-small-functions -fshort-wchar -fms-extensions

CFLAGS= -O2 $(WARNINGS) $(OTHER_CFLAGS) -std=gnu99 -I. -I./Include
	
LDFLAGS = --gc-sections -T ./boot.link

SOURCE_DIR=./Source

TC32_SRC = \
$(SOURCE_DIR)/main.c \
$(SOURCE_DIR)/crc.c \
$(SOURCE_DIR)/sleep.c \
$(SOURCE_DIR)/analog.c \
$(SOURCE_DIR)/flash.c

#
# Define all object files.
#
TC32_OBJ = $(TC32_SRC:.c=.o)

CRT0_OBJ = $(CRT0:.S=.o)

shws: $(TARGET).elf
	python TlsrMemInfo.py $<


all: $(TARGET).elf shws


$(TARGET).elf : $(TC32_OBJ) $(CRT0_OBJ)
	$(LD) $(LDFLAGS) -o $(TARGET).elf $(TC32_OBJ) $(CRT0_OBJ)
	$(OBJCOPY) -O binary $(TARGET).elf $(TARGET).bin
	$(OBJDUMP) -x -D -l -S $(TARGET).elf > $(TARGET).lst
	$(SIZE) --format=SysV $(TARGET).elf

$(TC32_OBJ) : %.o : %.c
	$(CC) -c $(CFLAGS) $< -o $@
	$(OBJDUMP) -x -D -l -S $*.o > $*.lst
	
$(CRT0_OBJ) : %.o : %.S
	$(AS) $< -o $@
	$(OBJDUMP) -x -D -l -S $*.o > $*.lst
	
print-%  : ; @echo $* = $($*)

clean :
	rm -rf $(TC32_OBJ) *.map *.mot *.elf *.a *.abs *.i *.s *.list
	find $(SOURCE_DIR) -name "*.lst" -exec rm {} \;
	find ./ -name "*.o" -exec rm {} \;
	find ./ -name "*.a" -exec rm {} \;
	find ./ -name "cscope.*" -exec rm {} \;
	find ./ -name "tags" -exec rm {} \;
	touch Makefile
