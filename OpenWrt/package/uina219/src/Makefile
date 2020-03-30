CFLAGS+=
SFLAGS:=-Os --std=gnu99
WFLAGS:=-Wall -Werror -pedantic
LDFLAGS?=

all: uina219

ina219: uina219.c 
	$(CC) $(C_DEFINES) -I. $(CFLAGS) $(SFLAGS) -lrt $(WFLAGS) $(LDFLAGS) -o $@ $+

clean:
	rm -f $(BINARY)
