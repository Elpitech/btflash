bindir ?= /usr/bin

CC := $(CROSS_COMPILE)gcc

CFLAGS+=-D_GNU_SOURCE -std=c99 -Wall

PROG = btflash2
OBJS = btflash2.o

all: $(PROG)

btflash2.o: btflash2.c spi.h spi_regs.h

install:
	install -d -D -m 755 ${DESTDIR}${bindir}
	install -m 755 $(PROG) ${DESTDIR}${bindir}

clean:
	rm -f $(OBJS) $(PROG)

