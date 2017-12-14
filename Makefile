CC=$(CROSS_COMPILE)gcc
CFLAGS+=-D_GNU_SOURCE -std=c99

all: btflash2

btflash2.o: btflash2.c spi.h spi_regs.h

clean:
	rm *.o btflash2

