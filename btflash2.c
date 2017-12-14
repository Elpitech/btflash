/*
 * File:        spi.c
 * Author:      Alexey Malahov <Alexey.Malahov@baikalelectronics.com>
 *              Ramil Zaripov <Ramil.Zaripov@baikalelectronics.com>
 * Copyright (C) 2014 Baikal Electronics.
 * Description: spi library
 */

//#include <linux/stddef.h>       /* null, size, uint */
#include <stdint.h>
#include <sys/types.h>
#include <stdlib.h>
#include <stdio.h> // dbg
#include "spi.h"
//#include "tools.h"

int debug_level;

#define dprintf(X...)	do {	\
	if (debug_level)	\
		fprintf(stderr, X);	\
} while (0)

/* #include <exports.h> */
    void *malloc (size_t);
    void  free   (void*);
    int   printf (const char *fmt, ...);

/*
0x20BB15 = N25Q016, 1.8V, (uniform sectors expected)
0x20BA16 = N25Q032, 3.0V, (uniform sectors expected)
0x20BB16 = N25Q032, 1.8V, (uniform sectors expected)
0x20BA17 = N25Q064, 3.0V, (64KB/4KB blocks/sectors)
0x20BB17 = N25Q064, 1.8V, (64KB/4KB blocks/sectors)
0x20BA18 = N25Q128, 3.0V, (uniform sectors expected)
0x20BB18 = N25Q128, 1.8V, (uniform sectors expected)
0x20BA19 = N25Q256, 3.0V, (64KB/4KB blocks/sectors)
0x20BB19 = N25Q256, 1.8V, (uniform sectors expected)
0x20BA20 = N25Q512, 3.0V, (uniform sectors expected)
0x20BB20 = N25Q512, 1.8V, (uniform sectors expected)
0x20BA21 = N25Q00A, 3.0V, (uniform sectors expected)
(c) https://chromium.googlesource.com/chromiumos/third_party/flashrom/+/master/flashchips.h
*/
/*
const llenv32_spi_flashes_t spi_flashes[] = {
    {.mid = 0x20, .dev_type = 0xBA, .dev_cap = 0x18, .name = "N25Q128 3.0V"},
    {.mid = 0x20, .dev_type = 0xBA, .dev_cap = 0x19, .name = "N25Q256 3.0V"},
    {.mid = 0x20, .dev_type = 0xBB, .dev_cap = 0x18, .name = "N25Q128 1.8V"},
    {.mid = 0x20, .dev_type = 0xBB, .dev_cap = 0x19, .name = "N25Q256 1.8V"},
};
*/


#define SPI_SET_ADDRESS(a, b)   do {  \
    uint8_t* _b = (void*)(b);     \
    _b[1] = (((a) >> 16) & 0xFF); \
    _b[2] = (((a) >>  8) & 0xFF); \
    _b[3] = (((a) >>  0) & 0xFF); \
    } while (0)

void llenv32_sleep (int delay)
{
    volatile uint16_t x;
    while(delay--){
        x = 4;
        while(x--);
    }
}

/*
------------------------------
 LOWLEVEL FUNCTIONS
------------------------------
*/

void llenv32_spi_init(int spi_port)
{
    SPI_REG_SSIENR (spi_port) = SSIENR_SSI_DE;      /* Disable device. */

    /* crt0 */
#if 1
    SPI_REG_CTRLR0 (spi_port) .dfs    = 8 -1;
    SPI_REG_CTRLR0 (spi_port) .frf    = SPI_FRF_SPI;
    SPI_REG_CTRLR0 (spi_port) .scph   = SPI_SCPH_MIDDLE;
    SPI_REG_CTRLR0 (spi_port) .scpol  = SPI_SCPOL_LOW;
    SPI_REG_CTRLR0 (spi_port) .tmod   = SPI_TMOD_TR;
    SPI_REG_CTRLR0 (spi_port) .slv_oe = 0;
    SPI_REG_CTRLR0 (spi_port) .srl    = 0;
    SPI_REG_CTRLR0 (spi_port) .cfs    = 0;
#endif

    /* other reg's */
    SPI_REG_CTRLR1 (spi_port) = 0;
    SPI_REG_BAUDR  (spi_port) = SPI_BAUDR_SCKDV;
    SPI_REG_TXFTLR (spi_port) = SPI_TH_TX;
    SPI_REG_RXFTLR (spi_port) = SPI_TH_RX;
    SPI_REG_IMR    (spi_port) = 0;
    SPI_REG_SER    (spi_port) = 0;
}

int llenv32_spi_transfer (int port, int line, void *tx_, int tx_len, void *rx_, int rx_len)
{
    int err = 0;
    uint8_t *tx = tx_, *rx = rx_;

    /* check line */
    SPI_REG_SSIENR (port) = SSIENR_SSI_DE;
    line = (1<<line);
    SPI_REG_SER (port) = line;
    if(SPI_REG_SER(port) != line){
        return 20;
    }
    SPI_REG_SER (port) = 0;

    /* mode */
    if(rx_len)  SPI_REG_CTRLR0(port).tmod = SPI_TMOD_EPROMREAD;   /* read mode */
    else        SPI_REG_CTRLR0(port).tmod = SPI_TMOD_TO;          /* write mode */

    switch(SPI_REG_CTRLR0(port).tmod){
        case SPI_TMOD_TR:       /* mode TR not supported */
            return 21;

        case SPI_TMOD_RO:       /* mode RO not supported */
            return 22;

        case SPI_TMOD_TO:
            if(!tx || !tx_len || tx_len > (256+4)){
                return 23;
            }

            SPI_REG_SSIENR (port) = SSIENR_SSI_EN;      /* ebable fifo */

            int n = (tx_len > SPI_FIFO_SIZE)? SPI_FIFO_SIZE: tx_len;
            for (int i = 0; i < n; i++) {           /* tx config (first SPI_FIFO_SIZE bytes) */
                SPI_REG_DR(port) = tx[i];
            }
            SPI_REG_SER (port) = line;                  /* start sending */

            for (int i = n; i < tx_len;) {          /* tx config (continue to fill buffer) */
                if(SPI_REG_SR(port).tfnf)
                {
                    SPI_REG_DR(port) = tx[i];
                    i++;
                }
            }

            /* wait... */
#if 0
            llenv32_sleep(SPI_REG_BAUDR(port));         /* replace with mdelay, __udelay? */
#else
	    while ((SPI_REG_SR(port).tfe == 0) || SPI_REG_SR(port).busy)
		;
#endif
//	    if (SPI_REG_RISR(port) & 1)
//		fprintf(stderr, "TXEIR %d\n", SPI_REG_ICR(port)); // dbg

            /* clean */
            SPI_REG_SSIENR (port) = SSIENR_SSI_DE;      /* stop sending */
            SPI_REG_SER (port) = 0;
            break;

        case SPI_TMOD_EPROMREAD:
            if(!tx || !rx || !tx_len || !rx_len || tx_len > (256+4) || rx_len > 256){
                return 24;
            }
            SPI_REG_SER (port) = 0;                     /* dont send anything */
            SPI_REG_CTRLR1 (port) = rx_len - 1;         /* rx config */
            SPI_REG_SSIENR (port) = SSIENR_SSI_EN;      /* ebable fifo */
            for (int i = 0; i < tx_len; i++) {      /* tx config */
                SPI_REG_DR(port) = tx[i];
            }
            SPI_REG_SER(port) = line;                   /* start sending */


            for (int i = 0; i < rx_len; ) {         /* read incoming data */
                if(SPI_REG_SR(port).rfne)
                {
                    rx[i] = SPI_REG_DR(port);
                    i++;
                } else if ((SPI_REG_RISR(port) & 8)/*SPI_REG_SR(port).busy*/) {
                	/* timeout? ...*/
			err = -1 + SPI_REG_ICR(port);
			break;
		}
            }

            /* clean */
            SPI_REG_SSIENR (port) = SSIENR_SSI_DE;
            SPI_REG_SER (port) = 0;
            break;
    }

    return err;
}

int llenv32_spi_exec (int port, int line, uint8_t cmd_op, uint32_t address,
                    uint8_t *buffer, uint32_t buf_len)
{
    uint8_t cmd [4 + SPI_MAX_SIZE];  /* largest command takes 4 bytes, the data block occupies SPI_MAX_SIZE bytes */
    uint8_t *in, *out;
    uint16_t in_len, out_len;
    size_t size;


    cmd[0] = cmd_op;            /* Save the SPI flash instruction. */
    switch (cmd_op) {           /* Prepare arguments for the SPI transaction. */
        case SPI_FLASH_RDID:    /* Read identification. */
            in      = cmd;
            in_len  = 1;
            out     = buffer;
            out_len = (buf_len > JEDEC_DATA)? JEDEC_DATA : buf_len;
            break;


        case SPI_FLASH_READ:    /* Read Data Bytes */
            in      = cmd;
            in_len  = 4;
            out     = buffer;
            out_len = (buf_len > SPI_MAX_SIZE)? SPI_MAX_SIZE : buf_len;

            SPI_SET_ADDRESS(address, cmd);
            break;

        case SPI_FLASH_RDSR:    /* Read Status Register */
	case 0x35:
	case 0x15:
            in      = cmd;
            in_len  = 1;
            out     = buffer;
            out_len = buf_len;
            break;


        case SPI_FLASH_WRSR:    /* Write Status Register */
            in      = cmd;
            in_len  = 2;
            out     = 0;
            out_len = 0;
            cmd[1]  = buffer[0];
            break;


        case SPI_FLASH_SSE:     /* SubSector Erase */
        case SPI_FLASH_SE:      /* Sector Erase */
            in      = cmd;
            in_len  = 4;
            out     = 0;
            out_len = 0;
            SPI_SET_ADDRESS(address, cmd);
            break;


        case SPI_FLASH_WRDI:    /* Write Disable */
        case SPI_FLASH_WREN:    /* Write Enable */
        case SPI_FLASH_BE:      /* Bulk Erase */
            in      = cmd;
            in_len  = 1;
            out     = 0;
            out_len = 0;
            break;


        case SPI_FLASH_PP:      /* Page Program */
            size = (buf_len > SPI_MAX_SIZE)? SPI_MAX_SIZE : buf_len;
            in      = cmd;
            in_len  = 4 + size;
            out     = 0;
            out_len = 0;
            SPI_SET_ADDRESS(address, cmd);
            for (int i = 0; i < size; i++) {
                cmd[i+4] = buffer[i];
            }
            break;

        default:
            dprintf("%s: SPI instruction %d is not provided\n", __FUNCTION__, cmd_op);
            return 11;
    }

    /* Execute the SPI transaction */
    return llenv32_spi_transfer(port, line, in, in_len, out, out_len);
}




/*
------------------------------
 HIGH LEVEL FUNCTIONS
------------------------------
*/

int llenv32_spi_status (int port, int line, void *status)
{
    return llenv32_spi_exec (port, line, SPI_FLASH_RDSR, 0, status, 1);
}

int llenv32_spi_wren (int port, int line)
{

#if 1
    uint8_t status;
    llenv32_spi_exec (port, line, SPI_FLASH_WREN, 0, 0, 0);
    llenv32_spi_exec (port, line, SPI_FLASH_RDSR, 0, &status, 1);
    return (status & SPI_FLASH_SR_WEL)? 0 : -1;
#else
    int err;
    uint8_t status;

    err = llenv32_spi_exec (port, line, SPI_FLASH_WREN, 0, 0, 0);
    if(err)
        return err;

    err = llenv32_spi_status(port, line, &status);
    if(err)
        return err;

    return (status & SPI_FLASH_SR_WEL)? 0 : -1;
#endif
}



int llenv32_spi_wait (int port, int line)
{
#if 1
    int err;
    uint8_t status;
    do {
        err = llenv32_spi_exec (port, line, SPI_FLASH_RDSR, 0, &status, 1);
        if(err)
            return err;
    } while (status & SPI_FLASH_SR_WIP);
    return 0;
#else
    int err;
    uint8_t status;


    do {
        err = llenv32_spi_status(port, line, &status);
        if(err)
            return err;
    } while (status & SPI_FLASH_SR_WIP);

    return 0;
#endif
}

int llenv32_spi_erase (int port, int line, int adr)
{
    int err;
    err = llenv32_spi_wren(port, line);
    if(err)
        return err;

    err = llenv32_spi_exec (port, line, SPI_FLASH_SE, adr, 0, 0);
    if(err)
        return err;

    err = llenv32_spi_wait(port, line);
    if(err)
        return err;

    return 0;
}




int llenv32_spi_write (int port, int line, int adr, void *data, uint32_t size)
{
    int err;
    err = llenv32_spi_wren(port, line);
    if(err)
        return err;

    err = llenv32_spi_exec (port, line, SPI_FLASH_PP, adr, data, size);
    if(err)
        return err;

    err = llenv32_spi_wait(port, line);
    if(err)
        return err;

    return 0;
}
int llenv32_spi_write_sector (int port, int line, int adr, void *sector)
{
    int err;
    uint32_t cnt = SPI_SECTOR_SIZE;
    uint8_t *p = sector;

    while(cnt){
        err = llenv32_spi_write  (port, line, adr, p, SPI_MAX_SIZE);
        if(err)
            return err;

        p += SPI_MAX_SIZE;
        adr += SPI_MAX_SIZE;
        cnt -= SPI_MAX_SIZE;
        usleep(100);
    }
    return 0;
}



/*
------------------------------------
 UTILITY FUNCTIONS
------------------------------------
*/

#define LLENV32_SPI_REGCNT 24 /* number of stored registers */

void* llenv32_spi_store_cfg (int port)
{
    void *cfg = malloc(LLENV32_SPI_REGCNT*sizeof(uint32_t));
    if(!cfg)
        return NULL;

    uint32_t *a = (void*) SPI_PORT(port);
    uint32_t *b = cfg;
    for(int i = 0; i < LLENV32_SPI_REGCNT; i++){
        *b++ = *a++;
    }
    return cfg;
}

int llenv32_spi_restore_cfg (int port, void *cfg)
{
    if(!cfg)
        return -1;

    uint32_t *a = (void*) SPI_PORT(port);
    uint32_t *b = cfg;
    for(int i = 0; i < LLENV32_SPI_REGCNT; i++){
        *a++ = *b++;
    }
    free(cfg);
    return 0;
}


void *bootctl_map;

#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <getopt.h>
#include <sys/fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>

#define SSECTOR_SIZE	0x1000	/* 4K */
#define MAX_RETRY	5	/* retries per subsector */
#define MAX_RETRY2	2	/* retries per read request */

int read_size = 256;	/* must be power of 2 and < SPI_MAX_SIZE */

int spi_read(uint32_t addr, uint8_t *buf, int size)
{
	int s, rc, i;
	uint32_t off;

	for (off = 0; off < size; off += read_size) {
		if (off < size - read_size)
			s = read_size;
		else
			s = size - off;
		for (i = 0; i < MAX_RETRY2; i++) {
			rc = llenv32_spi_exec(0, 0, SPI_FLASH_READ, addr + off, buf + off, s);
			if (!rc && !(SPI_REG_RISR(0) & 8))
				break;
			usleep(100 + SPI_REG_ICR(0));
		}
		if (rc)
			break;
	}
	return rc;
}

int spi_erase_subsect(uint32_t addr)
{
	int rc, i;

	for (i = 0; i < MAX_RETRY2; i++) {
		rc = llenv32_spi_wren(0, 0);
		if (!rc) {
			rc = llenv32_spi_exec(0, 0, SPI_FLASH_SSE, addr, 0, 0);
			if (!rc)
				rc = llenv32_spi_wait(0, 0);
		}
		if (!rc)
			break;
	}
	return rc;
}

int spi_write_subsector(uint32_t addr, uint8_t *buf)
{
	int rc, i;
	uint32_t off;

	for (off = 0; off < SSECTOR_SIZE; off += SPI_MAX_SIZE) {
		for (i = 0; i < MAX_RETRY2; i++) {
			rc = llenv32_spi_write(0, 0, addr + off, buf + off, SPI_MAX_SIZE);
			if (!rc)
				break;
		}
		if (rc)
			break;
	}
	return rc;
}

/*
 * returns:
 * 0 - verified at first attempt
 * 1 - written at first attempt (or verified at 2nd attempt if verify_only
 * >1 - completed job after n attempts
 * -1 - retry count exceeded
 */
int do_subsector(uint32_t addr, uint8_t *source, int verify_only)
{
	int OK = 0, rc;
	int cnt;
	uint8_t buf[SSECTOR_SIZE];

	for (cnt = 0; cnt < MAX_RETRY; cnt++) {
		rc = spi_read(addr, buf, SSECTOR_SIZE);
		if (rc)
			continue;
		OK = !memcmp(buf, source, SSECTOR_SIZE);
		if (!OK && (verify_only || cnt))
			dprintf(" data mismatch in sector %x\n", addr >> 12);
		if (OK || verify_only)
			break;
		// erase
		rc = spi_erase_subsect(addr);
		// write
		if (!rc)
			rc = spi_write_subsector(addr, source);
		/* ignore last error - we will verify anyway */
	}
	if (cnt < MAX_RETRY && OK)
		return cnt;
	else
		return -1;
}

int main(int argc, char *argv[])
{
	int mem_fd;
	int pg_size;
	uint32_t reg;
	unsigned char buf[20];
	int i, rc;
	struct stat st_buf;
	size_t img_size;
	int img_fd;
	char *img_buf;
	char *cfg_buf;
	uint32_t addr;
	int c;
	int verify_only = 0;
	int ok_sectors = 0, written_sectors = 0, bad_sectors = 0, retry_cnt = 0;

	while ((c = getopt(argc, argv, "dvs:")) != -1) {
		switch(c) {
		case 'd':
			debug_level = 1;
			break;
		case 'v':
			verify_only = 1;
			break;
		case 's':
			read_size = strtoul(optarg, NULL, 0);
			switch(read_size) {
			case 4:
			case 8:
			case 16:
			case 32:
			case 64:
			case 128:
			case 256:
				break;
			default:
				fprintf(stderr, "read_size must be power of 2 from 8 to 256\n");
				return 1;
			}
		default:
			fprintf(stderr, "Usage: %s <image_pathname> [-d][-v]\n", argv[0]);
		}
	}

	if ((argc - optind) != 1) {
		fprintf(stderr, "Usage: %s <image_pathname> [-d][-v]\n", argv[0]);
		return 1;
	}
	img_fd = open(argv[optind], O_RDONLY);
	if (img_fd < 0) {
		perror("open image");
		return 1;
	}
	if (fstat(img_fd, &st_buf)) {
		perror("Can't stat image");
		return 1;
	}
	img_size = st_buf.st_size;
	if (img_size != 16*1024*1024) {
		fprintf(stderr, "Wrong image size: %u (%x)\n", img_size, img_size);
		return 1;
	}
	img_buf = malloc(img_size);
	if (!img_buf) {
		fprintf(stderr, "No memory?\n");
		return 1;
	}
	i = read(img_fd, img_buf, img_size);
	if (i != img_size) {
		fprintf(stderr, "Can't read image? Got %d instead of %lu\n", i, img_size);
		return 1;
	}
	close(img_fd);

	mem_fd = open("/dev/mem", O_RDWR);
	if (mem_fd < 0) {
		perror("open /dev/mem");
		return 1;
	}
	pg_size = getpagesize();
	bootctl_map = mmap(NULL, pg_size, PROT_READ|PROT_WRITE, MAP_SHARED,
				mem_fd, BOOTCTL_BASE);
	close(mem_fd);
	if (bootctl_map == MAP_FAILED) {
		perror("mmap");
		return 1;
	}

	reg = *(uint32_t *)bootctl_map;
	reg |= 0x100;
	*(volatile uint32_t *)bootctl_map = reg;

	cfg_buf = llenv32_spi_store_cfg(0);
	llenv32_spi_init(0);
	rc = llenv32_spi_exec(0, 0, SPI_FLASH_RDID, 0, buf, sizeof(buf));
	if (rc) {
		fprintf(stderr, "spi_exec() failed with %d\n", rc);
	} else {
		dprintf("ID data:");
		for (i = 0; i < 20; i++)
			dprintf(" %02x", buf[i]);
		printf("\n");
	}

	if (verify_only)
		printf("Verify by 4K subsectors\n");
	else
		printf("Erase/Write/Verify by 4K subsectors\n");

	i = 0;
	printf("Sector %5d", i);
	fflush(stdout);
	for (addr = 0; addr < img_size; addr += SSECTOR_SIZE) {
		printf("\b\b\b\b\b%5d", i);
		fflush(stdout);
		rc = do_subsector(addr, img_buf + addr, verify_only);
		if (rc == 0)
			ok_sectors++;
		else if (rc == 1)
			written_sectors++;
		else if (rc > 0)
			retry_cnt += rc - 1;
		else { /* rc < 0 */
			bad_sectors++;
			if (!verify_only)
				break;
		}
		i++;
	}
	printf("\nDone\n");
	if (verify_only) {
		printf("%d sectors OK\n", ok_sectors + written_sectors);
	} else {
		printf("%d sectors written, %d - were ok, total_retries - %d\n",
			 written_sectors, ok_sectors, retry_cnt);
	}
	rc = !!bad_sectors;

	llenv32_spi_restore_cfg(0, cfg_buf);
	reg &= ~0x100;
	*(uint32_t *)bootctl_map = reg;
	munmap(bootctl_map, pg_size);

	return rc;
}

