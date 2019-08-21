/*
 * File:        spi.c
 * Author:      Alexey Malahov <Alexey.Malahov@baikalelectronics.com>
 *              Ramil Zaripov <Ramil.Zaripov@baikalelectronics.com>
 * Copyright (C) 2014 Baikal Electronics.
 * Description: spi library
 */

//#include <linux/stddef.h>       /* null, size, uint */
#include <unistd.h>
#include <stdint.h>
#include <sys/types.h>
#include <stdlib.h>
#include <stdio.h>
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
        case 0x65:		/* Read Any Register */
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
	case SPI_FLASH_RSTEN:	/* Reset Enable */
	case SPI_FLASH_RST:	/* Reset */
            in      = cmd;
            in_len  = 1;
            out     = 0;
            out_len = 0;
            break;


        case SPI_FLASH_PP:      /* Page Program */
        case 0x71:		/* Write Any Register */
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
    uint8_t status;

    llenv32_spi_exec (port, line, SPI_FLASH_WREN, 0, 0, 0);
    llenv32_spi_exec (port, line, SPI_FLASH_RDSR, 0, &status, 1);

    return (status & SPI_FLASH_SR_WEL)? 0 : -1;
}



int llenv32_spi_wait (int port, int line)
{
#if 1
    int err, cnt = 0;
    uint8_t status;
    do {
        err = llenv32_spi_exec (port, line, SPI_FLASH_RDSR, 0, &status, 1);
        if(err)
            return err;
        cnt++;
    } while (status & SPI_FLASH_SR_WIP);
    if ((cnt == 1) && (status & 0x7c)) {
        dprintf("Wait status = %02x\n", status);
        return -1; /* Work In Progress never seen (invalid command?) */
    }
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

#include <errno.h>
#include <string.h>
#include <getopt.h>
#include <sys/fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>

#define SSECTOR_SIZE	0x1000	/* Small Sector - 4K */
#define SECTOR_SIZE	0x10000	/* Sector 64K */
#define MAX_RETRY	10	/* retries per subsector */
#define MAX_RETRY2	5	/* retries per read request */

int read_size = 256;	/* must be power of 2 and < SPI_MAX_SIZE */
int sector_size = SSECTOR_SIZE;	/* or SECTOR_SIZE */
uint8_t *vr_buf;		/* allocated of sector_size */
uint8_t erase_cmd = SPI_FLASH_SSE;	/* or SPI_FLASH_SE for 64K sectors */

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
			rc = llenv32_spi_exec(0, 0, erase_cmd, addr, 0, 0);
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

	for (off = 0; off < sector_size; off += SPI_MAX_SIZE) {
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

	for (cnt = 0; cnt < MAX_RETRY; cnt++) {
		rc = spi_read(addr, vr_buf, sector_size);
		if (rc)
			continue;
		OK = !memcmp(vr_buf, source, sector_size);
		if (!OK && (verify_only || cnt)) {
			dprintf(" data mismatch in sector %x\n", addr / sector_size);
			dprintf("%02x %02x %02x %02x\n", vr_buf[0], vr_buf[1], vr_buf[2], vr_buf[3]);
		}
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

int read_subsector(uint32_t addr, uint8_t *dst)
{
	int rc;
	int cnt;

	for (cnt = 0; cnt < MAX_RETRY; cnt++) {
		rc = spi_read(addr, vr_buf, sector_size);
		if (rc == 0) {
			memcpy(dst, vr_buf, sector_size);
			break;
		}
	}
	if (cnt < MAX_RETRY)
		return cnt;
	else
		return -1;
}

int set_uniform_sect()
{
	uint8_t buf[16], cr3v;
	int rc;

	rc = llenv32_spi_exec(0, 0, 0x65, 0x000004, buf, 2);
	if (rc)
		return rc;
	cr3v = buf[1];	/* buf[0] is dummy */
	dprintf("CR3V = %02x\n", cr3v);
	cr3v = cr3v & ~0x2;
	cr3v |= 8;
	buf[0] = cr3v;
	rc = llenv32_spi_wren(0, 0);
	if (rc)
		return rc;
	rc = llenv32_spi_exec(0, 0, 0x71, 0x000004, buf, 1);
	if (rc)
		return rc;
	usleep(100);
	rc = llenv32_spi_exec(0, 0, SPI_FLASH_RSTEN, 0, NULL, 0);
	if (rc)
		return rc;
	rc = llenv32_spi_exec(0, 0, SPI_FLASH_RST, 0, NULL, 0);
	if (rc)
		return rc;
	usleep(100);
	rc = llenv32_spi_exec(0, 0, 0x65, 0x800004, buf, 2);
	if (rc)
		return rc;
	if ((buf[1] & 0xe) != 8) {
		fprintf(stderr, "Can't set uniform sectors (CR3V = %02d)\n", buf[1]);
		rc = -2;
	}
	return rc;
}

int spi_reset()
{
	int i, rc;

	rc = llenv32_spi_exec(0, 0, SPI_FLASH_RSTEN, 0, NULL, 0);
	if (rc)
		return rc;
	for (i = 50; i > 0; i--)
		;
	rc = llenv32_spi_exec(0, 0, SPI_FLASH_RST, 0, NULL, 0);

	return rc;
}

int usage(char *progname)
{
	fprintf(stderr, "Usage: %s <image_pathname> [-h][-d][-v][-b][-r][-f<num>][-s<num>]\n", progname);
	fprintf(stderr, "\t-h - this help\n"
			"\t-d - debug output\n"
			"\t-v - verify only\n"
			"\t-b - \"batch\" mode (no output)\n"
			"\t-r - read image into specified file\n"
			"\t-f<sec> - start from <sec> 4K-sector (0-4095)\n"
			"\t-s<len> - read/verify size <len> bytes per request\n");

	return 1;
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
	uint8_t *img_buf;
	uint8_t *cfg_buf;
	uint32_t addr;
	int c;
	int verify_only = 0, batch = 0, do_read = 0;
	int ok_sectors = 0, written_sectors = 0, bad_sectors = 0, retry_cnt = 0;
	int start_sector = 0;

	while ((c = getopt(argc, argv, "bf:dvs:r")) != -1) {
		switch(c) {
		case 'd':
			debug_level = 1;
			break;
		case 'v':
			verify_only = 1;
			break;
		case 'b':
			batch = 1;
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
		case 'r':
			do_read = 1;
			break;
		case 'f':
			start_sector = strtoul(optarg, NULL, 0);
			if (start_sector < 0 || start_sector >= 4096) {
				fprintf(stderr, "Bad sector number %d\n", start_sector);
				return 1;
			}
			break;
		default:
			return usage(argv[0]);
		}
	}

	if ((argc - optind) != 1) {
		return usage(argv[0]);
	}
	if (do_read)
		img_fd = open(argv[optind], O_RDWR|O_CREAT|O_TRUNC, 0644);
	else
		img_fd = open(argv[optind], O_RDONLY);
	if (img_fd < 0) {
		perror("open image");
		return 1;
	}
	if (!do_read) {
		if (fstat(img_fd, &st_buf)) {
			perror("Can't stat image");
			return 1;
		}
		img_size = st_buf.st_size;
		if (img_size != 16*1024*1024) {
			fprintf(stderr, "Wrong image size: %lu (%lx)\n", img_size, img_size);
			return 1;
		}
	} else {
		img_size = 16*1024*1024;
	}
	img_buf = malloc(img_size);
	if (!img_buf) {
		fprintf(stderr, "No memory?\n");
		return 1;
	}
	if (!do_read) {
		i = read(img_fd, img_buf, img_size);
		if (i != img_size) {
			fprintf(stderr, "Can't read image? Got %d instead of %lu\n", i, img_size);
			return 1;
		}
		close(img_fd);
	}

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
	spi_reset();
	rc = llenv32_spi_exec(0, 0, SPI_FLASH_RDID, 0, buf, sizeof(buf));
	if (rc) {
		fprintf(stderr, "spi_exec() failed with %d\n", rc);
		llenv32_spi_restore_cfg(0, cfg_buf);
		return rc;
	} else {
		dprintf("ID data:");
		for (i = 0; i < 20; i++)
			dprintf(" %02x", buf[i]);
		printf("\n");
		if (buf[0] == 1 && buf[1] == 0x20 && buf[2] == 0x18) {
			/* Cypress flash */
			sector_size = SECTOR_SIZE;
			erase_cmd = SPI_FLASH_SE;
			rc = set_uniform_sect();
			if (rc) {
				fprintf(stderr, "Flash setup failed.\n");
				return rc;
			}
		}
	}

	vr_buf = malloc(sector_size);

	if (verify_only)
		printf("Verify by %dK subsectors\n", sector_size >> 10);
	else if (do_read)
		printf("Read by %dK subsectors\n", sector_size >> 10);
	else
		printf("Erase/Write/Verify by %dK subsectors\n", sector_size >> 10);

	i = 0;
	if (!batch) {
		printf("Sector %5d", i);
		fflush(stdout);
	}
	for (addr = start_sector * sector_size; addr < img_size; addr += sector_size) {
		if (!batch) {
			printf("\b\b\b\b\b%5d", i);
			fflush(stdout);
		}
		if (do_read)
			rc = read_subsector(addr, img_buf + addr);
		else
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
	if (do_read) {
		rc = write(img_fd, img_buf, img_size);
		if (rc == img_size)
			printf("%d sectors written\n", ok_sectors + written_sectors);
	} else if (verify_only) {
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

	if (rc)
		fprintf(stderr, "Errors encountered! Flash image is corrupt!\n");
	return rc;
}

