/*
 * File:        spi.c
 * Author:      Alexey Malahov <Alexey.Malahov@baikalelectronics.com>
 *              Ramil Zaripov <Ramil.Zaripov@baikalelectronics.com>
 * Copyright (C) 2014 Baikal Electronics.
 * Description: spi library
 */

#include <linux/stddef.h>       /* null, size, uint */
#include "spi.h"
#include "tools.h"

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
            llenv32_sleep(SPI_REG_BAUDR(port));         /* replace with mdelay, __udelay? */

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
                }
                /* timeout? ...*/
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
            /* printf("%s: SPI instruction %d is not provided\n", __FUNCTION__, cmd_op); */
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

