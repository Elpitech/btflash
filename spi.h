/*
 * File:        spi.h
 * Author:      Alexey Malahov <Alexey.Malahov@baikalelectronics.com>
 *              Ramil Zaripov <Ramil.Zaripov@baikalelectronics.com>
 * Copyright (C) 2014 Baikal Electronics.
 * Description: spi library header
 */

#ifndef __LLENV32_SPI_H_
#define __LLENV32_SPI_H__


#include <linux/stddef.h>
//#include <configs/baikal_mips.h>
#include "spi_regs.h"
//#include "tools.h"


/* SPI FUNCTIONS */
void  llenv32_spi_init        (int port);
int   llenv32_spi_ping        (int port, int line);
int   llenv32_spi_wren        (int port, int line);
int   llenv32_spi_wait        (int port, int line);
int   llenv32_spi_erase       (int port, int line, int adr);
int   llenv32_spi_status      (int port, int line, void *status);
int   llenv32_spi_wren        (int port, int line);
int   llenv32_spi_wrde        (int port, int line);
int   llenv32_spi_read        (int port, int line, int adr, void *data, uint32_t size);
int   llenv32_spi_write       (int port, int line, int adr, void *data, uint32_t size);
int   llenv32_spi_read_sector (int port, int line, int adr, void *sector);
int   llenv32_spi_write_sector(int port, int line, int adr, void *sector);
void *llenv32_spi_store_cfg   (int port);
int   llenv32_spi_restore_cfg (int port, void *cfg);


#define BOOTCTL_BASE	0x1f040000
#define BOOT_SPI_OFF	0x100
#define CONFIG_SPI_FIFO_SIZE	8

extern void *bootctl_map;

/* memory description */
typedef struct {
    unsigned int mid;           /* manufacturer identification */
    unsigned int dev_type;      /* device type */
    unsigned int dev_cap;       /* device capacity */
    char    name[16];
} llenv32_spi_flashes_t;


/* debug config */
#define SPI_CHECK_SAVEENV   0       /* check all sectors after u-boot cmd:saveenv */
#define SPI_LOWLEVEL_DEBUG  0       /* show lowlewel spi debug print */

/* constants  */
#define SPI_SECTOR_SIZE     0x10000
#define SPI_MAX_SIZE        256
#define SPI_BAUDR_SCKDV     40      /* default baud rate and thresholds. */
#define JEDEC_DATA          20
#define MAX_SPI_TRIES       10000
#define SPI_FIFO_SIZE       (CONFIG_SPI_FIFO_SIZE/sizeof(uint16_t))
#define SPI_TH_TX           (SPI_FIFO_SIZE/sizeof(uint16_t))        /* fifo threshold */
#define SPI_TH_RX           (SPI_FIFO_SIZE/sizeof(uint16_t))
#define BOOT_SPI_PORT       0



/* BASE_REGISTER ADDRESS */
//#define SPI_PORT(p)     ((p == 0)?  CONFIG_SPI0_BASE : (CONFIG_SPI1_BASE + (p-1)*(CONFIG_SPI2_BASE - CONFIG_SPI1_BASE)))
#define SPI_PORT(p)	(bootctl_map + BOOT_SPI_OFF)

/* REGISTER ADDRESSES */
#define SPI_CTRLR0(p)           (SPI_PORT(p) + 0x00) /* Control Register 0  */
#define SPI_CTRLR1(p)           (SPI_PORT(p) + 0x04) /* Control Register 1  */
#define SPI_SSIENR(p)           (SPI_PORT(p) + 0x08) /* SSI Enable Register */
#define SPI_MWCR(p)             (SPI_PORT(p) + 0x0c) /* Microwire Control Register.  */
#define SPI_SER(p)              (SPI_PORT(p) + 0x10) /* Slave Enable Register.  */
#define SPI_BAUDR(p)            (SPI_PORT(p) + 0x14) /* Baud Rate Select.  */
#define SPI_TXFTLR(p)           (SPI_PORT(p) + 0x18) /* Transmit FIFO Threshold Level.  */
#define SPI_RXFTLR(p)           (SPI_PORT(p) + 0x1c) /* Receive FIFO Threshold level.  */
#define SPI_TXFLR(p)            (SPI_PORT(p) + 0x20) /* Transmit FIFO Level Register */
#define SPI_RXFLR(p)            (SPI_PORT(p) + 0x24) /* Receive FIFO Level Register */
#define SPI_SR(p)               (SPI_PORT(p) + 0x28) /* Status Register */
#define SPI_IMR(p)              (SPI_PORT(p) + 0x2c) /* Interrupt Mask Register */
#define SPI_IS(p)               (SPI_PORT(p) + 0x30) /* Interrupt Status Register */
#define SPI_RISR(p)             (SPI_PORT(p) + 0x34) /* Raw Interrupt StatusRegister */
#define SPI_TXOICR(p)           (SPI_PORT(p) + 0x38) /* Transmit FIFO Overflow Interrupt Clear Register */
#define SPI_RXOICR(p)           (SPI_PORT(p) + 0x3c) /* Receive FIFO Overflow Interrupt Clear Register */
#define SPI_RXUICR(p)           (SPI_PORT(p) + 0x40) /* Receive FIFO Underflow Interrupt Clear Register */
#define SPI_MSTICR(p)           (SPI_PORT(p) + 0x44) /* Multi-Master Interrupt Clear Register */
#define SPI_ICR(p)              (SPI_PORT(p) + 0x48) /* Interrupt Clear Register */
#define SPI_DMACR(p)            (SPI_PORT(p) + 0x4c) /* DMA Control Register.  */
#define SPI_DMATDLR(p)          (SPI_PORT(p) + 0x50) /* DMA Transmit Data Level.  */
#define SPI_DMARDLR(p)          (SPI_PORT(p) + 0x54) /* DMA Receive Data Level.  */
#define SPI_IDR(p)              (SPI_PORT(p) + 0x58) /* Identification Register.  */
#define SPI_SSI_VERSION_ID(p)   (SPI_PORT(p) + 0x5c) /* coreKit Version ID Register */
#define SPI_DR0(p)              (SPI_PORT(p) + 0x60) /* A 16-bit read/write buffer for the transmit/receive FIFOs. */
#define SPI_DR35(p)             (SPI_PORT(p) + 0xec) /* A 16-bit read/write buffer for the transmit/receive FIFOs. */
#define SPI_RX_SAMPLE_DLY(p)    (SPI_PORT(p) + 0xf0) /* RX Sample Delay. */
#define SPI_RSVD_0(p)           (SPI_PORT(p) + 0xf4) /* RSVD_0 - Reserved address location */
#define SPI_RSVD_1(p)           (SPI_PORT(p) + 0xf8) /* RSVD_1 - Reserved address location */
#define SPI_RSVD_2(p)           (SPI_PORT(p) + 0xfc) /* RSVD_2 - Reserved address location */


/* COMMANDS */
#define SPI_FLASH_RDID          0x9F        /* (0, 1-20) Read identification. */
#define SPI_FLASH_READ          0x03        /* (3, 1-∞ ) Read Data Bytes */
#define SPI_FLASH_WREN          0x06        /* (0, 0   ) Write Enable */
#define SPI_FLASH_WRDI          0x04        /* (0, 0   ) Write Disable */
#define SPI_FLASH_PP            0x02        /* (3, 256 ) Page Program */
#define SPI_FLASH_SSE           0x20        /* (3, 0   ) SubSector Erase */
#define SPI_FLASH_SE            0xD8        /* (3, 0   ) Sector Erase */
#define SPI_FLASH_RDSR          0x05        /* (0, 1   ) Read Status Register */
#define SPI_FLASH_WRSR          0x01        /* (0, 1-∞ ) Write Status Register */
#define SPI_FLASH_RDLR          0xE8        /* (3, 1-∞ ) Read Lock Register */
#define SPI_FLASH_WRLR          0xE5        /* (3, 1   ) Write Lock Register */
#define SPI_FLASH_RFSR          0x70        /* (1 to ∞)  Read Flag Status Register */
#define SPI_FLASH_CLFSR         0x50        /* (0) Clear Flag Status Register */
#define SPI_FLASH_BE            0xC7        /* (0) Bulk Erase */
#define SPI_FLASH_RSTEN         0x66        /* Reset Enable */
#define SPI_FLASH_RST           0x99        /* Reset Memory */
/* RDNVCR Read NV Configuration Register B5h (2) */
/* WRNVCR Write NV Configuration Register B1h (2) */
/* RDVCR Read Volatile Configuration Register 85h (1 to ∞) */
/* WRVCR Write Volatile Configuration Register 81h (1) */
/* RDVECR Read Volatile Enhanced Configuration Register 65h (1 to ∞) */
/* WRVECR Write Volatile Enhanced Configuration Register 61h (1) */



/* SPI_SSIENR */
#define SSIENR_SSI_DE           0
#define SSIENR_SSI_EN           1

/* CTRL 0 */
    /* Slave output enable. */
#define SPI_SLV_OE_OFFSET       10
#define SPI_SLV_MASK            (0x1 << SPI_SLV_OE_OFFSET)
#define SPI_SLV_OE              1

    /* Transfer mode bits */
#define SPI_TMOD_OFFSET         8
#define SPI_TMOD_MASK           (0x3 << SPI_TMOD_OFFSET)
#define SPI_TMOD_TR             0x0         /* transmit and receive */
#define SPI_TMOD_TO             0x1         /* transmit only */
#define SPI_TMOD_RO             0x2         /* receive only */
#define SPI_TMOD_EPROMREAD      0x3         /* EEPROM read */

    /* Serial Clock Polarity. */
#define SPI_SCPOL_OFFSET        7
#define SPI_SCPOL_MASK          (0x3 << SPI_SCPOL_OFFSET)
#define SPI_SCPOL_LOW           0
#define SPI_SCPOL_HIGH          1

    /* Serial Clock Phase. */
#define SPI_SCPH_OFFSET         6
#define SPI_SCPH_MASK           (0x3 << SPI_SCPH_OFFSET)
#define SPI_SCPH_MIDDLE         0
#define SPI_SCPH_START          1

    /* Frame format. */
#define SPI_FRF_OFFSET          4
#define SPI_FRF_MASK            (0x3 << SPI_FRF_OFFSET)
#define SPI_FRF_SPI             0x0
#define SPI_FRF_SSP             0x1
#define SPI_FRF_MICROWIRE       0x2
#define SPI_FRF_RESV            0x3

    /* Data Frame Size */
#define SPI_DFS_OFFSET          0
#define SPI_DFS_MASK            (0x3 << SPI_DFS_OFFSET)
#define SPI_DFS(x)              (x - 1)

/* Status register bits. */
#define SPI_SR_DCOL             (1 << 6)    /* Data Collision Error */
#define SPI_SR_TXE              (1 << 5)    /* Transmition Error. */
#define SPI_SR_RFF              (1 << 4)    /* Receive FIFO Full */
#define SPI_SR_RFNE             (1 << 3)    /* Receive FIFO Not Empty */
#define SPI_SR_TFE              (1 << 2)    /* Transmit FIFO Empty */
#define SPI_SR_TFNF             (1 << 1)    /* Transmit FIFO Not Full */
#define SPI_SR_BUSY             (1 << 0)    /* SSI Busy Flag. */

/* Define the SPI bits of the status register. */
#define SPI_FLASH_SR_WIP        (1 << 0)    /* Write In Progress */
#define SPI_FLASH_SR_WEL        (1 << 1)    /* Write Enable Latch */

/* Flag Status Register */
#define SPI_FLAG_PE             (1 << 7)    /* P/E Controller (not WIP) */
#define SPI_FLAG_ER_SUSPEND     (1 << 6)    /* Erase Suspend */
#define SPI_FLAG_ERASE          (1 << 5)    /* Erase */
#define SPI_FLAG_PROGRAM        (1 << 4)    /* Program */
#define SPI_FLAG_VPP            (1 << 3)    /* VPP */
#define SPI_FLAG_PR_SUSPEND     (1 << 2)    /* Program Suspend */
#define SPI_FLAG_PROTECTION     (1 << 1)    /* Protection */
#define SPI_FLAG_               (1 << 0)    /* RESERVED */

#endif /* __LLENV32_SPI_H_ */
