/*
 * File:        spi_regs.h
 * Author:      Ramil Zaripov <Ramil.Zaripov@baikalelectronics.com>
 * Copyright (C) 2014 Baikal Electronics.
 * Description: spi controller registers
 */


#ifndef __LLENV32_SPI_REG_H_
#define __LLENV32_SPI_REG_H__

#include <linux/types.h>     /* uint16_t */

/* ctrl0 */
typedef struct {
    uint32_t  dfs    :4;    /* data frame size */
    uint32_t  frf    :2;    /* frame format (0-spi, 1-ssp, 2-micro, 3-reserved) */
    uint32_t  scph   :1;    /* clk phase */
    uint32_t  scpol  :1;    /* clk polarity */
    uint32_t  tmod   :2;    /* transfer mode (0-tx|rx, 1-tx, 2-rx, 3-eeprom) */
    uint32_t  slv_oe :1;    /* (ignore) slave output enable */
    uint32_t  srl    :1;    /* (ignore) shift register loop */
    uint32_t  cfs    :4;    /* (ignore) control frame size */
    uint32_t  _      :16;
} spi_ctrlr0_t;

typedef uint32_t spi_ctrlr1_t;
typedef uint32_t spi_ssienr_t;
typedef uint32_t spi_mwcr_t;
typedef uint32_t spi_ser_t;
typedef uint32_t spi_baudr_t;
typedef uint32_t spi_txftlr_t;
typedef uint32_t spi_rxftlr_t;
typedef uint32_t spi_txflr_t;
typedef uint32_t spi_rxflr_t;

/* status register */
typedef struct {
    uint32_t busy  :1;      /* busy */
    uint32_t tfnf  :1;      /* transmite fifo not full */
    uint32_t tfe   :1;      /* transmite fifo empty */
    uint32_t rfne  :1;      /* recieve fifo not empty */
    uint32_t rff   :1;      /* recieve fifo full */
    uint32_t txe   :1;      /* transmission error */
    uint32_t dcol  :1;      /* data collision error */
    uint32_t _     :2;
    uint32_t __    :8*3;
} spi_sr_t;

typedef uint32_t spi_imr_t;
typedef uint32_t spi_is_t;
typedef uint32_t spi_risr_t;
typedef uint32_t spi_txoicr_t;
typedef uint32_t spi_rxoicr_t;
typedef uint32_t spi_rxuicr_t;
typedef uint32_t spi_msticr_t;
typedef uint32_t spi_icr_t;
typedef uint32_t spi_dmacr_t;
typedef uint32_t spi_dmatdlr_t;
typedef uint32_t spi_dmardlr_t;
typedef uint32_t spi_idr_t;
typedef uint32_t spi_ssi_version_id_t;
typedef uint32_t spi_dr0_t;
typedef uint32_t spi_dr35_t;
typedef uint32_t spi_rx_sample_dly_t;
typedef uint32_t spi_rsvd_0_t;
typedef uint32_t spi_rsvd_1_t;
typedef uint32_t spi_rsvd_2_t;


/* MACROS TO ACCESS REGISTERS */
#define SPI_REG_CTRLR0(p)           (*(volatile spi_ctrlr0_t         *) SPI_CTRLR0(p)           )
#define SPI_REG_CTRLR1(p)           (*(volatile spi_ctrlr1_t         *) SPI_CTRLR1(p)           )
#define SPI_REG_SSIENR(p)           (*(volatile spi_ssienr_t         *) SPI_SSIENR(p)           )
#define SPI_REG_MWCR(p)             (*(volatile spi_mwcr_t           *) SPI_MWCR(p)             )
#define SPI_REG_SER(p)              (*(volatile spi_ser_t            *) SPI_SER(p)              )
#define SPI_REG_BAUDR(p)            (*(volatile spi_baudr_t          *) SPI_BAUDR(p)            )
#define SPI_REG_TXFTLR(p)           (*(volatile spi_txftlr_t         *) SPI_TXFTLR(p)           )
#define SPI_REG_RXFTLR(p)           (*(volatile spi_rxftlr_t         *) SPI_RXFTLR(p)           )
#define SPI_REG_TXFLR(p)            (*(volatile spi_txflr_t          *) SPI_TXFLR(p)            )
#define SPI_REG_RXFLR(p)            (*(volatile spi_rxflr_t          *) SPI_RXFLR(p)            )
#define SPI_REG_SR(p)               (*(volatile spi_sr_t             *) SPI_SR(p)               )
#define SPI_REG_IMR(p)              (*(volatile spi_imr_t            *) SPI_IMR(p)              )
#define SPI_REG_IS(p)               (*(volatile spi_is_t             *) SPI_IS(p)               )
#define SPI_REG_RISR(p)             (*(volatile spi_risr_t           *) SPI_RISR(p)             )
#define SPI_REG_TXOICR(p)           (*(volatile spi_txoicr_t         *) SPI_TXOICR(p)           )
#define SPI_REG_RXOICR(p)           (*(volatile spi_rxoicr_t         *) SPI_RXOICR(p)           )
#define SPI_REG_RXUICR(p)           (*(volatile spi_rxuicr_t         *) SPI_RXUICR(p)           )
#define SPI_REG_MSTICR(p)           (*(volatile spi_msticr_t         *) SPI_MSTICR(p)           )
#define SPI_REG_ICR(p)              (*(volatile spi_icr_t            *) SPI_ICR(p)              )
#define SPI_REG_DMACR(p)            (*(volatile spi_dmacr_t          *) SPI_DMACR(p)            )
#define SPI_REG_DMATDLR(p)          (*(volatile spi_dmatdlr_t        *) SPI_DMATDLR(p)          )
#define SPI_REG_DMARDLR(p)          (*(volatile spi_dmardlr_t        *) SPI_DMARDLR(p)          )
#define SPI_REG_IDR(p)              (*(volatile spi_idr_t            *) SPI_IDR(p)              )
#define SPI_REG_SSI_VERSION_ID(p)   (*(volatile spi_ssi_version_id_t *) SPI_SSI_VERSION_ID(p)   )
#define SPI_REG_DR(p)               (*(volatile spi_dr0_t            *) SPI_DR0(p)              )
#define SPI_REG_RX_SAMPLE_DLY(p)    (*(volatile spi_rx_sample_dly_t  *) SPI_RX_SAMPLE_DLY(p)    )
#define SPI_REG_RSVD_0(p)           (*(volatile spi_rsvd_0_t         *) SPI_RSVD_0(p)           )
#define SPI_REG_RSVD_1(p)           (*(volatile spi_rsvd_1_t         *) SPI_RSVD_1(p)           )
#define SPI_REG_RSVD_2(p)           (*(volatile spi_rsvd_2_t         *) SPI_RSVD_2(p)           )


#endif