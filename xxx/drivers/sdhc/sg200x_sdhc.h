/*
 * Copyright (c) 2024 Kejun Zhou <zhoukejun@outlook.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SDHC_SG200X_SDHC_H_
#define ZEPHYR_DRIVERS_SDHC_SG200X_SDHC_H_

#define EMMC_BASE			0x04300000
#define SD_BASE				0x04310000
#define SDIO_BASE			0x04320000


//SDMMC Registers Overview
//Name					Address		Offset Description
#define SDIF_SDMA_SADDR			0x000	// SDMA System Memory Address/ Argument2
#define SDIF_BLK_SIZE_AND_CNT		0x004	// Block Size and Block Count Register
#define SDIF_ARGUMENT			0x008	// Argument 1 Register
#define SDIF_XFER_MODE_AND_CMD		0x00c	// Transfer Mode and Command Register
#define SDIF_RESP31_0			0x010	// Response Bit 31-0 Regsiter
#define SDIF_RESP63_32			0x014	// Response Bit 63-32 Regsiter
#define SDIF_RESP95_64			0x018	// Response Bit 95-64 Regsiter
#define SDIF_RESP127_96			0x01c	// Response Bit 127-96 Regsiter
#define SDIF_BUF_DATA			0x020	// Buffer Data Port Register
#define SDIF_PRESENT_STS		0x024	// Present State Register
#define SDIF_HOST_CTL1_PWR_BG_WUP	0x028	// Host Control 1 , Power, Block Gap and Wakeup Register
#define SDIF_CLK_CTL_SWRST		0x02c	// Clock and Reset Control Register
#define SDIF_NORM_AND_ERR_INT_STS	0x030	// Normal and Error Interrupt Status Register
#define SDIF_NORM_AND_ERR_INT_STS_EN	0x034	// Normal and Error Interrupt Status Enable Register
#define SDIF_NORM_AND_ERR_INT_SIG_EN	0x038	// Normal and Error Interrupt Signal Enable Register
#define SDIF_AUTO_CMD_ERR_AND_HOST_CTL2	0x03c	// Auto CMD Error Status Register and Host Control 2 register
#define SDIF_CAPABILITIES1		0x040	// Capabilities 1 Register
#define SDIF_CAPABILITIES2		0x044	// Capabilities 2 Register
#define SDIF_FORCE_EVENT_ERR		0x050	// Force Event Register for Auto CMD Error Status
#define SDIF_ADMA_ERR_STS		0x054	// ADMA Error Status Register
#define SDIF_ADMA_SADDR_L		0x058	// ADMA System Address Register for low 32-bit
#define SDIF_ADMA_SADDR_H		0x05c	// ADMA System Address Register for high 32-bit
#define SDIF_PRESENT_VUL_INIT_DS	0x060	// Present Value Register for Initialization and Default Speed
#define SDIF_PRESENT_VUL_HS_SDR12	0x064	// Present Value Register for High-speed and SDR12
#define SDIF_PRESENT_VUL_SDR25_SDR50	0x068	// Present Value Register for SDR25 and SDR50
#define SDIF_PRESENT_VUL_SDR104_DDR50	0x06c	// Present Value Register for SDR104 and DDR50
#define SDIF_SLOT_INT_AND_HOST_VER	0x0fc	// Slot Interrupt Status and Host Controller Version Register
#define SDIF_EMMC_CTRL			0x200	// MSHC Control register
#define SDIF_EMMC_BOOT_CTL		0x204	// eMMC Boot Control Register
#define SDIF_CDET_TOUT_CTL		0x208	// Card Detect Control Register
#define SDIF_MBIU_CTRL			0x20c // MBIU Control register
#define SDIF_PHY_TX_RX_DLY		0x240   // PHY tx and rx delay line register
#define SDIF_PHY_DS_DLY			0x244   // PHY DS delay line register
#define SDIF_PHY_DLY_STS		0x248   // PHY delay line status register
#define SDIF_PHY_CONFIG			0x24c   // PHY Configuration register


//regiters alias
#define SDIF_PWR_CONTROL		0x29
#define SDIF_TOUT_CTRL			0x2e
#define SDIF_SOFTWARE_RESET		0x2f
#define SDIF_ERR_INT_STATUS_EN		0x36
#define SDIF_HOST_CONTROL2		0x3e



#define DEFAULT_DIV_SD_INIT_CLOCK	0x2

//alias
#define SDIF_RESPONSE_01		SDIF_RESP31_0
#define SDIF_COMMAND			(SDIF_XFER_MODE_AND_CMD + 0x2)

#define SDIF_GET_CMD(c)			((c >> 8) & 0x3f)





//====================================================================================
//PRESENT_STS, Offset Address: 0x024
//Name					   Description
#define SDIF_CMD_INHIBIT		0  // Command Inhibit (CMD). 1 : Cannot issue command; 0 : Can issue command using only CMD line
#define SDIF_CMD_INHIBIT_DAT		1  // Command Inhibit (DAT). 1 : Cannot issue command wihich used the DAT line; 0 : Can issue command using only DAT line
#define SDIF_DAT_LINE_ACTIVE		2  // DAT Line Active. This bit indicates whether one of the DAT line on SD Bus is in use. 1 : DAT Line Active; 0 : DAT Line Inactive
#define SDIF_RE_TUNE_REQ		3  //Re-Tuning Request. 1 : Sampling clock need re-tuning; 0 : Fixed or well tuned sampling clock
// bits 7:4, Reserved
#define SDIF_WR_XFER_ACTIVE		8  //Write Transfer Active. 1 : Transferring data; 0 : No valid data
#define SDIF_RD_XFER_ACTIVE		9  //Read Transfer Active. 1 : Transferring data; 0 : No valid data
#define SDIF_BUF_WR_ENABLE		10 //Buffer Write Enable. 1 : Enable; 0 : Disable
#define SDIF_BUF_RD_ENABLE		11 //Buffer Read Enable. 1 : Enable; 0 : Disable
// bits 15:12, Reserved
#define SDIF_CARD_INSERTED		16 //Card Inserted. 1 : Card Inserted; 0 : Reset or Debouncing or No card
#define SDIF_CARD_STABLE		17 //Card State Stable. 1 : No Card or Inserted; 0 : Reset or Debouncing
#define SDIF_CARD_CD_STS		18 //Card Detect Pin Level. 1 : Card Present ( SD_CD = 0); 0 : No Card Present (SD_CD = 1)
#define SDIF_CARD_WP_STS		19 //Write Protect Switch Pin Level. 1 : Write enabled ( SD_WP =0); 0 : Write protected (SD_WP = 1)
// bits 23:20, DAT[3:0] Line Signal Level
#define SDIF_DAT_3_0_STS_SHIFT		20
#define SDIF_DAT_3_0_STS_MASK		(0xF << 20)
#define SDIF_CMD_LINE_STS		24 //CMD Line Signal Level
// bits 31:25, Reserved




//CLK_CTL_SWRST, Offset Address: 0x02c
//Name					Bits	    Description
#define INT_CLK_EN			(0x1 << 0)  // Internal Clock Enable, 1 : Oscillate; 0 : Stop
#define INT_CLK_STABLE			(0x1 << 1)  // Internal Clock Stable. 1 : Ready; 0 : Not Ready
#define SD_CLK_EN			(0x1 << 2)  // SD Clock Enable for Card. 1 : Enable; 0 : Disable
#define PLL_EN				(0x1 << 3)  // PLL Enable 1 : Enable; 0 : Disable
//bits 5:4 Reserved
//bits 7:6 Upper Bits of SDCLK Frequency Select
#define UP_FREQ_SEL_SHIFT		6	    // Upper Bits of SDCLK Frequency Select Shift
#define UP_FREQ_SEL_MASK		(0x3 << 6)  // Bits 7:6. Upper Bits of SDCLK Frequency Select Mask
//bits 15:8. SDCLK Frequency Select
#define FREQ_SEL_SHIFT			8	    // SDCLK Frequency Select Shift 
#define FREQ_SEL_MASK			(0xFF << 8) //Bits 15:8. SDCLK Frequency Select Mask
//31:27	Reserved
///bits 19:16. Data Timeout Counter Value. 0x0 : TMCLK x 2^13; 0x1 : TMCLK x 2^14; ......; 0xe : TMCLK x 2^ 27; 0xf : Reserved
#define TOUT_CNT_SHIFT			16	    //Data Timeout Counter Value Shift 
#define TOUT_CNT_MASK			(0xF << 16) //Data Timeout Counter Value Mask
//bits 23:20 Reserved
#define SW_RST_ALL			(0x1 << 24) // Software Reset For All
#define SW_RST_CMD			(0x1 << 25) // Software Reset For CMD Line
#define SW_RST_DAT			(0x1 << 26) // Software Reset For DATA Line


//====================================================================================
//NORM_AND_ERR_INT_STS, Offset Address: 0x030
//Name					Bits 	    Description
#define SDIF_INT_CMD_CMPL		(0x1 << 0)  //Command Complete
#define SDIF_INT_XFER_CMPL		(0x1 << 1)  //Transfer Complete
#define SDIF_INT_BG_EVENT		(0x1 << 2)  //Block Gap Event
#define SDIF_INT_DMA_INT		(0x1 << 3)  //DMA Interrupt
#define SDIF_INT_BUF_WRDY		(0x1 << 4)  //Buffer Write Ready
#define SDIF_INT_BUF_RRDY		(0x1 << 5)  //Buffer Read Ready
#define SDIF_INT_CARD_INSERT_INT	(0x1 << 6)  //Card Insertion
#define SDIF_INT_CARD_REMOV_INT		(0x1 << 7)  //Card Removal
#define SDIF_INT_CARD_INT		(0x1 << 8)  //Card Interrupt
#define SDIF_INT_INT_A			(0x1 << 9)  //INT_A. This status is set if INT_A is enabled and INT_A pin is in low level
#define SDIF_INT_INT_B			(0x1 << 10) //INT_B. This status is set if INT_B is enabled and INT_B pin is in low level
#define SDIF_INT_INT_C			(0x1 << 11) // INT_C. This status is set if INT_C is enabled and INT_C pin is in low level
#define SDIF_INT_RE_TUNE_EVENT		(0x1 << 12) // Re-Tuning Event
//bits 13: Reserved
#define SDIF_INT_CQE_EVENT		(0x1 << 14) // Command Queuing Event
#define SDIF_INT_ERR_INT		(0x1 << 15) // Error Interrupt
#define SDIF_INT_CMD_TOUT_ERR		(0x1 << 16) // Command Timeout Error
#define SDIF_INT_CMD_CRC_ERR		(0x1 << 17) // Command CRC Error
#define SDIF_INT_CMD_ENDBIT_ERR		(0x1 << 18) // Command End Bit Error
#define SDIF_INT_CMD_IDX_ERR		(0x1 << 19) // Command Index Error
#define SDIF_INT_DAT_TOUT_ERR		(0x1 << 20) // Data Timeout Error
#define SDIF_INT_DAT_CRC_ERR		(0x1 << 21) // Data CRC Error
#define SDIF_INT_DAT_ENDBIT_ERR		(0x1 << 22) // Data End Bit Error
#define SDIF_INT_CURR_LIMIT_ERR		(0x1 << 23) // Current Limit Error
#define SDIF_INT_AUTO_CMD_ERR		(0x1 << 24) // Auto Command Error
#define SDIF_INT_ADMA_ERR		(0x1 << 25) // ADMA Error
#define SDIF_INT_TUNE_ERR		(0x1 << 26) // Tuning Error		
#define SDIF_INT_Reserved		(0x1 << 27)
#define SDIF_INT_BOOT_ACK_ERR		(0x1 << 28)
// bits 31:29 Reserved

#define SDIF_INT_CMD_MASK		(SDIF_INT_CMD_CMPL | SDIF_INT_CMD_TOUT_ERR | \
					SDIF_INT_CMD_CRC_ERR | SDIF_INT_CMD_ENDBIT_ERR | SDIF_INT_CMD_IDX_ERR)
#define SDIF_INT_DATA_MASK		(SDIF_INT_XFER_CMPL | SDIF_INT_BG_EVENT |SDIF_INT_DMA_INT | \
					SDIF_INT_BUF_WRDY | SDIF_INT_BUF_RRDY | \
					SDIF_INT_DAT_TOUT_ERR | SDIF_INT_DAT_CRC_ERR | \
					SDIF_INT_DAT_ENDBIT_ERR | SDIF_INT_ADMA_ERR)
#define SDIF_INT_BUS_POWER		SDIF_INT_CURR_LIMIT_ERR


#define SDIF_HOST_VER4_ENABLE		 BIT(12)


#define P_VENDOR_SPECIFIC_AREA		0xE8
#define P_VENDOR2_SPECIFIC_AREA		0xEA



//===================================================

#define MMC_CMD0			0
#define MMC_CMD1			1
#define MMC_CMD2			2
#define MMC_CMD3			3
#define MMC_CMD5			5
#define MMC_CMD6			6
#define MMC_CMD7			7
#define MMC_CMD8			8
#define MMC_CMD9			9
#define MMC_CMD11			11
#define MMC_CMD12			12
#define MMC_CMD13			13
#define MMC_CMD16			16
#define MMC_CMD17			17
#define MMC_CMD18			18
#define MMC_CMD19			19
#define MMC_CMD21			21
#define MMC_CMD23			23
#define MMC_CMD24			24
#define MMC_CMD25			25
#define MMC_CMD32			32
#define MMC_CMD33			33
#define MMC_CMD35			35
#define MMC_CMD36			36
#define MMC_CMD38			38
#define MMC_CMD52			52
#define MMC_CMD53			53
#define MMC_CMD55			55
#define SD_ACMD6			6
#define SD_ACMD13			13
#define SD_ACMD41			41
#define SD_ACMD42			42

static inline int mmc_op_multi(uint32_t opcode)
{
	return opcode == MMC_CMD25 || opcode == MMC_CMD18;
}


#define CLK_DIV_CRG_BASE		0x03002000

#define CLK_DIV_CLK_BYP_0		(CLK_DIV_CRG_BASE + 0x030)

#define CLK_DIV_DIV_CLK_EMMC		(CLK_DIV_CRG_BASE + 0x064)
#define CLK_DIV_DIV_CLK_SD0		(CLK_DIV_CRG_BASE + 0x070)
#define CLK_DIV_DIV_CLK_SD1		(CLK_DIV_CRG_BASE + 0x07c)

#define MMC_MAX_CLOCK_DIV_VALUE		0x40009


#endif /* ZEPHYR_DRIVERS_SDHC_SG200X_SDHC_H_*/
