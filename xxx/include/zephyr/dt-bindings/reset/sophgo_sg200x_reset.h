/*
 * Copyright (c) 2024 Kejun Zhou <zhoukejun@outlook.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_RESET_SOPHGO_SG200X_RESET_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_RESET_SOPHGO_SG200X_RESET_H_


//Reset Configuration Registers

//Address				Offset		Description
#define REG_SOFT_RSTN_0			0x000		//soft-reset ctrl register 0
#define REG_SOFT_RSTN_1			0x004		//soft-reset ctrl register 1
#define REG_SOFT_RSTN_2			0x008		//soft-reset ctrl register 2
#define REG_SOFT_RSTN_3			0x00c		//soft-reset ctrl register 3
#define REG_SOFT_CPUAC_RSTN		0x020		//CPU auto clear soft-reset ctrl register
#define REG_SOFT_CPU_RSTN		0x024		//CPU soft-reset ctrl register



//SOFT_RSTN_0, Offset Address: 0x000
//Name					Bits		Description
//bit 1:0 Reserved
#define REG_SOFT_RESET_X_DDR		(0x1 << 2)	//DDR system software reset (active low)
#define REG_SOFT_RESET_X_H264C		(0x1 << 3)	//H264 IP software reset (active low)
#define REG_SOFT_RESET_X_JPEG		(0x1 << 4)	//JPEG IP software reset (active low)
#define REG_SOFT_RESET_X_H265C		(0x1 << 5)	//H265 IP software reset (active low)
#define REG_SOFT_RESET_X_VIPSYS		(0x1 << 6)	//VIP system software reset (active low)
#define REG_SOFT_RESET_X_TDMA		(0x1 << 7)	//TPU_DMA IP software reset (active low)
#define REG_SOFT_RESET_X_TPU		(0x1 << 8)	//TPU IP software reset (active low)
#define REG_SOFT_RESET_X_TPUSYS		(0x1 << 9)	//TPU system software reset (active low)
//bit 10 Reserved
#define REG_SOFT_RESET_X_USB		(0x1 << 11)	//USB IP software reset (active low)
#define REG_SOFT_RESET_X_ETH0		(0x1 << 12)	//ETH0 IP software reset (active low)
#define REG_SOFT_RESET_X_ETH1		(0x1 << 13)	//ETH1 IP software reset (active low)
#define REG_SOFT_RESET_X_NAND		(0x1 << 14)	//NAND IP software reset (active low)
#define REG_SOFT_RESET_X_EMMC		(0x1 << 15)	//EMMC IP software reset (active low)
#define REG_SOFT_RESET_X_SD0		(0x1 << 16)	//SD0 IP software reset (active low)
//bit 17 Reserved
#define REG_SOFT_RESET_X_SDMA		(0x1 << 18)	//SDMA IP software reset (active low)
#define REG_SOFT_RESET_X_I2S0		(0x1 << 19)	//I2S0 IP software reset (active low)
#define REG_SOFT_RESET_X_I2S1		(0x1 << 20)	//I2S1 IP software reset (active low)
#define REG_SOFT_RESET_X_I2S2		(0x1 << 21)	//I2S2 IP software reset (active low)
#define REG_SOFT_RESET_X_I2S3		(0x1 << 22)     //I2S3 IP software reset (active low)
#define REG_SOFT_RESET_X_UART0		(0x1 << 23)     //UART0 IP software reset (active low)
#define REG_SOFT_RESET_X_UART1		(0x1 << 24)     //UART1 IP software reset (active low)
#define REG_SOFT_RESET_X_UART2		(0x1 << 25)     //UART2 IP software reset (active low)
#define REG_SOFT_RESET_X_UART3		(0x1 << 26)     //UART3 IP software reset (active low)
#define REG_SOFT_RESET_X_I2C0		(0x1 << 27)     //I2C0 IP software reset (active low)
#define REG_SOFT_RESET_X_I2C1		(0x1 << 28)     //I2C1 IP software reset (active low)
#define REG_SOFT_RESET_X_I2C2		(0x1 << 29)     //I2C2 IP software reset (active low)
#define REG_SOFT_RESET_X_I2C3		(0x1 << 30)     //I2C3 IP software reset (active low)
#define REG_SOFT_RESET_X_I2C4		(0x1 << 31)     //I2C4 IP software reset (active low)

//SOFT_RSTN_1, Offset Address: 0x004
//Name					Bits		Description
#define REG_SOFT_RESET_X_PWM0		(0x1 << 0)      //PWM0 IP software reset (active low)
#define REG_SOFT_RESET_X_PWM1		(0x1 << 1)      //PWM1 IP software reset (active low)
#define REG_SOFT_RESET_X_PWM2		(0x1 << 2)      //PWM2 IP software reset (active low)
#define REG_SOFT_RESET_X_PWM3		(0x1 << 3)      //PWM3 IP software reset (active low)
//bit 7:4 Reserved
#define REG_SOFT_RESET_X_SPI0		(0x1 << 8)      //SPI0 IP software reset (active low)
#define REG_SOFT_RESET_X_SPI1		(0x1 << 9)      //SPI1 IP software reset (active low)
#define REG_SOFT_RESET_X_SPI2		(0x1 << 10)     //SPI2 IP software reset (active low)
#define REG_SOFT_RESET_X_SPI3		(0x1 << 11)     //SPI3 IP software reset (active low)
#define REG_SOFT_RESET_X_GPIO0		(0x1 << 12)     //GPIO0 IP software reset (active low)
#define REG_SOFT_RESET_X_GPIO1		(0x1 << 13)     //GPIO1 IP software reset (active low)
#define REG_SOFT_RESET_X_GPIO2		(0x1 << 14)     //GPIO2 IP software reset (active low)
#define REG_SOFT_RESET_X_EFUSE		(0x1 << 15)     //EFUSE IP software reset (active low)
#define REG_SOFT_RESET_X_WDT		(0x1 << 16)     //WDT0 IP software reset (active low)
#define REG_SOFT_RESET_X_AHB_ROM	(0x1 << 17)     //ROM IP software reset (active low)
#define REG_SOFT_RESET_X_SPIC		(0x1 << 18)     //SPIC IP software reset (active low)
#define REG_SOFT_RESET_X_TEMPSEN	(0x1 << 19)     //TEMPSEN IP software reset (active low)
#define REG_SOFT_RESET_X_SARADC		(0x1 << 20)     //SARADC IP software reset (active low)
//bit 5:21 Reserved
#define REG_SOFT_RESET_X_COMBO_PHY0	(0x1 << 26)     //USB_PHY IP software reset (active low)
//bit 8:27 Reserved
#define REG_SOFT_RESET_X_SPI_NAND	(0x1 << 29)     //NAND IP software reset (active low)
#define REG_SOFT_RESET_X_SE		(0x1 << 30)     //SE IP software reset (active low)
//bit 31 Reserved


//SOFT_RSTN_2, Offset Address: 0x008
//Name					Bits		Description
//bit 9:0 Reserved
#define REG_SOFT_RESET_X_UART4		(0x1 << 10)	//UART4 IP software reset (active low)
#define REG_SOFT_RESET_X_GPIO3		(0x1 << 11)	//GPIO3 IP software reset (active low)
#define REG_SOFT_RESET_X_SYSTEM		(0x1 << 12)	//SYSTEM software reset (active low)
#define REG_SOFT_RESET_X_TIMER		(0x1 << 13)	//TIMER IP software reset (active low)
#define REG_SOFT_RESET_X_TIMER0		(0x1 << 14)	//TIMER0 IP software reset (active low)
#define REG_SOFT_RESET_X_TIMER1		(0x1 << 15)	//TIMER1 IP software reset (active low)
#define REG_SOFT_RESET_X_TIMER2		(0x1 << 16)	//TIMER2 IP software reset (active low)
#define REG_SOFT_RESET_X_TIMER3		(0x1 << 17)	//TIMER3 IP software reset (active low)
#define REG_SOFT_RESET_X_TIMER4		(0x1 << 18)	//TIMER4 IP software reset (active low)
#define REG_SOFT_RESET_X_TIMER5		(0x1 << 19)	//TIMER5 IP software reset (active low)
#define REG_SOFT_RESET_X_TIMER6		(0x1 << 20)	//TIMER6 IP software reset (active low)
#define REG_SOFT_RESET_X_TIMER7		(0x1 << 21)	//TIMER7 IP software reset (active low)
#define REG_SOFT_RESET_X_WGN0		(0x1 << 22)	//WGN0 IP software reset (active low)
#define REG_SOFT_RESET_X_WGN1		(0x1 << 23)	//WGN1 IP software reset (active low)
#define REG_SOFT_RESET_X_WGN2		(0x1 << 24)	//WGN2 IP software reset (active low)
#define REG_SOFT_RESET_X_KEYSCAN	(0x1 << 25)	//KEYSCAN IP software reset (active low)
//bit 26 Reserved
#define REG_SOFT_RESET_X_AUDDAC		(0x1 << 27)	//AUDDAC IP software reset (active low)
#define REG_SOFT_RESET_X_AUDDAC_APB	(0x1 << 28)	//AUDDAC APB software reset (active low)
#define REG_SOFT_RESET_X_AUDADC		(0x1 << 29)	//AUDADC IP software reset (active low)
//bit 30 Reserved
#define REG_SOFT_RESET_X_VCSYS		(0x1 << 31)	//VCSYS SYS software reset (active low)


//SOFT_RSTN_3, Offset Address: 0x00c
//Name					Bits		Description
#define REG_SOFT_RESET_X_ETHPHY		(0x1 << 0)	//ETHPHY IP software reset (active low)
#define REG_SOFT_RESET_X_ETHPHY_APB	(0x1 << 1)	//ETHPHY APB REG software reset (active low)
#define REG_SOFT_RESET_X_AUDSRC		(0x1 << 2)	//AUDSRC IP software reset (active low)
#define REG_SOFT _RESET_X_VIP_CAM0	(0x1 << 3)	//VIP CAM0 IP software reset (active low)
#define REG_SOFT_RESET_X_WDT1		(0x1 << 4)	//WDT1 IP software reset (active low)
#define REG_SOFT_RESET_X_WDT2		(0x1 << 5)	//WDT2 IP software reset (active low)
//bits 31:6 Reserved


//SOFT_CPUAC_RSTN, Offset Address: 0x020
//Name					Bits		Description
#define REG_AUTO_CLEAR_RESET_X_CPUCORE0	(0x1 << 0)	//CPUCORE0 auto_clear_reset (active low)
#define REG_AUTO_CLEAR_RESET_X_CPUCORE1 (0x1 << 1)      //CPUCORE1 auto_clear_reset (active low)
#define REG_AUTO_CLEAR_RESET_X_CPUCORE2 (0x1 << 2)      //CPUCORE2 auto_clear_reset (active low)
#define REG_AUTO_CLEAR_RESET_X_CPUCORE3 (0x1 << 3)      //CPUCORE3 auto_clear_reset (active low)
#define REG_AUTO_CLEAR_RESET_X_CPUSYS0  (0x1 << 4)      //CPUSYS0 auto_clear_reset (active low)
#define REG_AUTO_CLEAR_RESET_X_CPUSYS1  (0x1 << 5)      //CPUSYS1 auto_clear_reset (active low)
#define REG_AUTO_CLEAR_RESET_X_CPUSYS2  (0x1 << 6)      //CPUSYS2 auto_clear_reset (active low)
//bits 31:7 Reserved


//SOFT_CPU_RSTN, Offset Address: 0x024
//Name					Bits 		Description
#define REG_SOFT_RESET_X_CPUCORE0	(0x1 << 0)	//CPUCORE0 soft reset (active low)
#define REG_SOFT_RESET_X_CPUCORE1	(0x1 << 1)	//CPUCORE1 soft reset (active low)
#define REG_SOFT_RESET_X_CPUCORE2	(0x1 << 2)	//CPUCORE2 soft reset (active low)
#define REG_SOFT_RESET_X_CPUCORE3	(0x1 << 3)	//CPUCORE3 soft reset (active low)
#define REG_SOFT_RESET_X_CPUSYS0	(0x1 << 4)	//CPUSYS0 soft reset (active low)
#define REG_SOFT_RESET_X_CPUSYS1	(0x1 << 5)	//CPUSYS1 soft reset (active low)
#define REG_SOFT_RESET_X_CPUSYS2	(0x1 << 6)	//CPUSYS2 soft reset (active low)
//bits 31:7 Reserved



//SOFT_RSTN_0, Offset Address: 0x000
#define ID_RESET_X_DDR			(2)	//DDR system software reset (active low)
#define ID_RESET_X_H264C		(3)	//H264 IP software reset (active low)
#define ID_RESET_X_JPEG			(4)	//JPEG IP software reset (active low)
#define ID_RESET_X_H265C		(5)	//H265 IP software reset (active low)
#define ID_RESET_X_VIPSYS		(6)	//VIP system software reset (active low)
#define ID_RESET_X_TDMA			(7)	//TPU_DMA IP software reset (active low)
#define ID_RESET_X_TPU			(8)	//TPU IP software reset (active low)
#define ID_RESET_X_TPUSYS		(9)	//TPU system software reset (active low)
//bit 10 Reserved
#define ID_RESET_X_USB			(11)	//USB IP software reset (active low)
#define ID_RESET_X_ETH0			(12)	//ETH0 IP software reset (active low)
#define ID_RESET_X_ETH1			(13)	//ETH1 IP software reset (active low)
#define ID_RESET_X_NAND			(14)	//NAND IP software reset (active low)
#define ID_RESET_X_EMMC			(15)	//EMMC IP software reset (active low)
#define ID_RESET_X_SD0			(16)	//SD0 IP software reset (active low)
//bit 17 Reserved
#define ID_RESET_X_SDMA			(18)	//SDMA IP software reset (active low)
#define ID_RESET_X_I2S0			(19)	//I2S0 IP software reset (active low)
#define ID_RESET_X_I2S1			(20)	//I2S1 IP software reset (active low)
#define ID_RESET_X_I2S2			(21)	//I2S2 IP software reset (active low)
#define ID_RESET_X_I2S3			(22)	//I2S3 IP software reset (active low)
#define ID_RESET_X_UART0          	(23)	//UART0 IP software reset (active low)
#define ID_RESET_X_UART1		(24)	//UART1 IP software reset (active low)
#define ID_RESET_X_UART2		(25)	//UART2 IP software reset (active low)
#define ID_RESET_X_UART3		(26)	//UART3 IP software reset (active low)
#define ID_RESET_X_I2C0			(27)	//I2C0 IP software reset (active low)
#define ID_RESET_X_I2C1			(28)	//I2C1 IP software reset (active low)
#define ID_RESET_X_I2C2			(29)	//I2C2 IP software reset (active low)
#define ID_RESET_X_I2C3			(30)	//I2C3 IP software reset (active low)
#define ID_RESET_X_I2C4			(31)	//I2C4 IP software reset (active low)

//SOFT_RSTN_1, Offset Address: 0x004
//Name					Bits		Description
#define ID_RESET_X_PWM0			(32 + 0)      //PWM0 IP software reset (active low)
#define ID_RESET_X_PWM1			(32 + 1)      //PWM1 IP software reset (active low)
#define ID_RESET_X_PWM2			(32 + 2)      //PWM2 IP software reset (active low)
#define ID_RESET_X_PWM3			(32 + 3)      //PWM3 IP software reset (active low)
//bit 7:4 Reserved
#define ID_RESET_X_SPI0			(32 + 8)      //SPI0 IP software reset (active low)
#define ID_RESET_X_SPI1			(32 + 9)      //SPI1 IP software reset (active low)
#define ID_RESET_X_SPI2			(32 + 10)     //SPI2 IP software reset (active low)
#define ID_RESET_X_SPI3			(32 + 11)     //SPI3 IP software reset (active low)
#define ID_RESET_X_GPIO0		(32 + 12)     //GPIO0 IP software reset (active low)
#define ID_RESET_X_GPIO1		(32 + 13)     //GPIO1 IP software reset (active low)
#define ID_RESET_X_GPIO2		(32 + 14)     //GPIO2 IP software reset (active low)
#define ID_RESET_X_EFUSE		(32 + 15)     //EFUSE IP software reset (active low)
#define ID_RESET_X_WDT			(32 + 16)     //WDT0 IP software reset (active low)
#define ID_RESET_X_AHB_ROM		(32 + 17)     //ROM IP software reset (active low)
#define ID_RESET_X_SPIC			(32 + 18)     //SPIC IP software reset (active low)
#define ID_RESET_X_TEMPSEN		(32 + 19)     //TEMPSEN IP software reset (active low)
#define ID_RESET_X_SARADC		(32 + 20)     //SARADC IP software reset (active low)
//bit 5:21 Reserved
#define ID_RESET_X_COMBO_PHY0		(32 + 26)     //USB_PHY IP software reset (active low)
//bit 8:27 Reserved
#define ID_RESET_X_SPI_NAND		(32 + 29)     //NAND IP software reset (active low)
#define ID_RESET_X_SE			(32 + 30)     //SE IP software reset (active low)
//bit 31 Reserved


//SOFT_RSTN_2, Offset Address: 0x008
//Name					Bits		Description
//bit 9:0 Reserved
#define ID_RESET_X_UART4		(32 * 2 + 10)	//UART4 IP software reset (active low)
#define ID_RESET_X_GPIO3		(32 * 2 + 11)	//GPIO3 IP software reset (active low)
#define ID_RESET_X_SYSTEM		(32 * 2 + 12)	//SYSTEM software reset (active low)
#define ID_RESET_X_TIMER		(32 * 2 + 13)	//TIMER IP software reset (active low)
#define ID_RESET_X_TIMER0		(32 * 2 + 14)	//TIMER0 IP software reset (active low)
#define ID_RESET_X_TIMER1		(32 * 2 + 15)	//TIMER1 IP software reset (active low)
#define ID_RESET_X_TIMER2		(32 * 2 + 16)	//TIMER2 IP software reset (active low)
#define ID_RESET_X_TIMER3		(32 * 2 + 17)	//TIMER3 IP software reset (active low)
#define ID_RESET_X_TIMER4		(32 * 2 + 18)	//TIMER4 IP software reset (active low)
#define ID_RESET_X_TIMER5		(32 * 2 + 19)	//TIMER5 IP software reset (active low)
#define ID_RESET_X_TIMER6		(32 * 2 + 20)	//TIMER6 IP software reset (active low)
#define ID_RESET_X_TIMER7		(32 * 2 + 21)	//TIMER7 IP software reset (active low)
#define ID_RESET_X_WGN0			(32 * 2 + 22)	//WGN0 IP software reset (active low)
#define ID_RESET_X_WGN1			(32 * 2 + 23)	//WGN1 IP software reset (active low)
#define ID_RESET_X_WGN2			(32 * 2 + 24)	//WGN2 IP software reset (active low)
#define ID_RESET_X_KEYSCAN		(32 * 2 + 25)	//KEYSCAN IP software reset (active low)
//bit 26 Reserved
#define ID_RESET_X_AUDDAC		(32 * 2 + 27)	//AUDDAC IP software reset (active low)
#define ID_RESET_X_AUDDAC_APB		(32 * 2 + 28)	//AUDDAC APB software reset (active low)
#define ID_RESET_X_AUDADC		(32 * 2 + 29)	//AUDADC IP software reset (active low)
//bit 30 Reserved
#define ID_RESET_X_VCSYS		(32 * 2 + 31)	//VCSYS SYS software reset (active low)


//SOFT_RSTN_3, Offset Address: 0x00c
//Name					Bits		Description
#define ID_RESET_X_ETHPHY		(32 * 3 + 0)	//ETHPHY IP software reset (active low)
#define ID_RESET_X_ETHPHY_APB		(32 * 3 + 1)	//ETHPHY APB REG software reset (active low)
#define ID_RESET_X_AUDSRC		(32 * 3 + 2)	//AUDSRC IP software reset (active low)
#define ID_RESET_X_VIP_CAM0		(32 * 3 + 3)	//VIP CAM0 IP software reset (active low)
#define ID_RESET_X_WDT1			(32 * 3 + 4)	//WDT1 IP software reset (active low)
#define ID_RESET_X_WDT2			(32 * 3 + 5)	//WDT2 IP software reset (active low)


#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_RESET_SOPHGO_SG200X_RESET_H_ */
