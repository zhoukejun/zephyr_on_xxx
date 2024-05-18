/*
 * Copyright (c) 2024 Kejun Zhou <zhoukejun@outlook.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/devicetree.h>

#include "sg2002_uart.h"


void riscv_lowputc(char ch)
{
  /* Wait until the TX data register is empty */

  while (!(sys_read32(UART0_LSR) & LSR_TRANS_HOLDING_REG_EMPTY))
    {
    }

  /* Then send the character */

  sys_write32(ch, UART0_THR);

}

/****************************************************************************
 * Name: sg2002_lowsetup
 *
 * Description:
 *   This performs basic initialization of the UART used for the serial
 *   console.  Its purpose is to get the console output available as soon
 *   as possible.
 *
 ****************************************************************************/

static int sg2002_uart0_init(void)
{
  /* Software Reset */
  sys_write32(0x7, UART0_SRR);

  /* Enable and configure the selected console device */
  sys_write32(0, UART0_DLH);
  sys_write32(0, UART0_MCR);
  sys_write32(UART_FCR_DEFVAL, UART0_FCR);
  sys_write32(UART_LCR_BKSE | UART_LCRVAL, UART0_LCR);

  /* Configure the UART 115200 Baud Rate */
  sys_write32(14 & 0xFF, UART0_THR);
  sys_write32(14 >> 8, UART0_DLH);

  /* 8n1 no parity */
  sys_write32(UART_LCRVAL, UART0_LCR);

  sys_write32(IER_DLH_RECV_DATA | IER_DLH_RECV_LINE_STA , UART0_IER);

  riscv_lowputc('\r');
  riscv_lowputc('\n');
  riscv_lowputc('s');
  riscv_lowputc('g');
  riscv_lowputc('2');
  riscv_lowputc('0');
  riscv_lowputc('0');
  riscv_lowputc('2');
  riscv_lowputc('\r');
  riscv_lowputc('\n');

  return 0;
}

int arch_printk_char_out(int c)
{
	if (c == '\n') {
		riscv_lowputc('\r');
	}

	riscv_lowputc((char)c);

	return 0;
}

SYS_INIT(sg2002_uart0_init, EARLY,
		CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
