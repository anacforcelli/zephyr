/**
 * @file SoC configuration macros for the pulpino core
 */

#ifndef __PUPPY_SOC_H_
#define __PUPPY_SOC_H_

/* CSR Registers */
#define PULP_MESTATUS 0x7C0 /* Machine Exception Status Register */
#define PULP_LPSTART0 0x7B0 /* Hardware Loop 0 Start Register */
#define PULP_LPEND0   0x7B1 /* Hardware Loop 0 End Register */
#define PULP_LPCOUNT0 0x7B2 /* Hardware Loop 0 Count Register */
#define PULP_LPSTART1 0x7B4 /* Hardware Loop 1 Start Register */
#define PULP_LPEND1   0x7B5 /* Hardware Loop 1 End Register */
#define PULP_LPCOUNT1 0x7B6 /* Hardware Loop 1 Count Register */

/* IRQ numbers */
#define uDMA_IRQ           8  /* uDMA event */
#define PULP_TIMER_LO_IRQ  10 /* Timer LO event */
#define PULP_TIMER_HI_IRQ  11 /* Timer HI event */
#define PULP_GPIO_IRQ      15 /* GPIO event */
#define PULP_SOC_EVENT_IRQ 26 /* SOC event generator */

/* min value to consider as IRQ in MCAUSE register */
#define PULP_MIN_IRQ uDMA_IRQ

/* Exception numbers */
#define PULP_ECALL_EXP 11 /* ECALL Instruction */

/*
 * SOC-specific MSTATUS related info
 */
/* MSTATUS register to save/restore upon interrupt/exception/context switch */
#define SOC_MSTATUS_REG PULP_MESTATUS

#define SOC_MSTATUS_IEN (1 << 3) /* Machine Interrupt Enable bit */

/*
 * Default MSTATUS register value to restore from stack
 * upon scheduling a thread for the first time
 */
#define SOC_MSTATUS_DEF_RESTORE SOC_MSTATUS_IEN

/* SOC-specific MCAUSE bitfields */
#define SOC_MCAUSE_EXP_MASK  0x1F           /* Exception code Mask */
#define SOC_MCAUSE_ECALL_EXP PULP_ECALL_EXP /* ECALL exception number */

/* PAD configuration */
#include "soc_pad.h"

/* IRQ configuration */
#define PULP_IRQ_BASE 0x1A109000

#define PULP_IRQ_MASK_ADDR    (PULP_IRQ_BASE + 0x00) /* Mask (interrupt enable) */
#define PULP_IRQ_INT_ADDR     (PULP_IRQ_BASE + 0x0C) /* Pending interrupts */
#define PULP_IRQ_INT_CLR_ADDR (PULP_IRQ_BASE + 0x14) /* Pending interrupts */
#define PULP_IRQ_ACK_ADDR     (PULP_IRQ_BASE + 0x18)
#define PULP_IRQ_FIFO_DATA    (PULP_IRQ_BASE + 0x24)

/* uDMA configuration */
#include "soc_udma.h"

/* Event Unit */
#include "soc_event.h"

#ifndef _ASMLANGUAGE
#include <zephyr/irq.h>

/* Register Access MACRO */
#define PULP_REG(x) (*((volatile uint32_t *)(x)))

/* Interrupt Registers */
#define PULP_IRQ_MASK PULP_REG(PULP_IRQ_MASK_ADDR)
#define PULP_IRQ_MASK_SET                                                                          \
	PULP_REG(PULP_IRQ_MASK_ADDR + 0x04) /* This register sets bits on MASK register */
#define PULP_IRQ_MASK_CLR                                                                          \
	PULP_REG(PULP_IRQ_MASK_ADDR + 0x08) /* This register clears bits on MASK register */
#define PULP_IRQ_INT PULP_REG(PULP_IRQ_INT_ADDR)
#define PULP_IRQ_INT_SET                                                                           \
	PULP_REG(PULP_IRQ_INT_ADDR + 0x04) /* This register sets bits on INT register */
#define PULP_IRQ_INT_CLR                                                                           \
	PULP_REG(PULP_IRQ_INT_ADDR + 0x08) /* This register clears bits on INT register */
#define PULP_IRQ_ACK     PULP_REG(PULP_IRQ_ACK_ADDR)
#define PULP_IRQ_ACK_SET PULP_REG(PULP_IRQ_ACK_ADDR + 0x04)
#define PULP_IRQ_ACK_CLR PULP_REG(PULP_IRQ_ACK_ADDR + 0x08)

/* PAD MUX register */
#define PULP_PADMUX(group) PULP_REG(PULP_PAD_BASE + group * 0x4)

#define PULP_PAD_SPI  0
#define PULP_PAD_GPIO 1
#define PULP_PAD_MASK 1

/* PAD CFG register address */
#define PULP_PADCFG_REG(pad) PULP_REG(PULP_PAD_CFG_BASE + (pad / 4) * 0x4)

#if defined(CONFIG_RISCV_SOC_INTERRUPT_INIT)
void soc_interrupt_init(void);
#endif

#endif /* !_ASMLANGUAGE */

#endif /* __PUPPY_SOC_H_ */
