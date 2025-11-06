#ifndef _ASMLANGUAGE

#include "soc_event.h"
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <soc.h>
#include <sys/errno.h>

/* All callbacks will fire until the callback corresponding to the event number is fired */
static event_callback_s *callbacks[SOC_MAX_NUM_EVENT_CALLBACKS];

/*
 * Fire registered event callbacks when IRQ (26) is asserted
 */
void puppy_event_irq_handler(void *unused)
{
	ARG_UNUSED(unused);
	uint32_t event_num = sys_read32(PULP_IRQ_FIFO_DATA_GET_REG);
	unsigned int key = irq_lock();
	event_num = event_num >> 5;
	for (int i = 0; i < SOC_MAX_NUM_EVENT_CALLBACKS; i++) {
		/* For each event that exists, execute the event code and check if it
		 * returns zero (success)*/
		if (callbacks[i] != NULL) {
			if (!callbacks[i]->callback(callbacks[i]->dev, event_num)) {
				break;
			}
		}
	}

	irq_unlock(key);
}

int puppy_event_register_callback(int event_num, event_callback_s *callback)
{
	for (int i = 0; i < SOC_MAX_NUM_EVENT_CALLBACKS; i++) {
		if (callbacks[i] == NULL) {
			callbacks[i] = callback;
			return 0;
		}
	}
	return -ENOBUFS;
}

void puppy_event_clr_callback(int event_num, event_callback_s *callback)
{
	for (int i = 0; i < SOC_MAX_NUM_EVENT_CALLBACKS; i++) {
		if (callbacks[i] == callback) {
			callbacks[i] = NULL;
		}
	}
}

void puppy_event_mask(int event_num)
{
	int event_bit = event_num % 32;
	int event_reg = event_num / SOC_NB_EVENT_REGS;

	PULP_REG(PULP_EU_FC_MASK0_REG + 4 * event_reg) &= ~BIT(event_bit);
}

void puppy_event_set(int event_num)
{
	int event_bit = event_num % 32;
	int event_reg = event_num / SOC_NB_EVENT_REGS;

	PULP_REG(PULP_EU_FC_MASK0_REG + 4 * event_reg) |= BIT(event_bit);
}

/* Initialize SoC events status and enable IRQ. */
int sys_event_init(void)
{
	/* As a precaution, NULL all callback fields */
	for (int i = 0; i < SOC_MAX_NUM_EVENT_CALLBACKS; i++) {
		callbacks[i] = NULL;
	}

	/* Zero All Event Masks */
	for (int i = 0; i < SOC_NB_EVENT_REGS; i++) {
		PULP_REG(PULP_SOC_EU_ADDR + 4 * i) = 0;
	}

	IRQ_CONNECT(PULP_SOC_EVENT_IRQ, 0, puppy_event_irq_handler, NULL, 0);
	irq_enable(PULP_SOC_EVENT_IRQ);

	return 0;
}

SYS_INIT(sys_event_init, PRE_KERNEL_2, CONFIG_INTC_INIT_PRIORITY);

#endif /* _ASMLANGUAGE */
