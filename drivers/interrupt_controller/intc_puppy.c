#define DT_DRV_COMPAT caninos_puppy_event_unit

#include <zephyr/kernel.h>
#include <soc.h>

static volatile uint32_t __soc_event_status[SOC_NB_EVENT_REGS];

/*
 * Update SoC peripheral events status, when IRQ (26) is asserted
 */
void puppy_event_irq_handler(void *unused)
{
	ARG_UNUSED(unused);
	uint32_t event_id = sys_read32(PULP_IRQ_FIFO_DATA);
	int index = event_id >> 5;
	__soc_event_status[index] |= (1 << (event_id & 0x1f));
}

/*
 * Initialize SoC events status and enable IRQ.
 */
int sys_event_init(void)
{
	soc_eu_eventMask_reset(SOC_FC_FIRST_MASK);

	for (int i = 0; i < SOC_NB_EVENT_REGS; i++) {
		__soc_event_status[i] = 0;
	}
	IRQ_CONNECT(PULP_SOC_EVENT_IRQ, 0, puppy_event_irq_handler, NULL, 0);
	irq_enable(PULP_SOC_EVENT_IRQ);

	return 0;
}

int periph_get_event(int event)
{
	unsigned int key = irq_lock();

	int index = event >> 5;
	event &= 0x1f;

	int result = (__soc_event_status[index] >> event) & 1;

	irq_unlock(key);

	return result;
}

void periph_wait_event(int event, int clear)
{
	unsigned int key = irq_lock();

	int index = event >> 5;
	event &= 0x1f;
	while (!((__soc_event_status[index] >> event) & 1)) {
		k_cpu_idle();
		irq_unlock(key);
		key = irq_lock();
	}

	if (clear) {
		__soc_event_status[index] &= ~(1 << event);
	}

	irq_unlock(key);
}

void periph_clear_event(int event)
{
	unsigned int key = irq_lock();

	int index = event >> 5;
	event &= 0x1f;

	__soc_event_status[index] &= ~(1 << event);

	irq_unlock(key);
}

SYS_INIT(sys_event_init, PRE_KERNEL_2, CONFIG_INTC_INIT_PRIORITY);
