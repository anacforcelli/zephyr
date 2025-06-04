/**
 * @file
 * @brief pulpino interrupt management code
 */
#include <zephyr/irq.h>
#include <soc.h>

void arch_irq_enable(unsigned int irq)
{
	unsigned int key;

	key = irq_lock();
	/*
	 * enable both IRQ and Event
	 * Event will allow system to wakeup upon an interrupt,
	 * if CPU was set to sleep
	 */
	PULP_IRQ_MASK_SET = (1 << irq);
	irq_unlock(key);
};

void arch_irq_disable(unsigned int irq)
{
	unsigned int key;

	key = irq_lock();
	PULP_IRQ_MASK_CLR = (1 << irq);
	irq_unlock(key);
};

int arch_irq_is_enabled(unsigned int irq)
{
	return !!(PULP_IRQ_MASK & (1 << irq));
}

#if defined(CONFIG_RISCV_SOC_INTERRUPT_INIT)
void soc_interrupt_init(void)
{
	/* ensure that all interrupts are disabled */
	(void)irq_lock();
	PULP_IRQ_MASK_CLR = 0xFFFFFFFF;
}
#endif
