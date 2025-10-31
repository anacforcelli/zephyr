#define DT_DRV_COMPAT caninos_puppy_timer

#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/device.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/irq.h>

/* Timer Ctrl Bitfields */
#define TIMER_CTRL_EN     (1 << 0)            /* Timer Enable Bit */
#define TIMER_CTRL_PRE(x) (((x) & 0x07) << 3) /* Prescaler Value */

typedef struct __packed {
	uint32_t cfg_lo;
	uint32_t cfg_hi;
	uint32_t cnt_lo;
	uint32_t cnt_hi;
	uint32_t cmp_lo;
	uint32_t cmp_hi;
	uint32_t start_lo;
	uint32_t start_hi;
	uint32_t reset_lo;
	uint32_t reset_hi;
} puppy_timer_t;

static volatile puppy_timer_t *timer;

static uint32_t accumulated_cycle_count;

static inline int sys_clock_hw_cycles_per_tick(void)
{
#ifdef CONFIG_SYS_CLOCK_EXISTS
	return sys_clock_hw_cycles_per_sec() / CONFIG_SYS_CLOCK_TICKS_PER_SEC;
#else
	return 1; // to avoid division by 0
#endif
}

static void puppy_timer_irq_handler(void *unused)
{
	ARG_UNUSED(unused);

	accumulated_cycle_count += sys_clock_hw_cycles_per_tick();

	sys_clock_announce(1);
}

#ifdef CONFIG_TICKLESS_IDLE
#error "Tickless idle not yet implemented for Puppy timer"
#endif

/**
 *
 * @brief Read the platform's timer hardware
 *
 * This routine returns the current time in terms of timer hardware clock
 * cycles.
 *
 * @return up counter of elapsed clock cycles
 */
uint32_t sys_clock_cycle_get_32(void)
{
	return accumulated_cycle_count + timer->cnt_lo;
}

uint32_t sys_clock_elapsed(void)
{
	return 0;
}

#define PUPPY_TIMER_INIT(idx)                                                                      \
	static int puppy_timer_##idx##_init(const struct device *dev)                              \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(idx), 0, puppy_timer_irq_handler,                         \
			    DEVICE_DT_INST_GET(idx), 0);                                           \
                                                                                                   \
		irq_enable(DT_INST_IRQN(idx));                                                     \
		timer = (puppy_timer_t *)DT_INST_REG_ADDR(idx);                                    \
		timer->reset_lo = 1U;                                                              \
		timer->cmp_lo = sys_clock_hw_cycles_per_tick() - 1;                                \
		timer->cfg_lo = 0x15;                                                              \
                                                                                                   \
		return 0;                                                                          \
	}                                                                                          \
	DEVICE_DT_INST_DEFINE(idx, puppy_timer_##idx##_init, NULL, NULL, NULL, PRE_KERNEL_2,       \
			      CONFIG_SYSTEM_CLOCK_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(PUPPY_TIMER_INIT);
