#ifndef __PUPPY_SOC_EVENT_H__
#define __PUPPY_SOC_EVENT_H__

#include <zephyr/sys/util.h>

#ifndef _ASMLANGUAGE

#define PULP_SOC_EU_ADDR 0x1A106000
#define SOC_EU_EVENT     0x00

#define SOC_NB_EVENT_REGS 8

#define SOC_FC_FIRST_MASK 0x4
#define SOC_PR_FIRST_MASK 0x44
#define SOC_CL_FIRST_MASK 0x24

/* move defines to timer header */
#define SOC_TIMER_SEL_HI           0x84
#define SOC_TIMER_SEL_EVT_WIDTH    8
#define SOC_TIMER_SEL_EVT_MASK     ((~0U) >> (32 - SOC_TIMER_SEL_EVT_WIDTH))
#define SOC_TIMER_SEL_EVT_SHIFT    0
#define SOC_TIMER_SEL_ENABLE_SHIFT 31

static inline void soc_eu_eventMask_set(unsigned int reg, unsigned int value)
{
	sys_write32(value, PULP_SOC_EU_ADDR + reg);
}

static inline unsigned int soc_eu_eventMask_get(unsigned int reg)
{
	return sys_read32(PULP_SOC_EU_ADDR + reg);
}

static inline void soc_eu_eventMask_reset(unsigned int first_reg)
{
	for (int i = 0; i < SOC_NB_EVENT_REGS; i++) {
		soc_eu_eventMask_set(first_reg + i * 4, 0xffffffff);
	}
}

static inline void soc_eu_eventMask_setEvent(int evt, unsigned int first_reg)
{
	unsigned int reg = first_reg + (evt / 32 * 4);
	evt = evt % 32;
	soc_eu_eventMask_set(reg, soc_eu_eventMask_get(reg) & ~(1 << evt));
}

static inline void soc_eu_eventMask_clearEvent(int evt, unsigned int first_reg)
{
	unsigned int reg = first_reg + (evt / 32 * 4);
	evt = evt % 32;
	soc_eu_eventMask_set(reg, soc_eu_eventMask_get(reg) | (1 << evt));
}

static inline void soc_eu_fcEventMask_setEvent(int evt)
{
	soc_eu_eventMask_setEvent(evt, SOC_FC_FIRST_MASK);
}

static inline void soc_eu_prEventMask_setEvent(int evt)
{
	soc_eu_eventMask_setEvent(evt, SOC_PR_FIRST_MASK);
}

static inline void soc_eu_clEventMask_setEvent(int clusterId, int evt)
{
	soc_eu_eventMask_setEvent(evt, SOC_CL_FIRST_MASK);
}

static inline void soc_eu_fcEventMask_clearEvent(int evt)
{
	soc_eu_eventMask_clearEvent(evt, SOC_FC_FIRST_MASK);
}

static inline void soc_eu_prEventMask_clearEvent(int evt)
{
	soc_eu_eventMask_clearEvent(evt, SOC_PR_FIRST_MASK);
}

static inline void soc_eu_clEventMask_clearEvent(int clusterId, int evt)
{
	soc_eu_eventMask_clearEvent(evt, SOC_CL_FIRST_MASK);
}

static inline void soc_eu_genEventMask(unsigned int mask)
{
	sys_write32(mask, PULP_SOC_EU_ADDR + SOC_EU_EVENT);
}

/** \brief Select event to be propagated to timer unit event input.
 *
 * \param timer_id Choice timer Low (1) or High (0).
 * \param mask Value on 8 bit to select the event id to be forwarded to the selected timer.
 */
static inline void soc_eu_selEventTimer(unsigned int timer_id, unsigned int mask)
{
	sys_write32((sys_read32(PULP_SOC_EU_ADDR + SOC_TIMER_SEL_HI + timer_id * 4) &
		     ~(SOC_TIMER_SEL_EVT_MASK << SOC_TIMER_SEL_EVT_SHIFT)) |
			    (mask & SOC_TIMER_SEL_EVT_MASK),
		    PULP_SOC_EU_ADDR + SOC_TIMER_SEL_HI + timer_id * 4);
}

/** \brief Activation of the event forward to timer feature.
 *
 * \param timer_id Choice timer Low (1) or High (0).
 * \param val Value to enable/disable event forwarding to selected timer.
 */
static inline void soc_eu_setEnableEventTimer(unsigned int timer_id, unsigned int val)
{
	sys_write32((sys_read32(PULP_SOC_EU_ADDR + SOC_TIMER_SEL_HI + timer_id * 4) &
		     ~(1 << SOC_TIMER_SEL_ENABLE_SHIFT)) |
			    val,
		    PULP_SOC_EU_ADDR + SOC_TIMER_SEL_HI + timer_id * 4);
}

int periph_get_event(int event);

void periph_wait_event(int event, int clear);

void periph_clear_event(int event);

#endif // _ASMLANGUAGE
#endif // __PUPPY_SOC_EVENT_H__
