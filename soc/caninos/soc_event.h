#ifndef __PUPPY_SOC_EVENT_H__
#define __PUPPY_SOC_EVENT_H__

#ifndef _ASMLANGUAGE

#include <zephyr/device.h>

/* Event Unit */

#define PULP_SOC_EU_ADDR 0x1A106000

#define PULP_EU_SW_EVENT_REG (PULP_SOC_EU_ADDR + 0x00)
#define PULP_EU_FC_MASK0_REG (PULP_SOC_EU_ADDR + 0x04)
#define PULP_EU_FC_MASK1_REG (PULP_SOC_EU_ADDR + 0x08)
#define PULP_EU_FC_MASK2_REG (PULP_SOC_EU_ADDR + 0x0C)
#define PULP_EU_FC_MASK3_REG (PULP_SOC_EU_ADDR + 0x10)
#define PULP_EU_FC_MASK4_REG (PULP_SOC_EU_ADDR + 0x14)
#define PULP_EU_FC_MASK5_REG (PULP_SOC_EU_ADDR + 0x18)
#define PULP_EU_FC_MASK6_REG (PULP_SOC_EU_ADDR + 0x1C)
#define PULP_EU_FC_MASK7_REG (PULP_SOC_EU_ADDR + 0x20)

#define SOC_NB_EVENT_REGS 8

#define SOC_MAX_NUM_EVENT_CALLBACKS 32

/* move defines to timer header */
#define SOC_TIMER_SEL_HI           0x84
#define SOC_TIMER_SEL_EVT_WIDTH    8
#define SOC_TIMER_SEL_EVT_MASK     ((~0U) >> (32 - SOC_TIMER_SEL_EVT_WIDTH))
#define SOC_TIMER_SEL_EVT_SHIFT    0
#define SOC_TIMER_SEL_ENABLE_SHIFT 31

/* Each in-driver Event Callback receives the event number of the event fired */
typedef int (*event_callback_t)(const struct device *dev, int event_num);

typedef struct {
	const struct device *dev;
	event_callback_t callback;
} event_callback_s;

extern int puppy_event_register_callback(int event_num, event_callback_s *callback);

extern void puppy_event_clr_callback(int event_num, event_callback_s *callback);

extern void puppy_event_mask(int event_num);

extern void puppy_event_set(int event_num);

#endif // _ASMLANGUAGE
#endif // __PUPPY_SOC_EVENT_H__
