#define DT_DRV_COMPAT caninos_puppy_gpio

#include <errno.h>
#include <soc.h>
#include <zephyr/drivers/gpio.h>
#include "zephyr/drivers/gpio/gpio_utils.h"

/* Puppy GPIO register-set structure */
struct __packed gpio_puppy_t
{
	uint32_t paddir;
	uint32_t gpioen;
	uint32_t padin;
	uint32_t padout;
	uint32_t padoutset;
	uint32_t padoutclr;
	uint32_t inten;
	uint32_t inttype_00_15;
	uint32_t inttype_16_31;
	uint32_t intstatus;
	uint32_t padcfg_00_07;
	uint32_t padcfg_08_15;
	uint32_t padcfg_16_23;
	uint32_t padcfg_24_31;
};

struct gpio_puppy_config
{
	uint32_t base;
	volatile struct gpio_puppy_t *gpio;
	struct gpio_driver_data common;
};

struct gpio_puppy_data
{
	sys_slist_t callbacks;
};

/**
 * @brief Configure pin
 *
 * @param dev Device structure
 * @param pin The pin number
 * @param flags Flags of pin or port
 *
 * @return 0 if successful, failed otherwise
 */
static int
gpio_puppy_configure(const struct device *dev,
					 gpio_pin_t pin,
					 gpio_flags_t flags)
{
	const struct gpio_puppy_config *config = dev->config;

	int pad_bit;
	int group;

	if (pin > 31)
		return -EINVAL;

	group = pin / 16;
	pad_bit = (pin % 16) * 2;

	/* Configure pin as gpio */
	/* GPIO pin cannot be input and output simultaneously */
	if ((flags & GPIO_INPUT) && (flags & GPIO_OUTPUT))
	{
		PULP_PADMUX(group) &= ~(PULP_PAD_GPIO << pad_bit);
		config->gpio->gpioen &= ~BIT(pin);
	}
	else if ((flags & GPIO_INPUT) || (flags & GPIO_OUTPUT))
	{
		PULP_PADMUX(group) |= (PULP_PAD_GPIO << pad_bit);
		config->gpio->gpioen |= BIT(pin);
	}

	/* Configure gpio direction */
	if (flags & GPIO_OUTPUT)
		config->gpio->paddir |= BIT(pin);
	else if (flags & GPIO_INPUT)
		config->gpio->paddir &= ~BIT(pin);
	else if (flags & GPIO_OUTPUT_INIT_HIGH)
		config->gpio->padout |= BIT(pin);
	else if (flags & GPIO_OUTPUT_INIT_LOW)
		config->gpio->padout &= ~BIT(pin);

	/*
	 * Configure interrupt if GPIO_INT is set.
	 * Here, we just configure the gpio interrupt behavior,
	 * we do not enable/disable interrupt for a particular
	 * gpio.
	 * Interrupt for a gpio is:
	 * 1) enabled only via a call to gpio_puppy_enable_callback.
	 * 2) disabled only via a call to gpio_puppy_disabled_callback.
	 */
	if (!(flags & GPIO_INT_ENABLE))
		return 0;

	/* Edge or Level triggered ? */
	if (!(flags & GPIO_INT_EDGE))
		return -ENOTSUP;

	/*
	 * Configure interrupt edge sensitivity.
	 * 2'b00 : falling edge
	 * 2'b01 : rising edge
	 * 2'b10 : both edges
	 * 2'b11 : Reserved
	 */
	int edge_val = ((flags >> 25) - 1) & 0x3;
	if (pin < 16)
	{
		config->gpio->inttype_00_15 &= ~pad_bit;
		config->gpio->inttype_00_15 |= edge_val << pad_bit;
	}
	else
	{
		config->gpio->inttype_16_31 &= ~(0x3 << pad_bit);
		config->gpio->inttype_16_31 |= edge_val << pad_bit;
	}

	return 0;
}

/**
 * @brief Read GPIO port
 *
 * @param dev Device struct
 * @param value Value to set (0 or 1)
 *
 * @return 0 if successful, failed otherwise
 */
static int gpio_puppy_port_get_raw(const struct device *dev,
								   uint32_t *value)
{
	const struct gpio_puppy_config *config = dev->config;

	*value = config->gpio->padin;

	return 0;
}

/**
 * @brief Set port according to mask and value
 *
 * @param dev Device struct
 * @param mask The mask for GPIO port
 * @param value Value of input pin(s)
 *
 * @return 0 if successful, failed otherwise
 */
static int gpio_puppy_port_set_masked_raw(const struct device *dev,
										  uint32_t mask,
										  uint32_t value)
{
	const struct gpio_puppy_config *config = dev->config;

	config->gpio->padout = (config->gpio->padout & ~mask) | (mask & value);

	return 0;
}

/**
 * @brief Set bits according to mask
 *
 * @param dev Device struct
 * @param mask The mask for GPIO port
 *
 * @return 0 if successful, failed otherwise
 */
static int gpio_puppy_port_set_bits_raw(const struct device *dev,
										uint32_t mask)
{
	const struct gpio_puppy_config *config = dev->config;

	config->gpio->padout |= mask;

	return 0;
}

/**
 * @brief Clear bits according to mask
 *
 * @param dev Device struct
 * @param mask The mask for GPIO port
 *
 * @return 0 if successful, failed otherwise
 */
static int gpio_puppy_port_clear_bits_raw(const struct device *dev,
										  uint32_t mask)
{
	const struct gpio_puppy_config *config = dev->config;

	config->gpio->padout &= ~mask;

	return 0;
}

/**
 * @brief Toggle bits according to mask
 *
 * @param dev Device struct
 * @param mask The mask for GPIO port
 *
 * @return 0 if successful, failed otherwise
 */
static int gpio_puppy_port_toggle_bits(const struct device *dev,
									   uint32_t mask)
{
	const struct gpio_puppy_config *config = dev->config;

	config->gpio->padout ^= mask;

	return 0;
}

/**
 * @brief Configure pin interrupt settings
 *
 * @param dev Device struct
 * @param pin GPIO pin
 * @param flags GPIO Interrupt flags
 *
 * @return 0 if successful, failed otherwise
 */
static int gpio_puppy_pin_interrupt_configure(const struct device *dev,
											  gpio_pin_t pin,
											  enum gpio_int_mode mode,
											  enum gpio_int_trig trig)
{
	const struct gpio_puppy_config *config = dev->config;
	uint32_t inttype = 0;

	if (mode == GPIO_INT_MODE_DISABLED)
	{
		config->gpio->inten &= ~BIT(pin);
	}
	else
	{
		if (mode == GPIO_INT_MODE_LEVEL)
		{
			return -ENOTSUP;
		}
		else
		{
			switch (trig)
			{
			case GPIO_INT_TRIG_LOW:
				inttype = 0;
				break;
			case GPIO_INT_TRIG_HIGH:
				inttype = 1;
				break;
			case GPIO_INT_TRIG_BOTH:
				inttype = 2;
				break;
			default:
				return -EINVAL;
			}

			if (pin < 16)
			{
				config->gpio->inttype_00_15 &= ~(0x3 << (pin * 2));
				config->gpio->inttype_00_15 |= (inttype << (pin * 2));
			}
			else
			{
				config->gpio->inttype_16_31 &= ~(0x3 << ((pin - 16) * 2));
				config->gpio->inttype_16_31 |= (inttype << ((pin - 16) * 2));
			}
		}
		config->gpio->inten |= BIT(pin);
	}

	return 0;
}

/**
 * @brief Configure callbacks
 *
 * @param dev Device struct
 * @param callback callback handle
 * @param set
 *
 * @return 0 if successful, failed otherwise
 */
static int gpio_puppy_manage_callback(const struct device *dev,
									  struct gpio_callback *callback,
									  bool set)
{
	struct gpio_puppy_data *data = dev->data;

	gpio_manage_callback(&data->callbacks, callback, set);

	return 0;
}

/**
 * @brief Handles GPIO ISR
 *
 * Perform basic initialization of a GPIO controller
 *
 * @param dev GPIO device structs
 *
 * @return 0
 */
static void gpio_puppy_isr(const struct device *dev)
{
	const struct gpio_puppy_config *config = dev->config;
	struct gpio_puppy_data *data = dev->data;

	/* Interrupts cleared automatically when intstatus register is read */
	uint32_t int_status = config->gpio->intstatus;

	gpio_fire_callbacks(&data->callbacks, dev, int_status);
}

static DEVICE_API(gpio, gpio_puppy_driver_api) = {
	.pin_configure = gpio_puppy_configure,
	.port_get_raw = gpio_puppy_port_get_raw,
	.port_set_masked_raw = gpio_puppy_port_set_masked_raw,
	.port_set_bits_raw = gpio_puppy_port_set_bits_raw,
	.port_clear_bits_raw = gpio_puppy_port_clear_bits_raw,
	.port_toggle_bits = gpio_puppy_port_toggle_bits,
	.pin_interrupt_configure = gpio_puppy_pin_interrupt_configure,
	.manage_callback = gpio_puppy_manage_callback,
};

#define GPIO_PUPPY_INIT(idx)                                             \
	static const struct gpio_puppy_config gpio_puppy_##idx##_config = {  \
		.base = DT_INST_REG_ADDR(idx),                                   \
		.gpio = (volatile struct gpio_puppy_t *)(DT_INST_REG_ADDR(idx)), \
	};                                                                   \
                                                                         \
	static struct gpio_puppy_data gpio_puppy_##idx##_data = {            \
		.callbacks = NULL,                                               \
	};                                                                   \
                                                                         \
	static int gpio_puppy_##idx##_init(const struct device *dev)         \
	{                                                                    \
		IRQ_CONNECT(DT_INST_IRQN(idx),                                   \
					0,                                                   \
					gpio_puppy_isr,                                      \
					DEVICE_DT_INST_GET(idx),                             \
					0);                                                  \
                                                                         \
		irq_enable(DT_INST_IRQN(idx));                                   \
		return 0;                                                        \
	}                                                                    \
                                                                         \
	DEVICE_DT_INST_DEFINE(idx,                                           \
						  gpio_puppy_##idx##_init,                       \
						  NULL,                                          \
						  &gpio_puppy_##idx##_data,                      \
						  &gpio_puppy_##idx##_config,                    \
						  PRE_KERNEL_1,                                  \
						  CONFIG_GPIO_INIT_PRIORITY,                     \
						  &gpio_puppy_driver_api);

DT_INST_FOREACH_STATUS_OKAY(GPIO_PUPPY_INIT)
