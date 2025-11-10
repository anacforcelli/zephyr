#define DT_DRV_COMPAT caninos_puppy_uart

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <soc.h>
#include "uart_puppy.h"

#if CONFIG_UART_ASYNC_API
struct uart_puppy_async_data {
	uart_callback_t cb;
	void *user_data;
};
#endif

struct uart_puppy_data {
	uint32_t base;
	int id;
	struct k_sem tx_lock;
	struct k_sem rx_lock;
	struct uart_config uart_config;
#if CONFIG_UART_ASYNC_API
	struct uart_puppy_async_data async;
#endif
};

void tx_evt(unsigned int event_num, void *dev_ptr)
{
	const struct device* dev = (struct device*)dev_ptr;
	struct uart_puppy_data *data = dev->data;

	if (event_num == ARCHI_UDMA_UART0_TX_EVT(0) || event_num == ARCHI_UDMA_UART1_TX_EVT(0))
		k_sem_give(&data->tx_lock);
}

void rx_evt(unsigned int event_num, void *dev_ptr)
{
	const struct device* dev = (struct device*)dev_ptr;
	struct uart_puppy_data *data = dev->data;

	if (event_num == ARCHI_UDMA_UART0_RX_EVT(0) || event_num == ARCHI_UDMA_UART1_RX_EVT(0))
		k_sem_give(&data->rx_lock);
}

static int uart_puppy_setup(const struct device *dev)
{
	struct uart_puppy_data *data = dev->data;
	const struct uart_config *config = &(data->uart_config);

	uint32_t base = data->base;
	uint32_t setup = 0;

	uint32_t baudrate = config->baudrate;

	uint32_t clk_div;
	uint32_t rxtx_en = 0x3;
	uint32_t polling_en;
	uint32_t stop_bits;
	uint32_t bit_length;
	uint32_t parity_en;

	if (baudrate > sys_clock_hw_cycles_per_sec()) {
		return -ENOTSUP;
	} else {
		clk_div = (sys_clock_hw_cycles_per_sec() + baudrate / 2) / baudrate - 1;
	}

#if CONFIG_UART_ASYNC_API
	polling_en = 0;
#else
	polling_en = 1;
#endif

	if (config->parity == UART_CFG_PARITY_NONE) {
		parity_en = 0;
	} else if (config->parity == UART_CFG_PARITY_EVEN) {
		parity_en = 1;
	} else {
		return -ENOTSUP;
	}

	if (config->stop_bits == UART_CFG_STOP_BITS_1) {
		stop_bits = 0;
	} else if (config->stop_bits == UART_CFG_STOP_BITS_2) {
		stop_bits = 1;
	} else {
		return -ENOTSUP;
	}

	if (config->data_bits == UART_CFG_DATA_BITS_9) {
		return -ENOTSUP;
	} else {
		bit_length = (config->data_bits) & 0x3;
	}

	setup = (clk_div << 16) | (rxtx_en << 8) | (polling_en << 4) | (stop_bits << 3) |
		(bit_length << 1) | parity_en;

	sys_write32(setup, base + UART_SETUP);

	return 0;
}

/*
 * Polling available only in read operation
 */
static int uart_puppy_poll_in(const struct device *dev, unsigned char *c)
{
	struct uart_puppy_data *data = dev->data;

	if (!sys_read32(data->base + UART_VALID)) { // Data not ready to read
		return -1;
	}

	*c = sys_read32(data->base + UART_DATA);
	return 0;
}

#if CONFIG_UART_ASYNC_API

static int uart_puppy_async_callback_set(const struct device *dev, uart_callback_t callback,
					 void *user_data)
{
	struct uart_puppy_data *data = dev->data;

	if (!callback) {
		return -EINVAL;
	}

	data->async.cb = callback;
	data->async.user_data = user_data;

#if defined(CONFIG_UART_EXCLUSIVE_API_CALLBACKS)
	data->async.cb = NULL;
	data->async.user_data = NULL;
#endif

	return 0;
}

static int uart_puppy_async_tx(const struct device *dev, const uint8_t *buf, size_t len,
			       int32_t timeout)
{
	struct uart_puppy_data *data = dev->data;

	while (!plp_udma_canEnqueue(data->base + UDMA_CHANNEL_TX_OFFSET))
		;

	k_sem_take(&data->tx_lock, K_FOREVER);

	plp_udma_enqueue(data->base + UDMA_CHANNEL_TX_OFFSET, (uint32_t)buf, len,
			 UDMA_CHANNEL_CFG_EN);

	return 0;
}

static void uart_puppy_clean_rx_fifo(const struct device *dev)
{
	struct uart_puppy_data *data = dev->data;

	uint32_t setup = sys_read32(data->base + UART_SETUP);

	// Set clean fifo bit to 1 then to 0 to perform a reset
	sys_write32(setup | UART_CLEAN_FIFO, data->base + UART_SETUP);
	sys_write32(setup, data->base + UART_SETUP);

	return;
}

static int uart_puppy_async_rx_enable(const struct device *dev, uint8_t *buf, size_t len,
				      int32_t timeout)
{
	struct uart_puppy_data *data = dev->data;

	uint32_t irq_en;

	while (!plp_udma_canEnqueue(data->base + UDMA_CHANNEL_RX_OFFSET))
		;
	k_sem_take(&data->rx_lock, K_FOREVER);

	uart_puppy_clean_rx_fifo(dev);

	irq_en = sys_read32(data->base + UART_IRQEN) | UART_RXIRQEN;
	sys_write32(data->base + UART_IRQEN, irq_en);

	plp_udma_enqueue(data->base + UDMA_CHANNEL_RX_OFFSET, (uint32_t)buf, len,
			 UDMA_CHANNEL_CFG_EN);

	// implement timeout

	return 0;
}

static int uart_tx_busy(const struct device *dev)
{
	struct uart_puppy_data *data = dev->data;

	return sys_read32(data->base + UART_STATUS) & UART_TX_BUSY;
}

/*
 * Polling available only in read operation.
 * Using Async method for this function instead for console use
 */
static void uart_puppy_poll_out(const struct device *dev, unsigned char c)
{
	while (uart_tx_busy(dev))
		;
	uart_puppy_async_tx(dev, &c, 1, -1);
	return;
}

#endif /* CONFIG_UART_ASYNC_API */

static int uart_puppy_init(const struct device *dev)
{
	struct uart_puppy_data *data = dev->data;

	uint32_t cg_conf = plp_udma_cg_get();

	k_sem_init(&data->tx_lock, 1, 1);
	k_sem_init(&data->rx_lock, 1, 1);

	if (data->id == 0) {
		plp_udma_cg_set(cg_conf | BIT(UDMA_UART0_ID));

		puppy_event_enable(ARCHI_UDMA_UART0_TX_EVT(0));
		puppy_event_enable(ARCHI_UDMA_UART0_RX_EVT(0));

		puppy_event_register_callback((event_callback_t)&tx_evt, (void*)dev);
		puppy_event_register_callback((event_callback_t)&rx_evt, (void*)dev);
	}

#ifdef CONFIG_SOC_PUPPY_V2
	else if (data->id == 1) {
		plp_udma_cg_set(cg_conf | BIT(UDMA_UART1_ID));

		puppy_event_enable(ARCHI_UDMA_UART1_TX_EVT(0));
		puppy_event_enable(ARCHI_UDMA_UART1_RX_EVT(0));

		puppy_event_register_callback((event_callback_t)&tx_evt, (void*)dev);
		puppy_event_register_callback((event_callback_t)&rx_evt, (void*)dev);
	}
#endif

	return uart_puppy_setup(dev);
}

static DEVICE_API(uart, uart_puppy_driver_api) = {
	.poll_in = uart_puppy_poll_in,
#if CONFIG_UART_ASYNC_API
	.poll_out = uart_puppy_poll_out, // Polling not available for Tx in Puppy, using async
					 // instead (see description up)
	.callback_set = uart_puppy_async_callback_set,
	.tx = uart_puppy_async_tx,
	.rx_enable = uart_puppy_async_rx_enable,
#endif /* CONFIG_UART_ASYNC_API */
};

#define PUPPY_UART_INIT(idx)                                                                       \
                                                                                                   \
	static struct uart_puppy_data uart_puppy_##idx##_data = {                                  \
		.base = DT_INST_REG_ADDR(idx),                                                     \
		.id = DT_INST_PROP(idx, uart_id),                                                  \
		.uart_config = {                                                                   \
			.baudrate = DT_INST_PROP(idx, current_speed),                              \
			.stop_bits = DT_INST_ENUM_IDX_OR(idx, stop_bits, UART_CFG_STOP_BITS_1),    \
			.data_bits = DT_INST_ENUM_IDX_OR(idx, data_bits, UART_CFG_DATA_BITS_8),    \
			.parity = DT_INST_ENUM_IDX_OR(idx, parity, UART_CFG_PARITY_NONE),          \
		}};                                                                                \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(idx, uart_puppy_init, NULL, &uart_puppy_##idx##_data, NULL,          \
			      PRE_KERNEL_1, CONFIG_SERIAL_INIT_PRIORITY, &uart_puppy_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PUPPY_UART_INIT);
