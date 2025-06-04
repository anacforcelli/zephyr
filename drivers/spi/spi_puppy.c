#define DT_DRV_COMPAT caninos_puppy_spi

#include <../drivers/spi/spi_context.h>
#include <zephyr/logging/log.h>
#include "spi_puppy.h"

LOG_MODULE_REGISTER(spi_puppy, CONFIG_SPI_LOG_LEVEL);

struct spi_puppy_data
{
	struct spi_context ctx;
	struct spi_config *config;
	bool async;
	uint32_t base;
	int id;
	uint32_t cmd_buf[SPI_CMD_BUF_SIZE];
};

int spi_puppy_calc_clkdiv(int frequency)
{
	int clkdiv = 0;

	if (frequency < sys_clock_hw_cycles_per_sec())
	{
		// Round-up the divider to obtain an SPI frequency which is below the maximum
		clkdiv = (sys_clock_hw_cycles_per_sec() + frequency - 1) / frequency;

		// The SPIM always divide by 2 once we activate the divider, thus increase by 1
		// in case it is even to not go above the max frequency.
		if (clkdiv & 1)
			clkdiv += 1;

		clkdiv >>= 1;
	}

	return clkdiv;
}

static int spi_puppy_transceive(const struct device *dev,
								const struct spi_config *config,
								const struct spi_buf_set *tx_bufs,
								const struct spi_buf_set *rx_bufs)
{
	struct spi_puppy_data *data = (struct spi_puppy_data *)dev->data;

	int cpol = config->operation & SPI_MODE_CPOL;
	int cpha = config->operation & SPI_MODE_CPHA;
	int clkdiv = spi_puppy_calc_clkdiv(config->frequency);
	int cfg, word_size;

	int error = 0;

	data->cmd_buf[0] = SPI_CMD_CFG(clkdiv, cpol, cpha);

	word_size = (config->operation & SPI_WORD_SIZE_MASK) >> SPI_WORD_SIZE_SHIFT;

	switch (word_size)
	{
	case 8:
		cfg = UDMA_CHANNEL_CFG_SIZE_8;
		break;
	case 16:
		cfg = UDMA_CHANNEL_CFG_SIZE_16;
		break;
	case 32:
		cfg = UDMA_CHANNEL_CFG_SIZE_32;
		break;
	default:
		cfg = UDMA_CHANNEL_CFG_SIZE_8;
	}

	plp_udma_enqueue(data->base + UDMA_CHANNEL_CMD_OFFSET, (uint32_t)data->cmd_buf, sizeof(uint32_t), 0); // Size in Bytes!

	spi_context_lock(&data->ctx, false, NULL, NULL, config);
	spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 1);

	size_t chunk_len = spi_context_max_continuous_chunk(&data->ctx);

	while (chunk_len > 0)
	{
		int tx_len = spi_context_tx_on(&data->ctx) ? chunk_len : 0;
		int rx_len = spi_context_rx_on(&data->ctx) ? chunk_len : 0;
		int index = 0;

		// Set buffer sizes
		/*data->cmd_buf[1] = SPI_CMD_SETUP_UCA(0, &tx_bufs->buffers[i]->buf);
		data->cmd_buf[2] = SPI_CMD_SETUP_UCS(0, chunk_len);
		data->cmd_buf[3] = SPI_CMD_SETUP_UCA(1, &rx_bufs->buffers[i]->buf);
		data->cmd_buf[4] = SPI_CMD_SETUP_UCS(1, chunk_len);*/

		spi_context_cs_control(&data->ctx, true);

		data->cmd_buf[index++] = SPI_CMD_SOT(0);

		if (tx_len > 0 && rx_len > 0)
			data->cmd_buf[index++] = SPI_CMD_FUL(chunk_len, SPI_CMD_1_WORD_PER_TRANSF, SPI_WORD_SIZE_GET(config->operation), config->operation & SPI_TRANSFER_LSB ? SPI_CMD_LSB_FIRST : SPI_CMD_MSB_FIRST);
		else if (tx_len > 0)
			data->cmd_buf[index++] = SPI_CMD_TX_DATA(chunk_len, SPI_CMD_1_WORD_PER_TRANSF, SPI_WORD_SIZE_GET(config->operation), config->operation & SPI_LINES_QUAD, config->operation & SPI_TRANSFER_LSB ? SPI_CMD_LSB_FIRST : SPI_CMD_MSB_FIRST);
		else if (rx_len > 0)
			data->cmd_buf[index++] = SPI_CMD_RX_DATA(chunk_len, SPI_CMD_1_WORD_PER_TRANSF, SPI_WORD_SIZE_GET(config->operation), config->operation & SPI_LINES_QUAD, config->operation & SPI_TRANSFER_LSB ? SPI_CMD_LSB_FIRST : SPI_CMD_MSB_FIRST);

		// Finish & generate event
		data->cmd_buf[index++] = SPI_CMD_EOT(1, (config->operation & SPI_HOLD_ON_CS) ? 1 : 0);

		if (tx_len > 0 || rx_len > 0)
			plp_udma_enqueue(data->base + UDMA_CHANNEL_CMD_OFFSET, (uint32_t)data->cmd_buf, index * sizeof(uint32_t), cfg);
		if (tx_len > 0)
			plp_udma_enqueue(data->base + UDMA_CHANNEL_TX_OFFSET, (uint32_t)data->ctx.tx_buf, tx_len, cfg);
		if (rx_len > 0)
			plp_udma_enqueue(data->base + UDMA_CHANNEL_RX_OFFSET, (uint32_t)data->ctx.rx_buf, rx_len, cfg);

		if (data->async)
			;
		// event handling
		else
			spi_context_complete(&data->ctx, dev, error);

		error = spi_context_wait_for_completion(&data->ctx);

		if (error == -ETIMEDOUT)
		{
			chunk_len = 0;
		}
		else
		{
			spi_context_update_tx(&data->ctx, word_size / sizeof(uint8_t), chunk_len);
			spi_context_update_rx(&data->ctx, word_size / sizeof(uint8_t), chunk_len);
		}

		chunk_len = spi_context_max_continuous_chunk(&data->ctx);
	}

	spi_context_release(&data->ctx, error);

	return error;
}

static DEVICE_API(spi, spi_puppy_driver_api) = {
	.transceive = spi_puppy_transceive,
};

static int spi_puppy_init(const struct device *dev)
{
	int err;
	struct spi_puppy_data *data = dev->data;

	// Set clock-gating config
	uint32_t cg_conf = plp_udma_cg_get();
	plp_udma_cg_set(cg_conf | BIT(UDMA_SPIM_ID + data->id));

	data->async = false;

	err = spi_context_cs_configure_all(&data->ctx);
	if (err < 0)
	{
		return err;
	}

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

#define PUPPY_SPI_INIT(idx)                                 \
	static struct spi_puppy_data spi_puppy_##idx##_data = { \
		SPI_CONTEXT_INIT_LOCK(spi_puppy_##idx##_data, ctx), \
		SPI_CONTEXT_INIT_SYNC(spi_puppy_##idx##_data, ctx), \
		SPI_CONTEXT_CS_GPIOS_INITIALIZE(idx, ctx)           \
		.async = false,                                 \
		.base = DT_INST_REG_ADDR(idx),                      \
		.id = DT_INST_PROP(idx, spi_id),                    \
	};                                                      \
	SPI_DEVICE_DT_INST_DEFINE(idx,                          \
							  spi_puppy_init,               \
							  NULL,                         \
							  &spi_puppy_##idx##_data,      \
							  NULL,                         \
							  PRE_KERNEL_1,                 \
							  CONFIG_SPI_INIT_PRIORITY,     \
							  &spi_puppy_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PUPPY_SPI_INIT);
