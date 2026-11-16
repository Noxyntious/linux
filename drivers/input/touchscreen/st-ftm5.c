// SPDX-License-Identifier: GPL-2.0-only
/*
 * Minimal driver for STMicroelectronics FTM5 touch controllers on SPI, as
 * used by the Google Pixel 8 Pro (husky).
 *
 * The controller runs its own firmware from internal flash; this driver does
 * not update or tune it. It resets the chip, waits for the "controller ready"
 * event, turns scanning on and reports touches from the event FIFO.
 *
 * Protocol from Google's ftm5 driver (kernel/google-modules/touch/fts_touch,
 * ftm5/, android-gs-shusky-6.1-android16), SPI variant:
 *  - FIFO read: command 0x87, one dummy byte, then 8-byte events; byte 7 of
 *    the first event holds the number of further events queued.
 *  - scan on:  FA 20 00 00 00 00 00, then A0 00 FF (active, all sensing).
 *  - scan off: FA 20 00 00 00 00 00, then A0 00 00.
 *  - events: 0x03 controller ready, 0x13 enter, 0x23 motion, 0x33 leave
 *    (low nibble 3 = pointer group); id = ev[1] >> 4, type = ev[1] & 0xF,
 *    x = (ev[3] & 0xF) << 8 | ev[2], y = ev[4] << 4 | ev[3] >> 4, z = ev[5].
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>

#define FTM5_EVENT_SIZE		8
#define FTM5_FIFO_DEPTH		32
#define FTM5_CMD_READ_FIFO	0x87
#define FTM5_CMD_SCAN_MODE	0xA0
#define FTM5_SCAN_ACTIVE	0x00

#define FTM5_EVT_NONE		0x00
#define FTM5_EVT_READY		0x03
#define FTM5_EVT_ENTER		0x13
#define FTM5_EVT_MOTION		0x23
#define FTM5_EVT_LEAVE		0x33
#define FTM5_EVT_ERROR		0xF3

#define FTM5_MAX_TOUCHES	10
#define FTM5_DEBUG_EVENTS	16	/* log this many raw events after probe */

struct ftm5 {
	struct spi_device *spi;
	struct input_dev *input;
	struct touchscreen_properties prop;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[2];
	unsigned int debug_left;
	u8 tx[8] ____cacheline_aligned;
	u8 rx[1 + FTM5_EVENT_SIZE * FTM5_FIFO_DEPTH] ____cacheline_aligned;
};

static int ftm5_write(struct ftm5 *ts, const u8 *buf, size_t len)
{
	memcpy(ts->tx, buf, len);
	return spi_write(ts->spi, ts->tx, len);
}

/* Read @n events (with the leading dummy byte) into ts->rx + 1. */
static int ftm5_read_events(struct ftm5 *ts, unsigned int n)
{
	struct spi_transfer xfer[2] = {
		{ .tx_buf = ts->tx, .len = 1 },
		{ .rx_buf = ts->rx, .len = 1 + n * FTM5_EVENT_SIZE },
	};

	ts->tx[0] = FTM5_CMD_READ_FIFO;
	return spi_sync_transfer(ts->spi, xfer, ARRAY_SIZE(xfer));
}

static int ftm5_set_scan(struct ftm5 *ts, bool on)
{
	static const u8 pre[] = { 0xFA, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00 };
	const u8 cmd[] = { FTM5_CMD_SCAN_MODE, FTM5_SCAN_ACTIVE, on ? 0xFF : 0x00 };
	int ret;

	ret = ftm5_write(ts, pre, sizeof(pre));
	if (ret)
		return ret;
	return ftm5_write(ts, cmd, sizeof(cmd));
}

static void ftm5_report(struct ftm5 *ts, const u8 *ev)
{
	unsigned int id = ev[1] >> 4;
	unsigned int x = ((ev[3] & 0x0F) << 8) | ev[2];
	unsigned int y = (ev[4] << 4) | (ev[3] >> 4);
	unsigned int z = ev[5] ? ev[5] : 1;
	bool down = ev[0] != FTM5_EVT_LEAVE;

	if (id >= FTM5_MAX_TOUCHES)
		return;

	input_mt_slot(ts->input, id);
	input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, down);
	if (down) {
		touchscreen_report_pos(ts->input, &ts->prop, x, y, true);
		input_report_abs(ts->input, ABS_MT_PRESSURE, z);
	}
}

static irqreturn_t ftm5_irq(int irq, void *data)
{
	struct ftm5 *ts = data;
	unsigned int more, i;
	bool pointer = false;
	u8 *ev;

	if (ftm5_read_events(ts, 1))
		return IRQ_HANDLED;

	ev = ts->rx + 1;
	if (ev[0] == FTM5_EVT_NONE)
		return IRQ_HANDLED;

	more = min_t(unsigned int, ev[7] & 0x1F, FTM5_FIFO_DEPTH - 1);
	if (more) {
		u8 first[FTM5_EVENT_SIZE];

		memcpy(first, ev, FTM5_EVENT_SIZE);
		if (ftm5_read_events(ts, more))
			return IRQ_HANDLED;
		/* the follow-up read lands at rx + 1; put event 0 back in front */
		memmove(ts->rx + 1 + FTM5_EVENT_SIZE, ts->rx + 1,
			more * FTM5_EVENT_SIZE);
		memcpy(ts->rx + 1, first, FTM5_EVENT_SIZE);
	}

	for (i = 0; i <= more; i++) {
		ev = ts->rx + 1 + i * FTM5_EVENT_SIZE;

		if (ts->debug_left) {
			ts->debug_left--;
			dev_info(&ts->spi->dev, "event %*ph\n", FTM5_EVENT_SIZE, ev);
		}

		switch (ev[0]) {
		case FTM5_EVT_ENTER:
		case FTM5_EVT_MOTION:
		case FTM5_EVT_LEAVE:
			ftm5_report(ts, ev);
			pointer = true;
			break;
		case FTM5_EVT_ERROR:
			dev_warn(&ts->spi->dev, "error event %*ph\n",
				 FTM5_EVENT_SIZE, ev);
			break;
		default:
			break;
		}
	}

	if (pointer) {
		input_mt_sync_frame(ts->input);
		input_sync(ts->input);
	}

	return IRQ_HANDLED;
}

/* Reset the chip and wait for its "controller ready" event (IRQ masked). */
static int ftm5_reset(struct ftm5 *ts)
{
	int i;

	if (ts->reset_gpio) {
		gpiod_set_value_cansleep(ts->reset_gpio, 1);
		msleep(10);
		gpiod_set_value_cansleep(ts->reset_gpio, 0);
	} else {
		/* software system reset: write 0x81 to hw register 0x20000024 */
		static const u8 rst[] = { 0xFA, 0x20, 0x00, 0x00, 0x24, 0x81 };

		ftm5_write(ts, rst, sizeof(rst));
	}

	for (i = 0; i < 100; i++) {
		msleep(10);
		if (ftm5_read_events(ts, 1))
			continue;
		if (ts->rx[1] == FTM5_EVT_READY) {
			dev_info(&ts->spi->dev, "controller ready %*ph\n",
				 FTM5_EVENT_SIZE, ts->rx + 1);
			return 0;
		}
	}

	dev_err(&ts->spi->dev, "no controller-ready event (last %*ph)\n",
		FTM5_EVENT_SIZE, ts->rx + 1);
	return -ETIMEDOUT;
}

static int ftm5_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct ftm5 *ts;
	int ret;

	ts = devm_kzalloc(dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;
	ts->spi = spi;
	ts->debug_left = FTM5_DEBUG_EVENTS;
	spi_set_drvdata(spi, ts);

	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;
	ret = spi_setup(spi);
	if (ret)
		return ret;

	ts->supplies[0].supply = "vdd";
	ts->supplies[1].supply = "avdd";
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(ts->supplies),
				      ts->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get supplies\n");

	/* asserted = in reset; requested deasserted so a running chip stays up */
	ts->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ts->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ts->reset_gpio),
				     "failed to get reset gpio\n");

	ret = regulator_bulk_enable(ARRAY_SIZE(ts->supplies), ts->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable supplies\n");
	msleep(20);

	ret = ftm5_reset(ts);
	if (ret)
		goto err_power;

	ts->input = devm_input_allocate_device(dev);
	if (!ts->input) {
		ret = -ENOMEM;
		goto err_power;
	}
	ts->input->name = "ST FTM5 touchscreen";
	ts->input->id.bustype = BUS_SPI;
	input_set_abs_params(ts->input, ABS_MT_POSITION_X, 0, 1343, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_POSITION_Y, 0, 2991, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_PRESSURE, 0, 255, 0, 0);
	touchscreen_parse_properties(ts->input, true, &ts->prop);
	ret = input_mt_init_slots(ts->input, FTM5_MAX_TOUCHES,
				  INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
	if (ret)
		goto err_power;

	ret = input_register_device(ts->input);
	if (ret)
		goto err_power;

	ret = devm_request_threaded_irq(dev, spi->irq, NULL, ftm5_irq,
					IRQF_ONESHOT, "ftm5", ts);
	if (ret) {
		dev_err(dev, "failed to request irq %d: %d\n", spi->irq, ret);
		goto err_power;
	}

	ret = ftm5_set_scan(ts, true);
	if (ret) {
		dev_err(dev, "failed to start scanning: %d\n", ret);
		goto err_power;
	}

	dev_info(dev, "FTM5 touchscreen up, irq %d\n", spi->irq);
	return 0;

err_power:
	regulator_bulk_disable(ARRAY_SIZE(ts->supplies), ts->supplies);
	return ret;
}

static void ftm5_remove(struct spi_device *spi)
{
	struct ftm5 *ts = spi_get_drvdata(spi);

	ftm5_set_scan(ts, false);
	regulator_bulk_disable(ARRAY_SIZE(ts->supplies), ts->supplies);
}

static const struct of_device_id ftm5_of_match[] = {
	{ .compatible = "st,ftm5" },
	{ }
};
MODULE_DEVICE_TABLE(of, ftm5_of_match);

static const struct spi_device_id ftm5_spi_ids[] = {
	{ "ftm5" },
	{ }
};
MODULE_DEVICE_TABLE(spi, ftm5_spi_ids);

static struct spi_driver ftm5_driver = {
	.driver = {
		.name = "st-ftm5",
		.of_match_table = ftm5_of_match,
	},
	.id_table = ftm5_spi_ids,
	.probe = ftm5_probe,
	.remove = ftm5_remove,
};
module_spi_driver(ftm5_driver);

MODULE_DESCRIPTION("STMicroelectronics FTM5 touchscreen (minimal)");
MODULE_LICENSE("GPL");
