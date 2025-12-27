/* uart_mux.c */

#include "uart_mux.h"

#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/devicetree.h>

#define DT_DRV_COMPAT st_uart_mux

LOG_MODULE_REGISTER(uart_mux, CONFIG_LOG_DEFAULT_LEVEL);

/* ==== Frame format ==== */

#define MUX_FI        0xC0
#define MUX_FF        0x02

#define MUX_HDR_LEN   5   /* FI + FF + LEN(2) + PLC */
#define MUX_CRC_LEN   2

/* ==== CRC-16 CCITT-FALSE ==== */

#define CRC16_CCITT_POLY  0x1021
#define CRC16_CCITT_INIT  0xFFFF

/* ==== TX QUEUE SLAB ==== */

#define UART_MUX_TX_DESC_COUNT  8

K_MEM_SLAB_DEFINE(uart_mux_tx_slab,
		  sizeof(struct uart_mux_tx),
		  UART_MUX_TX_DESC_COUNT,
		  4);

static uint16_t crc16_ccitt_update(uint16_t crc, uint8_t data)
{
	crc ^= (uint16_t)data << 8;

	for (int i = 0; i < 8; i++) {
		if (crc & 0x8000) {
			crc = (crc << 1) ^ CRC16_CCITT_POLY;
		} else {
			crc <<= 1;
		}
	}
	return crc;
}

static uint16_t crc16_ccitt(const uint8_t *buf, size_t len)
{
	uint16_t crc = CRC16_CCITT_INIT;

	for (size_t i = 0; i < len; i++) {
		crc = crc16_ccitt_update(crc, buf[i]);
	}
	return crc;
}

/* ==== Global mux state ==== */

struct uart_mux {
	const struct device *phy;

	sys_slist_t channels;

	struct uart_mux_channel *tx_owner;
	struct uart_mux_tx *current_tx;
	atomic_t tx_segments_pending;

	/* RX state */
	enum {
		RX_WAIT_FI,
		RX_HDR,
		RX_PAYLOAD,
		RX_CRC,
	} rx_state;

	uint8_t  phy_rx_buf[CONFIG_UART_MUX_PHY_RX_BUF_SIZE];
	uint8_t  rx_hdr[MUX_HDR_LEN];
	size_t   rx_hdr_pos;

	uint8_t  frame_payload[CONFIG_UART_MUX_RX_BUF_SIZE];
	size_t  frame_len;
	size_t  frame_pos;

	uint8_t  rx_crc[2];
	size_t   rx_crc_pos;

	uint16_t rx_crc_calc;
};

static struct uart_mux mux;

/* ==== Helpers ==== */

static struct uart_mux_channel *find_channel(uint8_t plc)
{
	struct uart_mux_channel *ch;

	SYS_SLIST_FOR_EACH_CONTAINER(&mux.channels, ch, node) {
		for (int i = 0; i < sizeof(ch->plc_rx_len); i++) {
			if (ch->plc_rx[i] == plc) {
				return ch;
			}
		}
	}
	return NULL; /* drop frame */
}

static struct uart_mux_channel *pick_next_channel(void)
{
	struct uart_mux_channel *ch;
	struct uart_mux_channel *best = NULL;

	SYS_SLIST_FOR_EACH_CONTAINER(&mux.channels, ch, node) {
		if (k_fifo_is_empty(&ch->tx_fifo)) {
			continue;
		}

		if (!best || ch->priority < best->priority) {
			best = ch;
		}
	}
	return best;
}

/* ==== TX framing ==== */

static void mux_frame_tx(struct uart_mux_channel *ch,
                         struct uart_mux_tx *tx)
{
	uint8_t hdr[MUX_HDR_LEN];
	uint16_t crc;
        const uint8_t *payload = tx->buf;
	size_t len = tx->len;

	hdr[0] = MUX_FI;
	hdr[1] = MUX_FF;
	hdr[2] = len & 0xFF;
	hdr[3] = len >> 8;
	hdr[4] = tx->plc;
	
	if (ch->plc_tx < 0) {
		payload++;
		len--;
	}

	crc = crc16_ccitt(hdr, MUX_HDR_LEN);
	for (size_t i = 0; i < len; i++) {
		crc = crc16_ccitt_update(crc, payload[i]);
	}

	uint8_t crc_le[2] = {
		crc & 0xFF,
		crc >> 8,
	};

	atomic_set(&mux.tx_segments_pending, 3);

	uart_tx(mux.phy, hdr, sizeof(hdr), SYS_FOREVER_MS);
	uart_tx(mux.phy, payload, len, SYS_FOREVER_MS);
	uart_tx(mux.phy, crc_le, sizeof(crc_le), SYS_FOREVER_MS);
}

static void mux_try_tx(void)
{
	if (mux.tx_owner) {
		return;
	}

	struct uart_mux_channel *ch = pick_next_channel();
	if (!ch) {
		return;
	}

	struct uart_mux_tx *tx =
		k_fifo_get(&ch->tx_fifo, K_NO_WAIT);
	if (!tx) {
		return;
	}

	mux.tx_owner = ch;
	mux.current_tx = tx;

	mux_frame_tx(ch, tx);
}

/* ==== RX handling ==== */

static void mux_rx_byte(uint8_t b)
{
	switch (mux.rx_state) {

	case RX_WAIT_FI:
		if (b == MUX_FI) {
			mux.rx_hdr[0] = b;
			mux.rx_hdr_pos = 1;
			mux.rx_state = RX_HDR;
		}
		break;

	case RX_HDR:
		mux.rx_hdr[mux.rx_hdr_pos++] = b;

		if (mux.rx_hdr_pos == MUX_HDR_LEN) {

			if (mux.rx_hdr[1] != MUX_FF) {
				mux.rx_state = RX_WAIT_FI;
				break;
			}

			mux.frame_len = mux.rx_hdr[2] |
				     (mux.rx_hdr[3] << 8);

			if (mux.frame_len > sizeof(mux.frame_payload)) {
				mux.rx_state = RX_WAIT_FI;
				break;
			}

			struct uart_mux_channel *ch = find_channel(mux.rx_hdr[4]);
			mux.frame_pos = 0;
			if (ch->plc_tx < 0) {
				/* PLC should be considered part of the frame_payload */
			 	mux.frame_payload[mux.frame_pos++] = mux.rx_hdr[4];
				mux.frame_len++;
			}

			mux.rx_crc_calc =
				crc16_ccitt(mux.rx_hdr, MUX_HDR_LEN);

			mux.rx_state = RX_PAYLOAD;
		}
		break;

	case RX_PAYLOAD:
		mux.frame_payload[mux.frame_pos++] = b;
		mux.rx_crc_calc = crc16_ccitt_update(mux.rx_crc_calc, b);

		if (mux.frame_pos == mux.frame_len) {
			mux.rx_crc_pos = 0;
			mux.rx_state = RX_CRC;
		}
		break;

	case RX_CRC:
		mux.rx_crc[mux.rx_crc_pos++] = b;

		if (mux.rx_crc_pos == 2) {
			uint16_t rx_crc =
				mux.rx_crc[0] |
				(mux.rx_crc[1] << 8);

			if (rx_crc == mux.rx_crc_calc) {
				uint8_t plc = mux.rx_hdr[4];
				struct uart_mux_channel *ch =
					find_channel(plc);

				if (ch && ch->cb) {
					struct uart_event ev = {
						.type = UART_RX_RDY,
						.data.rx.buf = mux.frame_payload,
						.data.rx.len = mux.frame_len,
					};
					ch->cb(ch->dev, &ev, ch->cb_data);
				}
			}
			mux.rx_state = RX_WAIT_FI;
		}
		break;
	}
}

/* ==== Physical UART callback ==== */

static void phy_uart_cb(const struct device *dev,
			struct uart_event *evt,
			void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	switch (evt->type) {

	case UART_TX_DONE:
		if (atomic_dec(&mux.tx_segments_pending) == 1) {

			struct uart_mux_channel *ch = mux.tx_owner;
			struct uart_mux_tx *tx = mux.current_tx;

			mux.tx_owner = NULL;
			mux.current_tx = NULL;

			k_mem_slab_free(&uart_mux_tx_slab, (void *)tx);

			if (ch && ch->cb) {
				struct uart_event ev = {
					.type = UART_TX_DONE,
				};
				ch->cb(ch->dev, &ev, ch->cb_data);
			}

			mux_try_tx();
		}
		break;

	case UART_RX_RDY:
		for (size_t i = 0; i < evt->data.rx.len; i++) {
			mux_rx_byte(evt->data.rx.buf[i]);
		}
		break;

	default:
		break;
	}
}

/* ==== Virtual UART API ==== */

static int mux_uart_tx(const struct device *dev,
		       const uint8_t *buf,
		       size_t len,
		       int32_t timeout)
{
	ARG_UNUSED(timeout);

	struct uart_mux_channel *ch = dev->data;

	struct uart_mux_tx *tx;

	if (k_mem_slab_alloc(&uart_mux_tx_slab,
			     (void **)&tx,
			     K_NO_WAIT) != 0) {
		return -ENOMEM;
	}
	
	if (ch->plc_tx >= 0) {
		tx->plc = (uint8_t) ch->plc_tx;
	} else {
		tx->plc = buf[0];
	}
	tx->buf = (uint8_t *)buf;
	tx->len = len;

	k_fifo_put(&ch->tx_fifo, tx);
	mux_try_tx();

	return 0;
}

static int mux_uart_callback_set(const struct device *dev,
				 uart_callback_t cb,
				 void *user_data)
{
	struct uart_mux_channel *ch = dev->data;

	ch->cb = cb;
	ch->cb_data = user_data;
	return 0;
}

static const struct uart_driver_api mux_uart_api = {
	.tx = mux_uart_tx,
	.callback_set = mux_uart_callback_set,
};

/* ==== Channel device init ==== */

struct uart_mux_cfg {
	uint8_t channel_id;
	uint8_t priority;
	const uint8_t *plc_rx;
	uint8_t plc_rx_len;
	int plc_tx;
};

static int uart_mux_ch_init(const struct device *dev)
{
	struct uart_mux_channel *ch = dev->data;
	const struct uart_mux_cfg *cfg = dev->config;

	ch->dev = dev;
	ch->channel_id = cfg->channel_id;
	ch->priority = cfg->priority;
	ch->plc_rx = cfg->plc_rx;
	ch->plc_rx_len = cfg->plc_rx_len;
	ch->plc_tx = cfg->plc_tx;

	k_fifo_init(&ch->tx_fifo);
	sys_slist_append(&mux.channels, &ch->node);

	return 0;
}

/* ==== DTS instantiation ==== */

#define DT_DRV_COMPAT st_uart_mux

#define UART_MUX_DEFINE(inst)					\
	static struct uart_mux_channel ch_##inst;		\
	static const uint8_t ch_plc_rx_##inst[] =               \
	          DT_INST_PROP(inst, plc_rx);	         	\
	static const struct uart_mux_cfg cfg_##inst = {		\
		.channel_id = DT_INST_PROP(inst, channel_id),	\
		.priority   = DT_INST_PROP_OR(inst, priority, 1), \
		.plc_tx     = DT_INST_PROP_OR(inst, plc_tx, -1), \
		.plc_rx     = ch_plc_rx_##inst,                 \
		.plc_rx_len = sizeof(ch_plc_rx_##inst)          \
	};							\
	DEVICE_DT_DEFINE(DT_DRV_INST(inst),			\
			 uart_mux_ch_init,			\
			 NULL,					\
			 &ch_##inst,				\
			 &cfg_##inst,				\
			 POST_KERNEL,				\
			 CONFIG_KERNEL_INIT_PRIORITY_DEVICE,	\
			 &mux_uart_api);

DT_INST_FOREACH_STATUS_OKAY(UART_MUX_DEFINE)

/* ==== Mux core init ==== */

static int uart_mux_init(void)
{
	mux.phy = DEVICE_DT_GET(DT_CHOSEN(uart_mux));
	__ASSERT(device_is_ready(mux.phy), "Physical UART not ready");

	sys_slist_init(&mux.channels);

	uart_callback_set(mux.phy, phy_uart_cb, NULL);
	uart_rx_enable(mux.phy,
	       mux.phy_rx_buf,
	       sizeof(mux.phy_rx_buf),
	       SYS_FOREVER_MS);

	mux.rx_state = RX_WAIT_FI;

	return 0;
}

SYS_INIT(uart_mux_init, POST_KERNEL,
	 CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
