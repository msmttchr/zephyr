/*
 * uart_mux.c
 *
 * Application-level UART multiplexer implementation.
 *
 * This driver exposes multiple virtual UART devices backed by a single
 * physical UART using the Zephyr async UART API.
 *
 * Framing format:
 *   FI | FF | LEN (LE16) | PLC | PAYLOAD | CRC16-CCITT (LE)
 *
 * CRC calculation includes the FI byte.
 */

#include "uart_mux.h"

#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/devicetree.h>

LOG_MODULE_REGISTER(uart_mux, LOG_LEVEL_INF);

/* ============================= */
/* Protocol constants            */
/* ============================= */

#define UART_MUX_FI              0xC0
#define UART_MUX_FF              0x02

#define UART_MUX_HEADER_LEN      5
#define UART_MUX_CRC_LEN         2

/* ============================= */
/* Devicetree binding            */
/* ============================= */

#define DT_DRV_COMPAT st_uart_mux

/* ============================= */
/* TX descriptor slab            */
/* ============================= */

#define UART_MUX_TX_DESC_COUNT   4

K_MEM_SLAB_DEFINE(uart_mux_tx_slab,
		  sizeof(struct uart_mux_tx),
		  UART_MUX_TX_DESC_COUNT,
		  4);

/* ============================= */
/* CRC-16 CCITT                  */
/* ============================= */

static uint16_t crc16_ccitt_update(uint16_t crc, uint8_t data)
{
	crc ^= (uint16_t)data << 8;

	for (int i = 0; i < 8; i++) {
		if (crc & 0x8000) {
			crc = (crc << 1) ^ 0x1021;
		} else {
			crc <<= 1;
		}
	}

	return crc;
}

/* ============================= */
/* Global mux state              */
/* ============================= */

struct uart_mux {
	/* Physical UART selected via chosen(uart_mux) */
	const struct device *phy;

	/* Registered virtual UART channels */
	sys_slist_t channels;

	/* TX scheduling state */
	struct uart_mux_channel *tx_owner;
	struct uart_mux_tx *current_tx;
	atomic_t tx_segments_pending;

	/* RX parsing state */
	enum {
		RX_WAIT_FI,
		RX_HEADER,
		RX_PAYLOAD,
		RX_CRC,
	} rx_state;

	uint8_t phy_rx_buf[CONFIG_UART_MUX_PHY_RX_BUF_SIZE];

	uint8_t frame_payload[CONFIG_UART_MUX_RX_BUF_SIZE];
	size_t frame_len;
	size_t frame_pos;

	uint8_t hdr[UART_MUX_HEADER_LEN];
	size_t hdr_pos;

	uint8_t crc_buf[UART_MUX_CRC_LEN];
	size_t crc_pos;

	uint16_t crc_calc;
};

static struct uart_mux mux;

/* ============================= */
/* Helper functions              */
/* ============================= */

static bool plc_matches(const struct uart_mux_channel *ch, uint8_t plc)
{
	for (size_t i = 0; i < ch->plc_rx_len; i++) {
		if (ch->plc_rx[i] == plc) {
			return true;
		}
	}
	return false;
}

static struct uart_mux_channel *find_channel_by_plc(uint8_t plc)
{
	struct uart_mux_channel *ch;

	SYS_SLIST_FOR_EACH_CONTAINER(&mux.channels, ch, node) {
		if (plc_matches(ch, plc)) {
			return ch;
		}
	}

	return NULL;
}

static struct uart_mux_channel *pick_next_channel(void)
{
	struct uart_mux_channel *best = NULL;
	struct uart_mux_channel *ch;

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

/* ============================= */
/* TX path                       */
/* ============================= */

static void mux_try_tx(void);

static void mux_frame_tx(const struct uart_mux_tx *tx)
{
	uint8_t header[UART_MUX_HEADER_LEN];
	uint16_t crc = 0xFFFF;

	header[0] = UART_MUX_FI;
	header[1] = UART_MUX_FF;
	header[2] = tx->len & 0xFF;
	header[3] = tx->len >> 8;
	header[4] = tx->plc;

	for (size_t i = 0; i < UART_MUX_HEADER_LEN; i++) {
		crc = crc16_ccitt_update(crc, header[i]);
	}

	for (size_t i = 0; i < tx->len; i++) {
		crc = crc16_ccitt_update(crc, tx->buf[i]);
	}

	uint8_t crc_le[2] = {
		crc & 0xFF,
		crc >> 8,
	};

	/*
	 * One logical TX consists of three physical uart_tx() calls.
	 * Completion is signaled only after the last segment completes.
	 */
	atomic_set(&mux.tx_segments_pending, 3);

	uart_tx(mux.phy, header, sizeof(header), SYS_FOREVER_MS);
	uart_tx(mux.phy, tx->buf, tx->len, SYS_FOREVER_MS);
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

	mux_frame_tx(tx);
}

/* ============================= */
/* Physical UART callback        */
/* ============================= */

static void phy_uart_cb(const struct device *dev,
			struct uart_event *evt,
			void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	switch (evt->type) {

	case UART_TX_DONE:
		if (atomic_dec(&mux.tx_segments_pending) == 1) {

			struct uart_mux_tx *tx = mux.current_tx;
			struct uart_mux_channel *ch = mux.tx_owner;

			mux.current_tx = NULL;
			mux.tx_owner = NULL;

			if (tx) {
				k_mem_slab_free(&uart_mux_tx_slab,
						(void **)&tx);
			}

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
			uint8_t b = evt->data.rx.buf[i];

			switch (mux.rx_state) {

			case RX_WAIT_FI:
				if (b == UART_MUX_FI) {
					mux.hdr[0] = b;
					mux.hdr_pos = 1;
					mux.crc_calc = crc16_ccitt_update(0xFFFF, b);
					mux.rx_state = RX_HEADER;
				}
				break;

			case RX_HEADER:
				mux.hdr[mux.hdr_pos++] = b;
				mux.crc_calc = crc16_ccitt_update(mux.crc_calc, b);

				if (mux.hdr_pos == UART_MUX_HEADER_LEN) {
					mux.frame_len =
						(uint16_t)mux.hdr[2] |
						((uint16_t)mux.hdr[3] << 8);
					mux.frame_pos = 0;
					mux.rx_state = RX_PAYLOAD;
				}
				break;

			case RX_PAYLOAD:
				if (mux.frame_pos < sizeof(mux.frame_payload)) {
					mux.frame_payload[mux.frame_pos++] = b;
					mux.crc_calc =
						crc16_ccitt_update(mux.crc_calc, b);
				}
				if (mux.frame_pos == mux.frame_len) {
					mux.crc_pos = 0;
					mux.rx_state = RX_CRC;
				}
				break;

			case RX_CRC:
				mux.crc_buf[mux.crc_pos++] = b;
				if (mux.crc_pos == UART_MUX_CRC_LEN) {
					uint16_t rx_crc =
						mux.crc_buf[0] |
						((uint16_t)mux.crc_buf[1] << 8);

					if (rx_crc == mux.crc_calc &&
					    mux.frame_len > 0) {

						uint8_t plc =
							mux.hdr[4];

						struct uart_mux_channel *ch =
							find_channel_by_plc(plc);

						if (ch && ch->cb) {
							struct uart_event ev = {
								.type = UART_RX_RDY,
							};

							if (ch->plc_tx < 0) {
								/* Deliver payload including PLC */
								ev.data.rx.buf =
									mux.frame_payload;
								ev.data.rx.len =
									mux.frame_len;
							} else {
								/* Deliver payload without PLC */
								ev.data.rx.buf =
									&mux.frame_payload[1];
								ev.data.rx.len =
									mux.frame_len - 1;
							}

							ch->cb(ch->dev,
							       &ev,
							       ch->cb_data);
						}
					}

					mux.rx_state = RX_WAIT_FI;
				}
				break;
			}
		}
		break;

	default:
		break;
	}
}

/* ============================= */
/* Virtual UART API              */
/* ============================= */

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

	if (ch->plc_tx < 0) {
		/* PLC is first byte of payload */
		if (len < 1) {
			k_mem_slab_free(&uart_mux_tx_slab,
					(void **)&tx);
			return -EINVAL;
		}

		tx->plc = buf[0];
		tx->buf = &buf[1];
		tx->len = len - 1;
	} else {
		/* PLC is fixed, payload untouched */
		tx->plc = (uint8_t)ch->plc_tx;
		tx->buf = buf;
		tx->len = len;
	}

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

static const struct uart_driver_api uart_mux_api = {
	.tx = mux_uart_tx,
	.callback_set = mux_uart_callback_set,
};

/* ============================= */
/* Channel instantiation         */
/* ============================= */

struct uart_mux_cfg {
	uint8_t channel_id;
	uint8_t priority;
	int plc_tx;
	const uint8_t *plc_rx;
	size_t plc_rx_len;
};

static int uart_mux_ch_init(const struct device *dev)
{
	struct uart_mux_channel *ch = dev->data;
	const struct uart_mux_cfg *cfg = dev->config;

	ch->dev = dev;
	ch->channel_id = cfg->channel_id;
	ch->priority = cfg->priority;
	ch->plc_tx = cfg->plc_tx;
	ch->plc_rx = cfg->plc_rx;
	ch->plc_rx_len = cfg->plc_rx_len;

	k_fifo_init(&ch->tx_fifo);
	sys_slist_append(&mux.channels, &ch->node);

	return 0;
}

#define UART_MUX_DEFINE(inst)					\
	static uint8_t plc_rx_##inst[] =				\
		DT_INST_PROP(inst, plc_rx);				\
	static const struct uart_mux_cfg uart_mux_cfg_##inst = {	\
		.channel_id = DT_INST_PROP(inst, channel_id),		\
		.priority   = DT_INST_PROP(inst, priority),		\
		.plc_tx     = DT_INST_PROP(inst, plc_tx),		\
		.plc_rx     = plc_rx_##inst,				\
		.plc_rx_len = ARRAY_SIZE(plc_rx_##inst),		\
	};								\
	static struct uart_mux_channel uart_mux_ch_##inst;		\
	DEVICE_DT_DEFINE(DT_DRV_INST(inst),				\
			 uart_mux_ch_init, NULL,			\
			 &uart_mux_ch_##inst,				\
			 &uart_mux_cfg_##inst,				\
			 POST_KERNEL,					\
			 CONFIG_KERNEL_INIT_PRIORITY_DEVICE,		\
			 &uart_mux_api);

DT_INST_FOREACH_STATUS_OKAY(UART_MUX_DEFINE)

/* ============================= */
/* Mux initialization            */
/* ============================= */

static int uart_mux_init(void)
{
	mux.phy = DEVICE_DT_GET(DT_CHOSEN(uart_mux));
	__ASSERT(device_is_ready(mux.phy),
		 "Physical UART not ready");

	sys_slist_init(&mux.channels);

	uart_callback_set(mux.phy, phy_uart_cb, NULL);
	uart_rx_enable(mux.phy,
		       mux.phy_rx_buf,
		       sizeof(mux.phy_rx_buf),
		       SYS_FOREVER_MS);

	mux.rx_state = RX_WAIT_FI;

	return 0;
}

SYS_INIT(uart_mux_init,
	 POST_KERNEL,
	 CONFIG_KERNEL_INIT_PRIORITY_DEVICE);

