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
#include <zephyr/sys/byteorder.h>

LOG_MODULE_REGISTER(uart_mux, LOG_LEVEL_INF);

#define RX_TIMEOUT 500
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

static uint16_t crc16_ccitt(const uint8_t *data, size_t len)
{
	uint16_t crc = 0xFFFF;

	for (size_t i = 0; i < len; i++) {
		crc ^= (uint16_t)data[i] << 8;
		for (int j = 0; j < 8; j++) {
			if (crc & 0x8000) {
				crc = (crc << 1) ^ 0x1021;
			} else {
				crc <<= 1;
			}
		}
	}

	return crc;
}

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

enum uart_mux_rx_error {
	UART_MUX_RX_ERR_CRC,
	UART_MUX_RX_ERR_HEADER,
	UART_MUX_RX_ERR_LENGTH,
	UART_MUX_RX_ERR_NO_CHANNEL,
};

struct uart_mux {
	/* Physical UART selected via chosen(uart_mux) */
	const struct device *phy;

	/* Registered virtual UART channels */
	sys_slist_t channels;

	/* TX scheduling state */
	struct uart_mux_channel *tx_owner;
	struct uart_mux_tx *tx_ctx;

	enum uart_mux_tx_stage tx_stage;
	
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

	uint8_t rx_plc;
	struct uart_mux_channel *rx_ch;

	uint8_t hdr[UART_MUX_HEADER_LEN];
	size_t hdr_pos;

	uint8_t crc_buf[UART_MUX_CRC_LEN];
	size_t crc_pos;

	uint16_t crc_calc;

	int64_t rx_last_activity_ts;
	bool rx_paused;

	struct uart_mux_rx_stats rx_stats;
};

static struct uart_mux mux = {
    .channels = SYS_SLIST_STATIC_INIT(&mux.channels),
};


/* ============================= */
/* Helper functions              */
/* ============================= */

static void notify_rx_stopped(struct uart_mux_channel *ch, int reason)
{
	if (!ch || !ch->cb) {
		return;
	}

	struct uart_event ev = {
		.type = UART_RX_STOPPED,
		.data.rx_stop.reason = reason,
	};

	ch->cb(ch->dev, &ev, ch->cb_data);
}

static void deliver_rx(struct uart_mux_channel *ch,
		       uint8_t *data, size_t len)
{
	if (!ch->cb || !ch->rx_enabled) {
		return;
	}

	struct uart_event evt = {
		.type = UART_RX_RDY,
		.data.rx.buf = data,
		.data.rx.len = len,
		.data.rx.offset = 0,
	};

	ch->cb(ch->dev, &evt, ch->cb_data);
}

static void mux_rx_reset(void)
{
	mux.rx_state = RX_WAIT_FI;
	mux.hdr_pos = 0;
	mux.frame_pos = 0;
	mux.crc_pos = 0;
	mux.frame_len = 0;
	mux.rx_ch = NULL;
}

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

void uart_mux_get_rx_stats(struct uart_mux_rx_stats *out)
{
	*out = mux.rx_stats;
}

static void mux_rx_watchdog_check(void)
{
	if (mux.rx_state == RX_WAIT_FI || mux.rx_paused) {
		return;
	}

	int64_t now = k_uptime_get();
	if ((now - mux.rx_last_activity_ts) >
	    CONFIG_UART_MUX_RX_TIMEOUT_MS) {

		/* RX frame timed out → resync */
		mux.rx_stats.header_errors++;
		mux_rx_reset();
	}
}

static void mux_rx_byte(uint8_t b)
{
	switch (mux.rx_state) {

	/* ---------------- WAIT FI ---------------- */
	case RX_WAIT_FI:
		if (b == UART_MUX_FI) {
			mux.crc_calc = 0xFFFF;
			mux.crc_calc = crc16_ccitt_update(mux.crc_calc, b);

			mux.hdr[0] = b;
			mux.hdr_pos = 1;
			mux.rx_state = RX_HEADER;
		}
		break;

	/* ---------------- HEADER ---------------- */
	case RX_HEADER:
		mux.hdr[mux.hdr_pos++] = b;
		mux.crc_calc = crc16_ccitt_update(mux.crc_calc, b);

		if (mux.hdr_pos == UART_MUX_HEADER_LEN) {
			if (mux.hdr[1] != UART_MUX_FF) {
				mux.rx_stats.header_errors++;
				mux_rx_reset();
				break;
			}

			mux.frame_len = sys_get_le16(&mux.hdr[2]);

			if (mux.frame_len > CONFIG_UART_MUX_RX_BUF_SIZE) {
				mux.rx_stats.length_errors++;
				notify_rx_stopped(NULL, UART_MUX_RX_ERR_LENGTH);
				mux_rx_reset();
				break;
			}
			mux.rx_ch = find_channel_by_plc(mux.hdr[4]);
			if (mux.rx_ch == NULL) {
				mux.rx_stats.no_channel++;
				notify_rx_stopped(NULL, UART_MUX_RX_ERR_NO_CHANNEL);
				mux_rx_reset();
				break;
			}
			mux.rx_state = RX_PAYLOAD;
			mux.frame_pos = 0;
			if (mux.rx_ch->plc_tx < 0) {
				/* Special case: plc should be sent as a first bye of payload */
				mux.frame_payload[mux.frame_pos++] = mux.hdr[4];
				mux.frame_len++;
			}
		}
		break;

	/* ---------------- PAYLOAD ---------------- */
	case RX_PAYLOAD:
		mux.frame_payload[mux.frame_pos++] = b;
		mux.crc_calc = crc16_ccitt_update(mux.crc_calc, b);

		if (mux.frame_pos == mux.frame_len) {
			mux.rx_state = RX_CRC;
			mux.crc_pos = 0;
		}
		break;

	/* ---------------- CRC ---------------- */
	case RX_CRC:
		mux.crc_buf[mux.crc_pos++] = b;

		if (mux.crc_pos == UART_MUX_CRC_LEN) {
			uint16_t rx_crc =
				sys_get_le16(mux.crc_buf);

			if (rx_crc == mux.crc_calc) {

				mux.rx_stats.frames_ok++;
				deliver_rx(mux.rx_ch,
					   mux.frame_payload,
					   mux.frame_len);
			} else {
				mux.rx_stats.crc_errors++;
				/* Notify CRC error */
				notify_rx_stopped(mux.rx_ch, UART_MUX_RX_ERR_CRC);
			}
			mux_rx_reset();
		}
		break;
	}
}

void uart_mux_rx_inject(const uint8_t *data, size_t len)
{
	if (!data || len == 0) {
		return;
	}

	mux.rx_last_activity_ts = k_uptime_get();

	for (size_t i = 0; i < len; i++) {
		mux_rx_byte(data[i]);
	}
}

void uart_mux_rx_pause(bool pause)
{
	mux.rx_paused = pause;
}

bool uart_mux_rx_is_paused(void)
{
	return mux.rx_paused;
}

/* ============================= */
/* TX path                       */
/* ============================= */

static void mux_try_tx(void);


static int mux_frame_tx(struct uart_mux_tx *tx)
{
	uint16_t crc;
	int ret;

	/* Build header: FI, FF, LEN (LE), PLC */
	tx->hdr[0] = UART_MUX_FI;
	tx->hdr[1] = UART_MUX_FF;
	tx->hdr[2] = tx->len & 0xff;
	tx->hdr[3] = tx->len >> 8;
	tx->hdr[4] = tx->plc;

	/* Compute CRC over header + payload (CRC includes FI) */
	crc = crc16_ccitt(tx->hdr, UART_MUX_HEADER_LEN);
	for (int i = 0; i < tx->len; i++) {
		crc = crc16_ccitt_update(crc, tx->buf[i]);
	}
	tx->crc = crc;

	/* Start TX state machine with header */
	mux.tx_stage = UART_MUX_TX_HDR;

	ret = uart_tx(mux.phy,
		      tx->hdr,
		      UART_MUX_HEADER_LEN,
		      SYS_FOREVER_MS);

	return ret;
}

struct k_spinlock tx_lock;

static void mux_try_tx(void)
{
	int ret;

	k_spinlock_key_t key = k_spin_lock(&tx_lock);
	if (mux.tx_owner) {
		goto out;
	}

	struct uart_mux_channel *ch = pick_next_channel();
	if (!ch) {
		goto out;
	}

	struct uart_mux_tx *tx =
		k_fifo_get(&ch->tx_fifo, K_NO_WAIT);
	if (!tx) {
		goto out;
	}

	/* Claim ownership */
	mux.tx_owner = ch;
	mux.tx_ctx = tx;

	ret = mux_frame_tx(tx);
	if (ret) {
		/* Roll back cleanly */
		mux.tx_owner = NULL;
		mux.tx_ctx = NULL;
		mux.tx_stage = UART_MUX_TX_IDLE;

		/* Put TX back so it is not lost */
		k_fifo_put(&ch->tx_fifo, tx);
	}
out:
	k_spin_unlock(&tx_lock, key);
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

	mux_rx_watchdog_check();
	switch (evt->type) {

	case UART_TX_DONE:
		switch (mux.tx_stage) {

		case UART_MUX_TX_HDR:
			/* Header done → send payload */
			mux.tx_stage = UART_MUX_TX_PAYLOAD;

			uart_tx(mux.phy,
				mux.tx_ctx->buf,
				mux.tx_ctx->len,
				SYS_FOREVER_MS);
			break;

		case UART_MUX_TX_PAYLOAD:
			/*
			 * Payload done:
			 * - notify virtual UART
			 * - then send CRC
			 */
			if (mux.tx_owner && mux.tx_owner->cb) {
				struct uart_event ev = {
					.type = UART_TX_DONE,
					.data.tx.len = mux.tx_ctx->len,
				};

				mux.tx_owner->cb(mux.tx_owner->dev,
						  &ev,
						  mux.tx_owner->cb_data);
			}

			mux.tx_stage = UART_MUX_TX_CRC;

			uart_tx(mux.phy,
				(uint8_t *)&mux.tx_ctx->crc,
				UART_MUX_CRC_LEN,
				SYS_FOREVER_MS);
			break;

		case UART_MUX_TX_CRC:

			if (mux.tx_owner && mux.tx_owner->tx_done_cb) {
				mux.tx_owner->tx_done_cb(mux.tx_ctx->buf,
							 mux.tx_ctx->len,
							 mux.tx_owner->tx_done_user_data);
			}

			/* Final stage: free TX context and release ownership */
			k_mem_slab_free(&uart_mux_tx_slab, mux.tx_ctx);

			mux.tx_ctx = NULL;
			mux.tx_owner = NULL;
			mux.tx_stage = UART_MUX_TX_IDLE;

			/* Schedule next TX based on channel priority */
			mux_try_tx();
			break;

		default:
			/* Should never happen */
			break;
		}
		break;

	case UART_TX_ABORTED:
		if (mux.tx_ctx) {
			k_mem_slab_free(&uart_mux_tx_slab, mux.tx_ctx);
		}
		mux.tx_ctx = NULL;
		mux.tx_owner = NULL;
		mux.tx_stage = UART_MUX_TX_IDLE;

		mux_try_tx();
		break;

	case UART_RX_RDY: {
		const uint8_t *buf = evt->data.rx.buf;
		size_t len = evt->data.rx.len;
		size_t off = evt->data.rx.offset;

		if (mux.rx_paused) {
			return;
		}

		mux.rx_last_activity_ts = k_uptime_get();
		for (size_t i = 0; i < len; i++) {
			mux_rx_byte(buf[off + i]);
		}
		break;
	}

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

static int mux_uart_rx_enable(const struct device *dev,
			      uint8_t *buf, size_t len,
			      int32_t timeout)
{
	struct uart_mux_channel *ch = dev->data;

	ARG_UNUSED(buf);
	ARG_UNUSED(len);
	ARG_UNUSED(timeout);

	ch->rx_enabled = true;
	return 0;
}

static int mux_uart_rx_disable(const struct device *dev)
{
	struct uart_mux_channel *ch = dev->data;
	ch->rx_enabled = false;
	return 0;
}


static const struct uart_driver_api uart_mux_api = {
	.tx = mux_uart_tx,
	.callback_set = mux_uart_callback_set,
	.rx_enable = mux_uart_rx_enable,
	.rx_disable = mux_uart_rx_disable,
	/* Intentionally not implemented as it is not needed for virtual UART */
	.rx_buf_rsp = NULL,
};

/* ============================= */
/* Channel instantiation         */
/* ============================= */

struct uart_mux_cfg {
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
	ch->priority = cfg->priority;
	ch->plc_tx = cfg->plc_tx;
	ch->plc_rx = cfg->plc_rx;
	ch->plc_rx_len = cfg->plc_rx_len;

	k_fifo_init(&ch->tx_fifo);
	sys_slist_append(&mux.channels, &ch->node);

	return 0;
}

int uart_mux_register_tx_done_cb(const struct device *dev,
				 uart_mux_tx_done_cb_t cb,
				 void *user_data)
{
	struct uart_mux_channel *ch = dev->data;

	ch->tx_done_cb = cb;
	ch->tx_done_user_data = user_data;
	return 0;
}


#define UART_MUX_DEFINE(inst)					\
	static uint8_t plc_rx_##inst[] =				\
		DT_INST_PROP(inst, plc_rx);				\
	static const struct uart_mux_cfg uart_mux_cfg_##inst = {	\
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

	/* RX delivery enabled by default */
	mux.rx_paused = false;

	uart_callback_set(mux.phy, phy_uart_cb, NULL);
	uart_rx_enable(mux.phy,
		       mux.phy_rx_buf,
		       sizeof(mux.phy_rx_buf),
		       RX_TIMEOUT);

	mux.rx_state = RX_WAIT_FI;

	return 0;
}

SYS_INIT(uart_mux_init,
	 POST_KERNEL,
	 CONFIG_KERNEL_INIT_PRIORITY_DEVICE);

