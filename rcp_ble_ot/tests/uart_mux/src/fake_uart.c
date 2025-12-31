/*
 * Fake UART driver for uart_mux unit tests
 *
 * This driver implements a minimal UART async API sufficient to:
 *  - accept uart_tx() calls
 *  - immediately complete TX with UART_TX_DONE
 *  - optionally inject UART_TX_ABORTED
 *  - inject RX data as UART_RX_RDY events
 *
 * It is NOT a real UART driver.
 * It is intentionally synchronous and simplified to:
 *  - make tests deterministic
 *  - avoid timing dependencies
 *  - avoid hardware
 *
 * Design goals:
 *  - correctness over realism
 *  - bounded memory usage
 *  - stress-safe (no buffer overruns)
 */

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <string.h>

#define FAKE_UART DEVICE_DT_GET(DT_NODELABEL(fake_uart))

#define DT_DRV_COMPAT zephyr_fake_uart

/* -------------------------------------------------------------------------- */
/* Configuration                                                              */
/* -------------------------------------------------------------------------- */

/*
 * Maximum number of TX operations whose buffers/lengths are recorded
 * for inspection by tests.
 *
 * After this limit, TX operations are still counted, but buffers
 * are no longer stored to avoid memory corruption during stress tests.
 */
#define FAKE_UART_MAX_TX_CAPTURE 16

/* -------------------------------------------------------------------------- */
/* Fake UART state                                                             */
/* -------------------------------------------------------------------------- */

/*
 * Registered UART callback and user data.
 * These are set by uart_callback_set() and used to emit events.
 */
static uart_callback_t fake_cb;
static void *fake_cb_data;

/*
 * TX capture state.
 *
 * tx_count:
 *   Total number of uart_tx() calls received since last reset.
 *
 * tx_bufs / tx_lens:
 *   Pointers and lengths of the first FAKE_UART_MAX_TX_CAPTURE
 *   transmissions. Used by tests to inspect framing behavior.
 *
 * IMPORTANT:
 *   These arrays are bounded. They never overflow.
 */
static const uint8_t *tx_bufs[FAKE_UART_MAX_TX_CAPTURE];
static size_t tx_lens[FAKE_UART_MAX_TX_CAPTURE];
static int tx_count;

/*
 * RX enable state.
 *
 * rx_enabled:
 *   True after uart_rx_enable() has been called.
 *
 * rx_timeout_ms:
 *   Stored for completeness, but not used (RX is synchronous).
 */
static bool rx_enabled;
static int32_t rx_timeout_ms;

/* -------------------------------------------------------------------------- */
/* Fake UART driver API                                                        */
/* -------------------------------------------------------------------------- */

/*
 * Set UART async callback.
 *
 * Stores callback and user data for later use when generating
 * UART events (TX_DONE, RX_RDY, TX_ABORTED).
 */
static int fake_uart_callback_set(const struct device *dev,
				  uart_callback_t cb,
				  void *user_data)
{
	ARG_UNUSED(dev);

	fake_cb = cb;
	fake_cb_data = user_data;
	return 0;
}

/*
 * Fake async TX.
 *
 * Behavior:
 *  - Record buffer pointer and length (bounded)
 *  - Increment total TX count
 *  - Immediately emit UART_TX_DONE
 *
 * This mimics a UART that completes TX instantly.
 */
static int fake_uart_tx(const struct device *dev,
			const uint8_t *buf,
			size_t len,
			int32_t timeout)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(timeout);

	/* Capture TX metadata for tests (bounded) */
	if (tx_count < FAKE_UART_MAX_TX_CAPTURE) {
		tx_bufs[tx_count] = buf;
		tx_lens[tx_count] = len;
	}

	tx_count++;

	/* Emit TX_DONE immediately */
	if (fake_cb) {
		struct uart_event evt = {
			.type = UART_TX_DONE,
			.data.tx.buf = buf,
			.data.tx.len = len,
		};

		fake_cb(dev, &evt, fake_cb_data);
	}

	return 0;
}

/*
 * Fake RX enable.
 *
 * Enables RX event delivery. No buffers are managed here;
 * RX data is injected explicitly by test helpers.
 */
static int fake_uart_rx_enable(const struct device *dev,
			       uint8_t *buf,
			       size_t len,
			       int32_t timeout)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(buf);
	ARG_UNUSED(len);

	rx_enabled = true;
	rx_timeout_ms = timeout;
	return 0;
}

/*
 * Fake RX disable.
 */
static int fake_uart_rx_disable(const struct device *dev)
{
	ARG_UNUSED(dev);

	rx_enabled = false;
	return 0;
}

/* -------------------------------------------------------------------------- */
/* UART driver API table                                                       */
/* -------------------------------------------------------------------------- */

static const struct uart_driver_api fake_uart_api = {
	.callback_set = fake_uart_callback_set,
	.tx = fake_uart_tx,
	.rx_enable = fake_uart_rx_enable,
	.rx_disable = fake_uart_rx_disable,
};

/* -------------------------------------------------------------------------- */
/* Device instantiation                                                        */
/* -------------------------------------------------------------------------- */

DEVICE_DT_DEFINE(DT_NODELABEL(fake_uart),
		 NULL,
		 NULL,
		 NULL,
		 NULL,
		 POST_KERNEL,
		 CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		 &fake_uart_api);

/* -------------------------------------------------------------------------- */
/* Test helper functions (not part of UART API)                                */
/* -------------------------------------------------------------------------- */

/*
 * Reset TX capture state.
 *
 * Used at the beginning of tests to ensure a clean slate.
 */
void fake_uart_reset_tx(void)
{
	tx_count = 0;
	memset(tx_bufs, 0, sizeof(tx_bufs));
	memset(tx_lens, 0, sizeof(tx_lens));
}

/*
 * Get total number of uart_tx() calls observed.
 *
 * This number may exceed FAKE_UART_MAX_TX_CAPTURE.
 */
int fake_uart_get_tx_count(void)
{
	return tx_count;
}

/*
 * Get pointer to TX buffer for a captured TX index.
 *
 * Returns NULL if index is out of capture range.
 */
const uint8_t *fake_uart_get_tx_buf(int idx)
{
	if (idx < 0 || idx >= FAKE_UART_MAX_TX_CAPTURE) {
		return NULL;
	}

	return tx_bufs[idx];
}

/*
 * Get TX length for a captured TX index.
 *
 * Returns 0 if index is out of capture range.
 */
size_t fake_uart_get_tx_len(int idx)
{
	if (idx < 0 || idx >= FAKE_UART_MAX_TX_CAPTURE) {
		return 0;
	}

	return tx_lens[idx];
}

/*
 * Inject an RX frame into the fake UART.
 *
 * This generates a UART_RX_RDY event if RX is enabled.
 * Data is delivered exactly as provided.
 */
void fake_uart_inject_rx(const uint8_t *data, size_t len)
{
	if (!rx_enabled || !fake_cb) {
		return;
	}

	struct uart_event evt = {
		.type = UART_RX_RDY,
		.data.rx.buf = (uint8_t *) data,
		.data.rx.len = len,
	};

	fake_cb(FAKE_UART, &evt, fake_cb_data);
}

/*
 * Inject a TX_ABORTED event.
 *
 * Used to test recovery paths in uart_mux.
 */
void fake_uart_inject_tx_abort(void)
{
	if (!fake_cb) {
		return;
	}

	struct uart_event evt = {
		.type = UART_TX_ABORTED,
	};

	fake_cb(FAKE_UART, &evt, fake_cb_data);
}

