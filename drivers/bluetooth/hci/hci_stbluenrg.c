/* hci_stbluenrg.c - HCI driver for stm32wb0x based on BlueNRG-LPx */

/*
 * Copyright (c) 2017 Linaro Ltd.
 * Copyright (c) 2024 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Licenses TBD ST+Linaro */

#include <bluenrg_lp_stack.h>
#undef MIN
#undef MAX
#include <zephyr/init.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/bluetooth/hci_driver.h>
#include <ble_controller.h>
#include <rf_driver_hal_vtimer.h>
#include <rf_driver_ll_timer.h>
#include <rf_driver_ll_rcc.h>
#include <rf_driver_ll_bus.h>
#include <miscutil.h>
#include <hal_miscutil.h>
#include <pka_manager.h>
#include <aes_manager.h>
#include <rng_manager.h>
#include "DTM_config.h"
#include "DTM_cmd_db.h"
#include "dm_alloc.h"
#include "aci_adv_nwk.h"

#define LOG_LEVEL CONFIG_BT_HCI_DRIVER_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bt_driver);
/**
* @brief Max HS startup time expressed in system time (1953 us / 2.4414 us)
*/
#define MAX_HS_STARTUP_TIME	0x320
#define BLE_WKUP_PRIO		0
#define BLE_WKUP_FLAGS		0	 /* IRQ flags TBD */
#define BLE_TX_RX_PRIO		0
#define BLE_TX_RX_FLAGS		0	 /* IRQ flags TBD */
#define CPU_WKUP_PRIO		1
#define CPU_WKUP_FLAGS		0	 /* IRQ flags TBD */
#define BLE_ERROR_PRIO		3
#define BLE_ERROR_FLAGS		0	 /* IRQ flags TBD */
#define BLE_RXTX_SEQ_PRIO	3
#define BLE_RXTX_SEQ_FLAGS	0	 /* IRQ flags TBD */

#define LL_EVENT_HANDLER_PRIO		4
#define LL_EVENT_HANDLER_STACKSIZE	1024
#define MAX_EVENT_SIZE			259
#define MAX_ISO_DATA_LOAD_LENGTH	512

#define HCI_CMD			0x01
#define HCI_ACL			0x02
#define HCI_EVT			0x04
#define HCI_ISO			0x05

#define PACKET_TYPE		0
#define EVT_HEADER_TYPE		0
#define EVT_HEADER_EVENT	1
#define EVT_HEADER_SIZE		2
#define EVT_LE_META_SUBEVENT	3
#define EVT_VENDOR_CODE_LSB	3
#define EVT_VENDOR_CODE_MSB	4

static uint32_t dyn_alloc_a[DYNAMIC_MEMORY_SIZE>>2];
static uint8_t buffer_out_mem[MAX_EVENT_SIZE];
static struct k_thread ll_event_handler_thread_data;
K_THREAD_STACK_DEFINE(ll_event_handler_stack, LL_EVENT_HANDLER_STACKSIZE);

static struct net_buf *get_rx(uint8_t *msg);

uint32_t __noinit aci_gatt_adv_nwk_buffer[ACI_GATT_ADV_NWK_BUFFER_SIZE_CONF>>2];

BLEPLAT_NvmStatusTypeDef BLEPLAT_NvmGet(BLEPLAT_NvmSeekModeTypeDef a, BLEPLAT_NvmRecordTypeDef b, uint16_t c, uint8_t *d, uint16_t e)
{
	return 0;
}

static void ll_event_handler(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (true) {
		HAL_VTIMER_Tick();
		BLE_STACK_Tick();
		k_msleep(1); /* arbitrary value */
	}
}

/* Process Commands */
static uint16_t process_command(uint8_t *buffer, uint16_t buffer_in_length, uint8_t *buffer_out, uint16_t buffer_out_max_length)
{
	uint32_t i;
	uint16_t ret_val;
	uint16_t op_code;
	uint8_t *buffer_in = buffer + sizeof(struct bt_hci_cmd_hdr);
	struct bt_hci_cmd_hdr *hdr = (struct bt_hci_cmd_hdr *) buffer;

	buffer_in_length -= sizeof(struct bt_hci_cmd_hdr);
	op_code = hdr->opcode;
#if 0
	if (op_code == 0x0c03) {
		// For HCI_RESET, reset the system, so that Controller, Timer module and HEAP are reinitialized.
		RAM_VR.Reserved[0] = 0x01; // Remember to send a command complete after reset is completed.
		reset_pending = 1;
		return 0;
	}
#endif
	ret_val = 0; // check_legacy_extended_call(op_code, buffer_out);// CHECK function definition
	if(ret_val != 0){
		return ret_val;
	}

	for (i = 0; hci_command_table[i].opcode != 0; i++) {
		if (op_code == hci_command_table[i].opcode) {
			ret_val = hci_command_table[i].execute(buffer_in, buffer_in_length, buffer_out, buffer_out_max_length);
			/* add get crash handler */
			return ret_val;
		}
	}


	// Unknown command length
	buffer_out[0] = 0x04;
	buffer_out[1] = 0x0F;
	buffer_out[2] = 0x04;
	buffer_out[3] = 0x01;
	buffer_out[4] = 0x01;
	HOST_TO_LE_16(buffer_out+5, op_code);
	return 7;
}

void send_event(uint8_t *buffer_out, uint16_t buffer_out_length, int8_t overflow_index)
{
	ARG_UNUSED(buffer_out_length);
	ARG_UNUSED(overflow_index);
	/* Construct net_buf from event data */
	struct net_buf *buf = get_rx(buffer_out);
	if (buf) {
		/* Handle the received HCI data */
		LOG_DBG("New event %p len %u type %u", buf, buf->len, bt_buf_get_type(buf));
		bt_recv(buf);
	} else {
		LOG_ERR("Buf is null");
	}
}

ISR_DIRECT_DECLARE(BLE_WKUP_IRQHandler)
{
	HAL_VTIMER_WakeUpCallback();
	ISR_DIRECT_PM(); /* PM done after servicing interrupt for best latency */
	return 1; /* We should check if scheduling decision should be made */
}

/* irq_count used for the aci_hal_transmitter_test_packets_process() command implementation */
uint32_t irq_count = 1;
uint16_t num_packets = 0;

ISR_DIRECT_DECLARE(BLE_TX_RX_IRQHandler)
{
	uint32_t blue_status = BLUE->STATUSREG;
	uint32_t blue_interrupt = BLUE->INTERRUPT1REG;
	/** clear all pending interrupts */
	BLUE->INTERRUPT1REG = blue_interrupt;
 
	HAL_VTIMER_EndOfRadioActivityIsr();
	BLE_STACK_RadioHandler(blue_status|blue_interrupt);
	HAL_VTIMER_RadioTimerIsr();
	if(irq_count != num_packets)
	{
		irq_count++;
	}

	/* If the device is configured with 
		 System clock = 64 MHz and BLE clock = 16 MHz
		 a register read is necessary to end fine
		 the clear interrupt register operation,
		 due the AHB down converter latency */ 
	blue_interrupt = BLUE->INTERRUPT1REG;
	ISR_DIRECT_PM(); /* PM done after servicing interrupt for best latency TBD */
	return 1; /* We should check if scheduling decision should be made TBD */
}

ISR_DIRECT_DECLARE(BLE_RXTX_SEQ_IRQHandler)
{
	HAL_RXTX_SEQ_IRQHandler();
	ISR_DIRECT_PM(); /* PM done after servicing interrupt for best latency TBD */
	return 1; /* We should check if scheduling decision should be made TBD */
}

ISR_DIRECT_DECLARE(CPU_WKUP_IRQHandler)
{
	HAL_VTIMER_TimeoutCallback();
	ISR_DIRECT_PM(); /* PM done after servicing interrupt for best latency TBD */
	return 1; /* We should check if scheduling decision should be made TBD */
}

ISR_DIRECT_DECLARE(BLE_ERROR_IRQHandler)
{
	volatile uint32_t debug_cmd;

	BLUE->DEBUGCMDREG |= 1;
	/* If the device is configured with 
		 System clock = 64 MHz and BLE clock = 16 MHz
		 a register read is necessary to end fine
		 the clear interrupt register operation,
		 due the AHB down converter latency */ 
	debug_cmd = BLUE->DEBUGCMDREG;
	ISR_DIRECT_PM(); /* PM done after servicing interrupt for best latency TBD */
	return 1; /* We should check if scheduling decision should be made TBD */
}

static void ble_isr_installer(void)
{
	IRQ_DIRECT_CONNECT(BLE_WKUP_IRQn, BLE_WKUP_PRIO, BLE_WKUP_IRQHandler, BLE_WKUP_FLAGS);
	irq_enable(BLE_WKUP_IRQn);
	IRQ_DIRECT_CONNECT(BLE_TX_RX_IRQn, BLE_TX_RX_PRIO, BLE_TX_RX_IRQHandler, BLE_TX_RX_FLAGS);
	irq_enable(BLE_TX_RX_IRQn);
	IRQ_DIRECT_CONNECT(CPU_WKUP_IRQn, CPU_WKUP_PRIO, CPU_WKUP_IRQHandler, CPU_WKUP_FLAGS);
	irq_enable(CPU_WKUP_IRQn);
	IRQ_DIRECT_CONNECT(BLE_SEQ_IRQn, BLE_RXTX_SEQ_PRIO, BLE_RXTX_SEQ_IRQHandler, BLE_RXTX_SEQ_FLAGS);
	irq_enable(BLE_SEQ_IRQn);
	IRQ_DIRECT_CONNECT(BLE_ERROR_IRQn, BLE_ERROR_PRIO, BLE_ERROR_IRQHandler, BLE_ERROR_FLAGS);
	irq_enable(BLE_ERROR_IRQn);
}

static struct net_buf *get_rx(uint8_t *msg)
{
	bool discardable = false;
	k_timeout_t timeout = K_FOREVER;
	struct net_buf *buf;
	int len;

	switch (msg[PACKET_TYPE]) {
	case HCI_EVT:
		switch (msg[EVT_HEADER_EVENT]) {
		case BT_HCI_EVT_VENDOR:
			/* Run event through interface handler */
			//if (bt_spi_handle_vendor_evt(msg)) {
				return NULL;
			//}
			/* Event has not yet been handled */
			__fallthrough;
		default:
			if (msg[EVT_HEADER_EVENT] == BT_HCI_EVT_LE_META_EVENT &&
			    (msg[EVT_LE_META_SUBEVENT] == BT_HCI_EVT_LE_ADVERTISING_REPORT)) {
				discardable = true;
				timeout = K_NO_WAIT;
			}
			buf = bt_buf_get_evt(msg[EVT_HEADER_EVENT],
					     discardable, timeout);
			if (!buf) {
				LOG_DBG("Discard adv report due to insufficient buf");
				return NULL;
			}
		}

		len = sizeof(struct bt_hci_evt_hdr) + msg[EVT_HEADER_SIZE];
		if (len > net_buf_tailroom(buf)) {
			LOG_ERR("Event too long: %d", len);
			net_buf_unref(buf);
			return NULL;
		}
		net_buf_add_mem(buf, &msg[1], len);
		break;
	case HCI_ACL:
		struct bt_hci_acl_hdr acl_hdr;
		buf = bt_buf_get_rx(BT_BUF_ACL_IN, timeout);
		memcpy(&acl_hdr, &msg[1], sizeof(acl_hdr));
		len = sizeof(acl_hdr) + sys_le16_to_cpu(acl_hdr.len);
		if (len > net_buf_tailroom(buf)) {
			LOG_ERR("ACL too long: %d", len);
			net_buf_unref(buf);
			return NULL;
		}
		net_buf_add_mem(buf, &msg[1], len);
		break;
	case HCI_ISO:

		struct bt_hci_iso_hdr iso_hdr;

		buf = bt_buf_get_rx(BT_BUF_ISO_IN, timeout);

		if (buf) {
			memcpy(&iso_hdr, &msg[1], sizeof(iso_hdr));
			len = sizeof(iso_hdr) + sys_le16_to_cpu(iso_hdr.len);
		} else {
			LOG_ERR("No available ISO buffers!");
			return NULL;
		}
		if (len > net_buf_tailroom(buf)) {
			LOG_ERR("ISO too long: %d", len);
			net_buf_unref(buf);
			return NULL;
		}
		net_buf_add_mem(buf, &msg[1], len);
		break;
	default:
		LOG_ERR("Unknown BT buf type %d", msg[0]);
		return NULL;
	}

	return buf;
}

static int bt_hci_stbluenrg_send(struct net_buf *buf)
{
	int ret = 0;
	uint8_t *hci_buffer = buf->data;

	switch (bt_buf_get_type(buf)) {
	case BT_BUF_ACL_OUT:
	{
		uint16_t connection_handle;
		uint16_t data_len;
		uint8_t* pdu;
		uint8_t	pb_flag;
		uint8_t	bc_flag;

		connection_handle = ((hci_buffer[1] & 0x0F) << 8) + hci_buffer[0];
		data_len = (hci_buffer[3] << 8) + hci_buffer[2];
		pdu = hci_buffer+4;
		pb_flag = (hci_buffer[1] >> 4) & 0x3;
		bc_flag = (hci_buffer[1] >> 6) & 0x3;
		hci_tx_acl_data(connection_handle, pb_flag, bc_flag, data_len, pdu);
		break;
	}
#if defined(CONFIG_BT_ISO)
	case BT_BUF_ISO_OUT:
	{
		uint16_t connection_handle;
		uint16_t iso_data_load_len;
		uint8_t* iso_data_load;
		uint8_t	pb_flag;
		uint8_t	ts_flag;

		connection_handle = LE_TO_HOST_16(hci_buffer) & 0x0FFF;
		iso_data_load_len = LE_TO_HOST_16(hci_buffer+2) & 0x3FFF;
		pb_flag = (hci_buffer[1] >> 4) & 0x3;
		ts_flag = (hci_buffer[1] >> 6) & 0x1;
		iso_data_load = &hci_buffer[4];
		hci_tx_iso_data(connection_handle, pb_flag, ts_flag, iso_data_load_len, iso_data_load);
		break;
	}
#endif /* CONFIG_BT_ISO */
	case BT_BUF_CMD:
		process_command(hci_buffer, buf->len, buffer_out_mem, sizeof(buffer_out_mem));
		send_event(buffer_out_mem, 0, 0);
		break;
	default:
		LOG_ERR("Unsupported type");
		return -EINVAL;
	}
	net_buf_unref(buf);

	return ret;
}

static int bt_hci_stbluenrg_open(void)
{
	k_tid_t tid;
	HAL_VTIMER_InitType VTIMER_InitStruct = {MAX_HS_STARTUP_TIME, 0, 0};
	BLE_STACK_InitTypeDef BLE_STACK_InitParams = BLE_STACK_INIT_PARAMETERS;

	LL_RCC_SetRFClockSource(LL_RCC_RF_RC64MPLL_DIV4);
	LL_APB2_EnableClock(LL_APB2_PERIPH_MRBLE);
	BLECNTR_InitGlobal();

	HAL_VTIMER_Init(&VTIMER_InitStruct);

	BLEPLAT_Init();

	PKAMGR_Init();
	RNGMGR_Init();


	AESMGR_Init();

	BLE_STACK_Init(&BLE_STACK_InitParams);
	dm_init(ACI_GATT_ADV_NWK_BUFFER_SIZE_CONF, aci_gatt_adv_nwk_buffer);
	aci_adv_nwk_init();
	ble_isr_installer();

	tid = k_thread_create(&ll_event_handler_thread_data, ll_event_handler_stack,
			K_KERNEL_STACK_SIZEOF(ll_event_handler_stack), ll_event_handler, NULL, NULL,
			NULL, LL_EVENT_HANDLER_PRIO, 0, K_NO_WAIT);
	k_thread_name_set(tid, "LL_EVENT_HANDLER");

	return 0;
}

static const struct bt_hci_driver drv = {
	.name					 = "BT WB0x",
	.bus					 = BT_HCI_DRIVER_BUS_IPM,
	.open					 = bt_hci_stbluenrg_open,
	.send					 = bt_hci_stbluenrg_send,
	.quirks					 = BT_QUIRK_NO_RESET,
};

static int bt_hci_stbluenrg_init(void)
{
	bt_hci_driver_register(&drv);

	return 0;
}

SYS_INIT(bt_hci_stbluenrg_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
