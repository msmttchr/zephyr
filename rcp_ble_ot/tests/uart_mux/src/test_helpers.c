#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "test_helpers.h"

extern void fake_uart_inject_rx(const uint8_t *data, size_t len);

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

void inject_garbage(size_t len)
{
	uint8_t buf[32];

	if (len > sizeof(buf)) {
		len = sizeof(buf);
	}

	for (size_t i = 0; i < len; i++) {
		buf[i] = rand() & 0xFF;
	}

	fake_uart_inject_rx(buf, len);
}

void inject_valid_frame(uint8_t plc, const uint8_t *payload, size_t len)
{
	uint8_t frame[64];
	size_t pos = 0;

	frame[pos++] = 0xC0;
	frame[pos++] = 0x02;
	frame[pos++] = len & 0xFF;
	frame[pos++] = len >> 8;
	frame[pos++] = plc;

	memcpy(&frame[pos], payload, len);
	pos += len;

	patch_crc_to_frame(frame, pos + 2);
	pos = pos + 2;

	fake_uart_inject_rx(frame, pos);
}

void patch_crc_to_frame(uint8_t* frame, uint16_t len)
{
	uint16_t crc = 0xFFFF;
	for (size_t i = 0; i < (len - 2); i++) {
		crc = crc16_ccitt_update(crc, frame[i]);
	}
	crc ^= 0xFFFF;
	frame[len - 2] = crc & 0xFF;
	frame[len - 1] = crc >> 8;
}

