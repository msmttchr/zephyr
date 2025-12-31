#pragma once
#include <stddef.h>
#include <stdint.h>

void inject_garbage(size_t len);
void inject_valid_frame(uint8_t plc, const uint8_t *payload, size_t len);
void patch_crc_to_frame(uint8_t* frame, uint16_t len);
