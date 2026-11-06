#include "fgcp_protocol.h"
#include "vcfg_protocol.h"
#include <string.h>

static uint16_t u16(const uint8_t *p) { return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }
static uint32_t u32(const uint8_t *p) { return (uint32_t)u16(p) | ((uint32_t)u16(p + 2) << 16); }
static int32_t i32(const uint8_t *p) {
    uint32_t v = u32(p);
    return v <= INT32_MAX ? (int32_t)v : -1 - (int32_t)(UINT32_MAX - v);
}
static int16_t i16(const uint8_t *p) {
    uint16_t v = u16(p);
    return (int16_t)(v <= INT16_MAX ? (int32_t)v : (int32_t)v - 65536);
}
uint8_t fgcp_crc8(const uint8_t *data, size_t length)
{
    uint8_t crc = 0xffU;
    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; ++bit) {
            crc = (uint8_t)((crc << 1) ^ ((crc & 0x80U) ? 0x1dU : 0U));
        }
    }
    return crc ^ 0xffU;
}
bool fgcp_decode_serial(const uint8_t *data, size_t length, fgcp_command_t *out)
{
    if (length != FGCP_SERIAL_SIZE || data[0] != 'F' || data[1] != 'G'
        || data[2] != 1 || data[3] != 0x10 || u16(data + 4) != 32
        || u32(data + 28) != vcfg_crc32(data, 28)
        || u16(data + 20) > 7 || data[22] > 8 || data[23] > 8
        || u16(data + 24) > 10000 || u16(data + 26) != FGCP_VALID_MS) { return false; }
    int32_t linear = i32(data + 12), angular = i32(data + 16);
    /* Current motion adapters use int16 thousandths; reject, never truncate. */
    if (linear < INT16_MIN || linear > INT16_MAX || angular < INT16_MIN || angular > INT16_MAX) { return false; }
    *out = (fgcp_command_t){ .linear_mm_s = (int16_t)linear, .angular_mrad_s = (int16_t)angular,
        .sequence = u16(data + 6), .source_time_ms = u32(data + 8), .flags = data[20],
        .fan = data[22], .pump = data[23], .shredder_x100 = u16(data + 24) };
    return true;
}
bool fgcp_decode_can(fgcp_can_parser_t *parser, uint16_t id,
                     const uint8_t *data, size_t length, bool extended,
                     bool remote, uint32_t now, fgcp_command_t *out)
{
    if (id != 0x180 && id != 0x181) { return false; }
    if (extended || remote || length != 8 || data[6] != 30 || data[7] != fgcp_crc8(data, 7)) {
        parser->has_drive = false; return false;
    }
    if (id == 0x180) {
        parser->has_drive = data[4] <= 7;
        if (parser->has_drive) { memcpy(parser->drive, data, 8); parser->drive_at = now; }
        return false;
    }
    bool valid = parser->has_drive && now - parser->drive_at <= 50U
        && data[5] == parser->drive[5] && data[0] <= 8 && data[1] <= 8
        && data[4] == 0 && u16(data + 2) <= 10000;
    parser->has_drive = false;
    if (!valid) { return false; }
    *out = (fgcp_command_t){ .linear_mm_s = i16(parser->drive), .angular_mrad_s = i16(parser->drive + 2),
        .sequence = data[5], .flags = parser->drive[4], .fan = data[0], .pump = data[1],
        .shredder_x100 = u16(data + 2), .source_time_ms = 0 };
    return true;
}
