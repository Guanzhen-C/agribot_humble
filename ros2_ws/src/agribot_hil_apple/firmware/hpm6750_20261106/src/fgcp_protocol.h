#ifndef FGCP_PROTOCOL_H
#define FGCP_PROTOCOL_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define FGCP_SERIAL_SIZE 32U
#define FGCP_VALID_MS 300U
typedef struct {
    int16_t linear_mm_s;
    int16_t angular_mrad_s;
    uint16_t sequence;
    uint32_t source_time_ms;
    uint16_t shredder_x100;
    uint8_t fan;
    uint8_t pump;
    uint8_t flags;
} fgcp_command_t;

typedef struct {
    uint8_t drive[8];
    bool has_drive;
    uint32_t drive_at;
} fgcp_can_parser_t;

bool fgcp_decode_serial(const uint8_t *data, size_t length, fgcp_command_t *out);
bool fgcp_decode_can(fgcp_can_parser_t *parser, uint16_t id,
                     const uint8_t *data, size_t length, bool extended,
                     bool remote, uint32_t now, fgcp_command_t *out);
uint8_t fgcp_crc8(const uint8_t *data, size_t length);
#endif
