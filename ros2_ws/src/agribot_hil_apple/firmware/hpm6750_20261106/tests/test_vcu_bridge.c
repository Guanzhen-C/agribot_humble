#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "vcfg_protocol.h"
#include "vcu_bridge.h"
#include "vcu_machine_profile.h"
#include "vcu_telemetry.h"

static uint32_t u32(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }
static uint16_t u16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static int32_t s32(const uint8_t *p) { return (int32_t)u32(p); }

static void ackermann_feedback(uint8_t frames[3][8], int16_t voltage, int16_t speed, int16_t yaw, uint8_t stop)
{
    uint8_t body[24] = {0x7B, stop};
    body[2] = (uint8_t)((uint16_t)speed >> 8); body[3] = (uint8_t)speed;
    body[6] = (uint8_t)((uint16_t)yaw >> 8); body[7] = (uint8_t)yaw;
    body[20] = (uint8_t)((uint16_t)voltage >> 8); body[21] = (uint8_t)voltage;
    for (unsigned i = 0; i < 22; ++i) { body[22] ^= body[i]; }
    body[23] = 0x7D;
    memcpy(frames, body, 24);
}

static void xor_close(uint8_t d[8]) { d[7] = 0; for (unsigned i = 0; i < 7; ++i) { d[7] ^= d[i]; } }

int main(void)
{
    /* Mode follows the selected protocol and compute interface. */
    assert(vcu_select_mode(VCU_PROTOCOL_STEERING, VCFG_COMPUTE_INTERFACE_CAN) == VCU_MODE_BRIDGE);
    assert(vcu_select_mode(VCU_PROTOCOL_STEERING, VCFG_COMPUTE_INTERFACE_SOCKETCAN) == VCU_MODE_BRIDGE);
    assert(vcu_select_mode(VCU_PROTOCOL_IMPLEMENT, VCFG_COMPUTE_INTERFACE_CAN) == VCU_MODE_BRIDGE);
    assert(vcu_select_mode(VCU_PROTOCOL_STEERING, VCFG_COMPUTE_INTERFACE_SERIAL) == VCU_MODE_ADAPTER);
    assert(vcu_select_mode(VCU_PROTOCOL_MOTION, VCFG_COMPUTE_INTERFACE_CAN) == VCU_MODE_ADAPTER);
    assert(vcu_bridge_profile(VCU_PROTOCOL_STEERING)->bitrate == 1000000U);
    assert(vcu_bridge_profile(VCU_PROTOCOL_IMPLEMENT)->bitrate == 250000U);
    assert(vcu_bridge_profile(VCU_PROTOCOL_MOTION) == NULL);

    /* Ackermann: 0x181 bytes 4..5 are front-wheel angle, forwarded unchanged. */
    vcu_bridge_t b;
    assert(vcu_bridge_init(&b, VCU_PROTOCOL_STEERING));
    uint8_t cmd[8] = {0xFE, 0xD4, 0, 0, 0x01, 0x80, 0, 0};  /* -300 mm/s, 384 mrad */
    assert(!vcu_bridge_from_compute(&b, 0x180, cmd, true, 10));
    assert(!vcu_bridge_from_compute(&b, 0x181, cmd, false, 10));
    assert(b.dropped_frames == 1 && !b.has_command);
    assert(vcu_bridge_from_compute(&b, 0x181, cmd, true, 10));
    vcu_bridge_sent(&b, true, true);
    assert(b.target[0] == -300 && b.target[1] == 384 && b.command_forwarded == 1);

    uint8_t fb[3][8];
    ackermann_feedback(fb, 24000, 120, -50, 1);
    assert(!vcu_bridge_from_machine(&b, 0x104, fb[0], true, 20));
    for (unsigned i = 0; i < 3; ++i) {
        assert(vcu_bridge_from_machine(&b, 0x101 + i, fb[i], true, 20 + i));
        vcu_bridge_sent(&b, false, true);
    }
    assert(b.feedback_forwarded == 3 && b.has_motion && b.voltage_mv == 24000);
    assert(b.speed_mm_s == 120 && b.yaw_mrad_s == -50 && b.drive_disabled == 1);
    vcu_bridge_sent(&b, true, false);
    assert(b.dropped_frames == 2 && (vcu_bridge_fault(&b, 30) & 32U));
    vcu_bridge_sent(&b, true, true);
    assert(vcu_bridge_fault(&b, 30) == 0);
    assert(vcu_bridge_fault(&b, 10 + 301) & 2U);  /* command timeout after 300 ms */

    /* Telemetry v2: layout read by agribot vcu_monitor.decode_telemetry. */
    uint8_t t[VCU_TELEMETRY_SIZE];
    vcu_encode_bridge_telemetry(&b, VCU_MACHINE_STEERING, VCU_PROTOCOL_STEERING,
        VCFG_COMPUTE_INTERFACE_CAN, true, true, 40, 42, t);
    assert(memcmp(t, "VCFG\x01\xa0", 6) == 0 && u16(t + 6) == 64 && u32(t + 8) == 42);
    assert(u32(t + 76) == vcfg_crc32(t, 76));
    const uint8_t *p = t + 12;
    assert(p[0] == 2 && p[1] == 15 && u16(p + 2) == 0 && u16(p + 4) == 18 && u16(p + 6) == 514 && p[8] == 1);
    assert(u16(p + 10) == 3 && u32(p + 12) == 40 && u32(p + 16) == 30 && u32(p + 20) == 18);
    assert(s32(p + 24) == 24000 && s32(p + 28) == 120 && s32(p + 32) == -50);
    assert(s32(p + 36) == -300 && s32(p + 40) == 384);
    assert(p[48] == 255 && p[49] == 1);
    assert(u32(p + 52) == 2 && u32(p + 56) == 2 && u32(p + 60) == 3);
    vcu_encode_bridge_telemetry(&b, 18, 514, 1, true, true, 1000, 43, t);
    assert(p[1] == 9 && u16(p + 10) == 0 && s32(p + 24) == 0 && s32(p + 36) == 0 && p[49] == 255);

    /* Identity: byte-identical to agribot test reply (version 1.0.0, flags 3, 18/514, seq 42). */
    uint8_t id[VCU_IDENTITY_SIZE];
    vcu_encode_identity(18, 514, true, true, 42, id);
    const uint8_t expected_head[] = {'V','C','F','G',1,0xa1,8,0,42,0,0,0,1,0,0,3,18,0,0x02,0x02};
    assert(memcmp(id, expected_head, sizeof(expected_head)) == 0 && u32(id + 20) == vcfg_crc32(id, 20));
    uint8_t query[24] = {'V','C','F','G',1,0x21,8,0,7,0,0,0};
    uint32_t crc = vcfg_crc32(query, 20), seq = 0;
    for (unsigned i = 0; i < 4; ++i) { query[20 + i] = (uint8_t)(crc >> (8 * i)); }
    assert(vcu_decode_query(query, VCU_IDENTITY_QUERY_OPCODE, &seq) && seq == 7);
    assert(!vcu_decode_query(query, VCU_QUERY_OPCODE, &seq));

    /* Three-in-one tracked: 0x514 percents, 0x532 voltage/mode/e-stop, 0x533/0x534 rpm. */
    assert(vcu_bridge_init(&b, VCU_PROTOCOL_IMPLEMENT));
    uint8_t track[8] = {0, 0xF6, 30, 0, 0, 0, 5, 0}; xor_close(track);  /* -10 %, 30 % */
    assert(!vcu_bridge_from_compute(&b, 0x181, track, true, 0));
    assert(vcu_bridge_from_compute(&b, 0x514, track, true, 0));
    assert(b.target[0] == -10 && b.target[1] == 30);
    uint8_t chassis[8] = {0x05, 0, 0xF0, 0x00, 0, 0, 1, 0}; xor_close(chassis);  /* mode 1, e-stop, 24.0 V */
    uint8_t left[8] = {0, 0x38, 0xFF, 0, 0, 0, 1, 0}; xor_close(left);  /* -200 rpm */
    uint8_t right[8] = {0, 0xC8, 0x00, 0, 0, 0, 1, 0}; xor_close(right);  /* 200 rpm */
    assert(vcu_bridge_from_machine(&b, 0x532, chassis, true, 100));
    assert(vcu_bridge_from_machine(&b, 0x533, left, true, 100));
    assert(vcu_bridge_from_machine(&b, 0x534, right, true, 100));
    vcu_encode_bridge_telemetry(&b, 1, 257, 2, true, true, 500, 1, t);
    assert(p[1] == 15 && u16(p + 10) == 13 && s32(p + 24) == 24000);
    assert(s32(p + 36) == -10 && s32(p + 40) == 30);
    assert((int16_t)u16(p + 44) == -200 && (int16_t)u16(p + 46) == 200 && p[48] == 1 && p[49] == 1);
    right[7] ^= 1;  /* corrupted payload is still forwarded verbatim, but never decoded */
    assert(vcu_bridge_from_machine(&b, 0x534, right, true, 700));
    vcu_encode_bridge_telemetry(&b, 1, 257, 2, true, true, 701, 2, t);
    assert(!(u16(p + 10) & 8));

    puts("native CAN bridge tests passed");
    return 0;
}
