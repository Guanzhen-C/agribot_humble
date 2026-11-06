#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "hechuan_motion.h"
#include "vcu_machine_profile.h"

int main(void)
{
    const hechuan_velocity_t velocity = {1000, -500, 250};
    hechuan_velocity_t decoded = {0};
    hechuan_feedback_t decoded_feedback;
    uint8_t serial[11];
    uint8_t can[8];
    const uint8_t expected[] = {0x7b, 0, 0, 3, 0xe8, 0xfe, 0x0c, 0, 0xfa, 0x98, 0x7d};
    uint8_t feedback[] = {0x7b, 1, 3, 0xe8, 0xfe, 0x0c, 0, 0xfa,
        0, 1, 0xff, 0xfe, 0x40, 0, 0, 3, 0xff, 0xfc, 0, 5, 0x5d, 0xc0, 0, 0x7d};
    hechuan_encode_serial(&velocity, serial);
    assert(memcmp(serial, expected, sizeof(serial)) == 0);
    assert(hechuan_decode_serial(serial, sizeof(serial), &decoded));
    assert(decoded.forward_mm_s == 1000 && decoded.lateral_mm_s == -500 && decoded.yaw_mrad_s == 250);
    assert(!hechuan_decode_serial(serial, 10, &decoded));
    serial[9] ^= 1U;
    assert(!hechuan_decode_serial(serial, 11, &decoded));
    serial[9] ^= 1U;
    serial[1] = 0xffU; serial[9] ^= 0xffU;
    assert(!hechuan_decode_serial(serial, 11, &decoded));
    hechuan_encode_velocity_can(&velocity, can);
    assert(memcmp(can, expected + 3, 6) == 0 && can[6] == 0 && can[7] == 0);
    hechuan_encode_steering_can(1000, -250, can);
    assert(can[2] == 0 && can[3] == 0 && can[4] == 0xff && can[5] == 6);
    for (size_t i = 0; i < 22; ++i) { feedback[22] ^= feedback[i]; }
    assert(hechuan_decode_feedback(feedback, sizeof(feedback), &decoded_feedback));
    assert(decoded_feedback.drive_disabled && decoded_feedback.voltage_mv == 24000);
    assert(decoded_feedback.acceleration_raw[1] == -2 && decoded_feedback.acceleration_raw[2] == 16384);
    assert(decoded_feedback.angular_velocity_raw[1] == -4);
    feedback[22] ^= 1U;
    assert(!hechuan_decode_feedback(feedback, sizeof(feedback), &decoded_feedback));

    vcfg_write_request_t request = {0};
    const uint16_t models[] = {1, 0x10, 0x11, 0x12, 0x13, 0x14};
    for (size_t i = 0; i < sizeof(models) / sizeof(models[0]); ++i) {
        request.machine_id = models[i];
        request.protocol_id = models[i] == 1 ? 0x101 : (models[i] == 0x12 ? 0x202 : 0x201);
        for (uint32_t iface = 0; iface <= 2; ++iface) {
            request.compute_interface_id = iface;
            assert(vcu_validate_selection(&request) == VCFG_STATUS_OK);
        }
        assert(vcu_machine_can_bitrate(models[i]) == (models[i] == 1 ? 250000U : 1000000U));
    }
    request.compute_interface_id = 3;
    assert(vcu_validate_selection(&request) == VCFG_STATUS_UNSUPPORTED_COMPUTE_INTERFACE);
    request.compute_interface_id = 0; request.protocol_id = 0x202;
    assert(vcu_validate_selection(&request) == VCFG_STATUS_INCOMPATIBLE_SELECTION);
    request.protocol_id = 0xffff;
    assert(vcu_validate_selection(&request) == VCFG_STATUS_UNSUPPORTED_PROTOCOL);
    request.machine_id = 0xffff;
    assert(vcu_validate_selection(&request) == VCFG_STATUS_UNSUPPORTED_MACHINE);
    assert(vcu_machine_can_bitrate(0xffff) == 0);
    puts("hechuan motion/profile tests passed");
    return 0;
}
