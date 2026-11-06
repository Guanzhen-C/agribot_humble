#include "fgmp_profile.h"
#include "vcfg_protocol.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct { unsigned up, down; bool fail; uint16_t id[128]; uint8_t data[128][8]; } capture_t;
static fgmp_engine_t engine;
static uint8_t package[FGMP_MAX_PACKAGE];
static size_t package_length;
static capture_t capture;

static bool send_frame(void *context, bool upstream, uint16_t id, const uint8_t data[8])
{
    capture_t *out = context;
    if (out->fail) { return false; }
    if (upstream) { out->up++; return true; }
    assert(out->down < 128); out->id[out->down] = id; memcpy(out->data[out->down++], data, 8);
    return true;
}
static void seal(fgmp_can_frame_t *frame)
{
    frame->data[7] = 0;
    for (size_t i = 0; i < 7; i++) { frame->data[7] ^= frame->data[i]; }
}
static fgmp_can_frame_t drive(int left, int right, bool light, unsigned counter)
{
    fgmp_can_frame_t f = {.id = 0x514, .length = 8};
    f.data[1] = (uint8_t)left; f.data[2] = (uint8_t)right; f.data[3] = light; f.data[6] = counter & 15U; seal(&f); return f;
}
static fgmp_can_frame_t feedback(unsigned flags, unsigned counter)
{
    fgmp_can_frame_t f = {.id = 0x532, .length = 8}; f.data[0] = flags; f.data[6] = counter & 15U; seal(&f); return f;
}
static fgmp_status_t receive(bool up, fgmp_can_frame_t *f, uint32_t now)
{
    return fgmp_engine_receive(&engine, up, f, now, send_frame, &capture);
}
static void load(const char *directory, const char *name)
{
    char path[1024]; assert(snprintf(path, sizeof(path), "%s/%s", directory, name) > 0);
    FILE *file = fopen(path, "rb"); assert(file);
    package_length = fread(package, 1, sizeof(package), file); assert(feof(file)); fclose(file);
    assert(fgmp_engine_init(&engine, package, package_length) == FGMP_OK);
    memset(&capture, 0, sizeof(capture));
}
static void enable_at(uint32_t now)
{
    fgmp_engine_set_permit(&engine, true);
    fgmp_can_frame_t state = feedback(1, 0); assert(receive(false, &state, now) == FGMP_OK);
    fgmp_can_frame_t stop = drive(0, 0, false, 0); assert(receive(true, &stop, now) == FGMP_OK);
}
static int16_t be16(const uint8_t *data)
{
    unsigned n = ((unsigned)data[0] << 8) | data[1]; return (int16_t)(n > 32767 ? (int)n - 65536 : (int)n);
}
static void package_crc(void)
{
    uint32_t crc = vcfg_crc32(package + 16, package_length - 16);
    for (unsigned i = 0; i < 4; i++) { package[12 + i] = (uint8_t)(crc >> (8 * i)); }
}

int main(int argc, char **argv)
{
    assert(argc == 2);
    load(argv[1], "native-draft.fgmp");
    assert(engine.profile.source_length > 0 && !engine.profile.commissioned);
    fgmp_engine_set_permit(&engine, true); assert(!engine.permit);
    fgmp_can_frame_t state = feedback(1, 0), stop = drive(0, 0, false, 0);
    assert(receive(false, &state, 0) == FGMP_OK); assert(receive(true, &stop, 0) == FGMP_INTERLOCK); assert(capture.down == 0);
    package[80] ^= 1; assert(fgmp_engine_init(&engine, package, package_length) == FGMP_BAD_PACKAGE); assert(!engine.initialized);

    load(argv[1], "native-test.fgmp"); enable_at(0);
    fgmp_can_frame_t move = drive(50, 60, true, 1);
    move.data[4] = 0x55; seal(&move); /* Reserved source bytes are preserved, not reinterpreted. */
    assert(receive(true, &move, 10) == FGMP_OK); assert(capture.down == 2);
    assert(capture.id[1] == 0x514 && memcmp(capture.data[1], move.data, 8) == 0);
    assert(receive(true, &move, 11) == FGMP_SEQUENCE); assert(capture.down == 2);
    assert(engine.command_at[0] == 10 && !engine.primed[0]);
    assert(fgmp_engine_tick(&engine, 510, send_frame, &capture) == FGMP_TIMEOUT);
    stop = drive(0, 0, false, 0); assert(receive(true, &stop, 511) == FGMP_OK);
    move = drive(1, 1, false, 1); move.extended = true;
    assert(receive(true, &move, 512) == FGMP_BAD_FRAME);
    move.extended = false; move.data[7] ^= 1; assert(receive(true, &move, 512) == FGMP_BAD_FRAME);
    move = drive(101, 0, false, 1); assert(receive(true, &move, 512) == FGMP_RANGE);
    move = drive(0, 0, false, 1); assert(receive(false, &move, 512) == FGMP_UNSUPPORTED);

    load(argv[1], "native-test.fgmp"); enable_at(0);
    state = feedback(5, 1); assert(receive(false, &state, 10) == FGMP_OK); assert(!engine.primed[0]);
    move = drive(30, 30, false, 1); assert(receive(true, &move, 11) == FGMP_INTERLOCK);
    state = feedback(1, 2); assert(receive(false, &state, 12) == FGMP_OK);
    move = drive(30, 30, false, 2); assert(receive(true, &move, 13) == FGMP_INTERLOCK);
    stop = drive(0, 0, false, 3); assert(receive(true, &stop, 14) == FGMP_OK);
    capture.fail = true; move = drive(30, 30, false, 4); assert(receive(true, &move, 15) == FGMP_IO); assert(!engine.permit);

    load(argv[1], "native-test.fgmp"); enable_at(UINT32_MAX - 200U);
    move = drive(10, 10, false, 1); assert(receive(true, &move, 20) == FGMP_OK);
    assert(fgmp_engine_tick(&engine, 520, send_frame, &capture) == FGMP_TIMEOUT);

    load(argv[1], "native-test.fgmp"); enable_at(0);
    state = feedback(1, 0); assert(receive(false, &state, 100) == FGMP_SEQUENCE);
    assert(engine.feedback_at[0] == 0 && !engine.primed[0]);

    load(argv[1], "ackermann-test.fgmp"); enable_at(0);
    assert(fgmp_engine_tick(&engine, 0, send_frame, &capture) == FGMP_OK); assert(capture.down == 2);
    move = drive(50, 50, true, 1); assert(receive(true, &move, 100) == FGMP_OK);
    assert(fgmp_engine_tick(&engine, 100, send_frame, &capture) == FGMP_OK);
    assert(capture.id[2] == 0x181 && be16(capture.data[2]) == 1000 && be16(capture.data[2] + 4) == 0);
    const uint8_t lamps[8] = {15,0,0,0,0,0,1,14}; assert(memcmp(capture.data[3], lamps, 8) == 0);
    assert(fgmp_engine_tick(&engine, 600, send_frame, &capture) == FGMP_TIMEOUT);
    assert(be16(capture.data[4]) == 0 && capture.data[5][0] == 0);
    move = drive(50, 50, false, 2); assert(receive(true, &move, 610) == FGMP_INTERLOCK);
    stop = drive(0, 0, false, 3); assert(receive(true, &stop, 620) == FGMP_OK);
    move = drive(50, -50, false, 4); assert(receive(true, &move, 630) == FGMP_UNSUPPORTED);
    stop = drive(0, 0, false, 4); assert(receive(true, &stop, 640) == FGMP_OK);
    move = drive(-50, -50, false, 5); assert(receive(true, &move, 650) == FGMP_OK);
    assert(fgmp_engine_tick(&engine, 700, send_frame, &capture) == FGMP_OK); assert(be16(capture.data[6]) == -1000);
    move = drive(40, 60, false, 6); assert(receive(true, &move, 710) == FGMP_OK);
    assert(engine.values[0][8] > engine.values[0][9] && engine.values[0][8] <= 600);
    fgmp_can_frame_t work = {.id = 0x580, .length = 8}; work.data[0] = 1; seal(&work);
    assert(receive(true, &work, 711) == FGMP_UNSUPPORTED);

    load(argv[1], "ackermann-test.fgmp");
    package[128 + 16 + 20 + 1] = 0; package_crc(); /* Recomputed CRC cannot make overlapping fields valid. */
    assert(fgmp_engine_init(&engine, package, package_length) == FGMP_BAD_PACKAGE);
    load(argv[1], "ackermann-test.fgmp"); package[65] = 255; package_crc();
    assert(fgmp_engine_init(&engine, package, package_length) == FGMP_BAD_PACKAGE);
    load(argv[1], "ackermann-test.fgmp"); package[128 + 16] = 99; package_crc();
    assert(fgmp_engine_init(&engine, package, package_length) == FGMP_BAD_PACKAGE);

    load(argv[1], "differential-test.fgmp"); enable_at(0);
    move = drive(40, 60, false, 1); assert(receive(true, &move, 100) == FGMP_OK);
    assert(fgmp_engine_tick(&engine, 100, send_frame, &capture) == FGMP_OK);
    assert(be16(capture.data[0]) == 1000 && be16(capture.data[0] + 4) == 400);
    memset(work.data, 0, 8); seal(&work); assert(receive(true, &work, 101) == FGMP_OK);
    work.data[0] = 0x32; work.data[1] = 0xd2; work.data[2] = 4; work.data[6] = 1; seal(&work);
    assert(receive(true, &work, 102) == FGMP_OK);
    assert(fgmp_engine_tick(&engine, 200, send_frame, &capture) == FGMP_OK);
    assert(capture.id[3] == 0x212 && capture.data[3][0] == 0x32 && capture.data[3][1] == 0xd2 && capture.data[3][2] == 4);
    puts("PASS: one C executable, three runtime profiles; original forwarding, Ackermann, lamps, work, interlocks, timeout, corruption and wraparound (offline only)");
    return 0;
}
