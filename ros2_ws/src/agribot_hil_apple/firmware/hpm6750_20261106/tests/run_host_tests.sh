#!/bin/sh
set -eu

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
SOURCE_DIR=$(CDPATH= cd -- "$SCRIPT_DIR/../src" && pwd)
TEST_BINARY="${TMPDIR:-/tmp}/vcu_configurator_tests"

cc -std=c11 -Wall -Wextra -Werror -pedantic \
    -I"$SOURCE_DIR" \
    "$SOURCE_DIR/vcfg_protocol.c" \
    "$SOURCE_DIR/vcu_selection_store.c" \
    "$SCRIPT_DIR/test_vcu_config.c" \
    -o "$TEST_BINARY"

"$TEST_BINARY"

cc -std=c11 -Wall -Wextra -Werror -pedantic \
    -I"$SOURCE_DIR" \
    "$SOURCE_DIR/hechuan_motion.c" \
    "$SOURCE_DIR/vcu_machine_profile.c" \
    "$SCRIPT_DIR/test_hechuan_motion.c" \
    -o "${TEST_BINARY}_motion"

"${TEST_BINARY}_motion"

PROFILE_TEST_DIR=$(mktemp -d)
node "$SCRIPT_DIR/../../../host_app/scripts/profile-fixtures.mjs" "$PROFILE_TEST_DIR"
cc -std=c11 -Wall -Wextra -Werror -pedantic \
    -I"$SOURCE_DIR" "$SOURCE_DIR/fgmp_profile.c" "$SOURCE_DIR/vcfg_protocol.c" \
    "$SCRIPT_DIR/test_fgmp_profile.c" -lm -o "$PROFILE_TEST_DIR/profile_tests"
"$PROFILE_TEST_DIR/profile_tests" "$PROFILE_TEST_DIR"

cc -std=c11 -Wall -Wextra -Werror -pedantic \
    -I"$SOURCE_DIR" "$SOURCE_DIR/vcu_bridge.c" "$SOURCE_DIR/vcu_telemetry.c" \
    "$SOURCE_DIR/vcfg_protocol.c" "$SOURCE_DIR/hechuan_motion.c" \
    "$SCRIPT_DIR/test_vcu_bridge.c" -o "${TEST_BINARY}_bridge"
"${TEST_BINARY}_bridge"
