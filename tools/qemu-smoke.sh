#!/usr/bin/env bash
#
# QEMU boot smoke test: boot the built image in qemu-system-xtensa and assert
# that app_main actually gets through early bring-up.
#
# Why this exists (2026-08-01): a one-line reordering in app_main moved an
# ESP_LOGI ahead of drv_uart_usbc_init(), so the log path took a semaphore that
# did not exist yet. xSemaphoreTake(NULL) trips configASSERT and the device
# panicked on EVERY boot, before the anti-brick boot-loop guard could even count
# it. The firmware compiled cleanly, the host tests passed, and a code review
# had signed it off. Only running it catches this class, and every field device
# runs exactly this sequence.
#
# Scope: this proves the image BOOTS and reaches the banner. QEMU's esp32
# machine does not emulate the WiFi radio, so the firmware ALWAYS panics inside
# phy_init (LoadStorePIFAddrError on the radio registers at 0x600xxxxx). That is
# a QEMU limitation, not a defect, so the log is truncated at the first phy_init
# line and only the prefix before it is judged. That prefix still covers
# config/NVS, RTCM ingest init, uart_init, the boot-loop guard, app_main's
# banner and the software half of wifi_init. It is a boot gate, not a functional
# test: no UM980, no caster, no radio.
#
# Usage: tools/qemu-smoke.sh [build-dir]      (run inside the ESP-IDF env)
# Env:   QEMU_SMOKE_TIMEOUT_S (default 60)
set -euo pipefail

BUILD_DIR="${1:-build}"
TIMEOUT_S="${QEMU_SMOKE_TIMEOUT_S:-60}"
FLASH_BIN="$(mktemp -t qemu-smoke-flash.XXXXXX.bin)"
LOG="$(mktemp -t qemu-smoke-log.XXXXXX.txt)"
trap 'rm -f "$FLASH_BIN" "$LOG"' EXIT

if [ ! -f "$BUILD_DIR/flash_args" ]; then
    echo "qemu-smoke: no $BUILD_DIR/flash_args; run idf.py build first" >&2
    exit 2
fi

echo "qemu-smoke: merging flash image from $BUILD_DIR"
( cd "$BUILD_DIR" && python -m esptool --chip esp32 merge_bin \
    --fill-flash-size 16MB -o "$FLASH_BIN" @flash_args ) >/dev/null

echo "qemu-smoke: booting (timeout ${TIMEOUT_S}s)"
# QEMU exits via timeout, so a non-zero status here is expected and ignored;
# the assertions below are the gate.
#
# Serial goes to a FILE, not stdio. With -nographic QEMU multiplexes the
# monitor and the serial port onto stdio, and when the container has a TTY
# (the esp-idf-ci-action runs `docker run -t`) nothing reaches a redirected
# stdout: the CI run on 2026-08-03 captured zero bytes and the assertions
# below then reported a phantom "early crash loop". -display none + -monitor
# none + -serial file: is TTY-independent, and stdin is closed so QEMU can
# never wait on a terminal.
timeout "$TIMEOUT_S" qemu-system-xtensa \
    -machine esp32 -m 4M -display none -monitor none \
    -serial file:"$LOG" \
    -drive file="$FLASH_BIN",if=mtd,format=raw </dev/null >/dev/null 2>&1 || true

fail() { echo "qemu-smoke: FAIL: $1" >&2; echo "--- judged boot prefix (last 40 lines) ---" >&2; tail -40 "$PREFIX" >&2; exit 1; }

# Judge only the boot prefix up to the emulator's WiFi-radio wall (see Scope).
PREFIX="$(mktemp -t qemu-smoke-prefix.XXXXXX.txt)"
trap 'rm -f "$FLASH_BIN" "$LOG" "$PREFIX"' EXIT
# Match the phy_init LOG TAG ("phy_init: phy_version ..."), not the bare word:
# the bootloader's partition-table dump also lists a partition called phy_init.
sed '/phy_init: /q' "$LOG" >"$PREFIX"

# 0. No output at all is an environment failure, NOT a firmware verdict. Saying
#    "early crash loop" for an empty log sent us hunting a firmware bug that did
#    not exist (2026-08-03); name the real problem instead.
if [ ! -s "$LOG" ]; then
    echo "qemu-smoke: FAIL: qemu produced NO serial output in ${TIMEOUT_S}s." >&2
    echo "  This is an emulator/environment problem, not a firmware verdict." >&2
    echo "  Check that qemu-system-xtensa exists and supports '-machine esp32'." >&2
    qemu-system-xtensa --version 2>&1 | head -2 >&2 || echo "  qemu-system-xtensa not runnable" >&2
    exit 1
fi

# 1. A panic, assert or abort before the radio wall is fatal.
if grep -qE 'Guru Meditation|assert failed|abort\(\) was called|CORRUPT HEAP|stack protection fault' "$PREFIX"; then
    fail "panic/assert during early boot: $(grep -oE 'Guru Meditation[^\r]*|assert failed[^\r]*|abort\(\) was called[^\r]*' "$PREFIX" | head -1)"
fi

# 2. Exactly one reset in the prefix. A second "rst:0x" means the image rebooted
#    before it even reached WiFi, i.e. an early crash loop (the first
#    POWERON_RESET is the normal one).
resets=$(grep -c 'rst:0x' "$PREFIX" || true)
if [ "$resets" -ne 1 ]; then
    fail "expected 1 reset before WiFi, saw $resets (early crash loop)"
fi

# 3. app_main must reach the banner, which sits AFTER config/NVS init, the RTCM
#    ingest init, uart_init and the boot-loop guard. That is the whole window
#    the 2026-08-01 defect lived in.
grep -q 'RTKdata Station' "$PREFIX" || fail "never reached the app_main banner"

# 4. The boot-loop guard must have run. It sits after ntrip_server_ingest_init()
#    AND uart_init(), so seeing it proves both came up.
#    (Deliberately NOT asserting the "RTCM ingest ready" line: ingest_init runs
#    before the console UART exists, so by design that line only reaches the
#    web-log ring buffer, never this serial output.)
grep -q 'boot-loop guard' "$PREFIX" || fail "never reached the boot-loop guard (ingest/uart bring-up)"

echo "qemu-smoke: PASS (banner reached, ingest up, no panic, single boot)"
