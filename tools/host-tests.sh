#!/usr/bin/env bash
#
# All host tests in one place: every C test twice (-Werror, then ASan+UBSan
# with recovery off) and every Python fault-injection test. CI (build.yml and
# release.yml) and local runs call this script, so the gate cannot drift from
# what a developer runs. Before 2026-11-06 the workflows listed three of the
# six C tests and none of the Python ones.
#
# Usage: tools/host-tests.sh            (from the repo root; needs gcc, python3)
#
# ASan on recent kernels (vm.mmap_rnd_bits=32, e.g. WSL2 and ubuntu-24.04
# runners) can loop on "AddressSanitizer:DEADLYSIGNAL" before main. Sanitizer
# binaries therefore run under `setarch -R` (ASLR off) when the environment
# allows it; that changes nothing the tests check.
set -u
cd "$(dirname "$0")/.."

OUT="$(mktemp -d -t rtk-host-tests.XXXXXX)"
trap 'rm -rf "$OUT"' EXIT
rc=0

WARN="-std=c99 -O2 -Wall -Wextra -Werror -Imain/include -Imain/include/interface"
SAN="-std=c99 -O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=all -Imain/include -Imain/include/interface"

RUN_SAN=()
if setarch "$(uname -m)" -R true >/dev/null 2>&1; then
    RUN_SAN=(setarch "$(uname -m)" -R)
fi

ctest() {
    local name=$1; shift
    local mode flags bin r
    for mode in warn san; do
        if [ "$mode" = warn ]; then flags=$WARN; else flags=$SAN; fi
        bin="$OUT/$name.$mode"
        if ! gcc $flags -o "$bin" "$@" -lm 2>"$bin.cc"; then
            echo "FAIL $name/$mode: compile"; head -30 "$bin.cc"; rc=1; continue
        fi
        if [ "$mode" = san ]; then
            # Without setarch the ASLR failure is random per run: retry, but
            # only when the run died in that loop before printing anything
            # of its own (never to paper over a real sanitizer report).
            local attempt
            for attempt in 1 2 3 4 5; do
                timeout 60 "${RUN_SAN[@]}" "$bin" >"$bin.out" 2>&1; r=$?
                # Retry only when the output is NOTHING but that marker: the
                # test printed nothing of its own, so it never ran. Any other
                # line (a real report, a test message) ends the loop.
                [ $r -ne 0 ] && [ -s "$bin.out" ] && \
                    ! grep -qv '^AddressSanitizer:DEADLYSIGNAL$' "$bin.out" || break
                echo "     $name/$mode: ASan ASLR startup failure, retry $attempt"
            done
        else
            timeout 300 "$bin" >"$bin.out" 2>&1; r=$?
        fi
        if [ $r -eq 0 ]; then
            echo "ok   $name/$mode: $(tail -1 "$bin.out" | head -c 160)"
        else
            echo "FAIL $name/$mode: exit $r"; head -c 4000 "$bin.out"; echo; rc=1
        fi
    done
}

ctest gnss_ack       tests/host/test_gnss_ack.c main/gnss_ack.c
ctest gnss_capture   tests/host/test_gnss_capture.c main/gnss_capture.c main/gnss_ack.c
ctest ntrip_tx_core  tests/host/test_ntrip_tx_core.c main/interface/ntrip_tx_core.c
ctest ntrip_conn     tests/host/test_ntrip_conn.c main/interface/ntrip_conn.c
ctest telemetry_json tests/host/test_telemetry_json.c main/telemetry_json.c
ctest gnss_site      tests/host/test_gnss_site.c main/gnss_site.c
ctest rtcm_obs       tests/host/test_rtcm_observations.c main/interface/ntrip_tx_core.c

for p in tests/host/test_*.py; do
    if python3 "$p" >"$OUT/py.out" 2>&1; then
        echo "ok   $(basename "$p"): $(tail -1 "$OUT/py.out" | head -c 160)"
    else
        echo "FAIL $(basename "$p")"; tail -30 "$OUT/py.out"; rc=1
    fi
done

if [ $rc -eq 0 ]; then echo "host tests: all passed"; else echo "host tests: FAILED"; fi
exit $rc
