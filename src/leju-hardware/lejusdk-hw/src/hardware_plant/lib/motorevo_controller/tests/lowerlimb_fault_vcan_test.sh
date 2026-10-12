#!/usr/bin/env bash
# CAN FD per-motor health integration test on vcan. No physical CAN is used.
# Usage: sudo bash lowerlimb_fault_vcan_test.sh [test_binary]
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../../../../../../.." && pwd)"
CLI="${1:-$REPO_ROOT/build/src/leju-hardware/lejusdk-hw/src/hardware_plant/lib/motorevo_controller/tests/test_lowerlimb_fault_vcan}"
RESPONDER="$SCRIPT_DIR/lowerlimb_fault_vcan_responder.py"
IFACE=vbcan0
RESPONDER_PID=""

stop_responder() {
    if [[ -n "$RESPONDER_PID" ]]; then
        kill "$RESPONDER_PID" 2>/dev/null || true
        wait "$RESPONDER_PID" 2>/dev/null || true
        RESPONDER_PID=""
    fi
}

cleanup() {
    stop_responder
    ip link del "$IFACE" 2>/dev/null || true
}
trap cleanup EXIT

[[ -x "$CLI" && -s "$CLI" ]] || { echo "test binary missing or empty: $CLI" >&2; exit 2; }
file "$CLI" | grep -q 'ELF' || { echo "test binary is not an ELF executable: $CLI" >&2; exit 2; }
[[ -f "$RESPONDER" ]] || { echo "responder not found: $RESPONDER" >&2; exit 2; }

modprobe vcan || { echo "modprobe vcan failed (run with sudo)" >&2; exit 2; }
ip link del "$IFACE" 2>/dev/null || true
ip link add dev "$IFACE" type vcan || exit 2
ip link set "$IFACE" mtu 72 || exit 2
ip link set up "$IFACE" || exit 2

run_scenario() {
    local test_scenario="$1"
    local responder_scenario="$test_scenario"
    [[ "$test_scenario" == "normal_lifecycle" ]] && responder_scenario="healthy"

    echo
    echo "========== scenario: $test_scenario =========="
    python3 "$RESPONDER" "$IFACE" 1,2 1 "$responder_scenario" 0.5 &
    RESPONDER_PID=$!
    sleep 0.2

    "$CLI" "$IFACE" "$test_scenario"
    local result=$?
    stop_responder
    if (( result != 0 )); then
        echo "scenario failed: $test_scenario" >&2
        exit "$result"
    fi
    sleep 0.1
}

run_scenario fault
run_scenario disabled
run_scenario single_timeout
run_scenario bus_timeout
run_scenario normal_lifecycle

echo
echo "ALL VCAN MOTOR HEALTH SCENARIOS PASSED"
