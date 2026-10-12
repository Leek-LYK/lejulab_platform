#!/bin/bash
# DDS RPC 首次调用压力测试
# 每次启动新进程，测试首次建立连接的延迟
#
# Usage: ./rpc_first_call_stress.sh [count] [--visualize]
#
# 需要先在另一个终端启动 calculator_server

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../../../../.." && pwd)"

source "$WS_DIR/devel/setup.bash"
export CYCLONEDDS_URI=file://$WS_DIR/src/leju_launch/config/cyclonedds_shm.xml

COUNT=${1:-100}
VISUALIZE=false
CSV_FILE="/tmp/rpc_first_call_latency.csv"

for arg in "$@"; do
    case $arg in
        --visualize) VISUALIZE=true ;;
        [0-9]*) COUNT=$arg ;;
    esac
done

echo "=== DDS RPC First Call Stress Test ==="
echo "Test count: $COUNT"
echo "Output: $CSV_FILE"
echo ""

echo "iteration,total_us,discovery_us,rpc_us,status" > $CSV_FILE

SUCCESS=0
TIMEOUT=0
FAIL=0

for i in $(seq 1 $COUNT); do
    OUTPUT=$(rosrun lejusdk-dds-rpc rpc_first_call_test --count 1 2>&1)

    if echo "$OUTPUT" | grep -qF "] OK"; then
        TOTAL=$(echo "$OUTPUT" | grep -F "] OK" | grep -oE "total=[0-9]+" | sed 's/total=//')
        DISCOVERY=$(echo "$OUTPUT" | grep -F "] OK" | grep -oE "discovery=[0-9]+" | sed 's/discovery=//')
        RPC=$(echo "$OUTPUT" | grep -F "] OK" | grep -oE "rpc=[0-9]+" | sed 's/rpc=//')
        echo "$i,$TOTAL,$DISCOVERY,$RPC,OK" >> $CSV_FILE
        echo "[$i/$COUNT] OK  total=${TOTAL}us discovery=${DISCOVERY}us rpc=${RPC}us"
        ((SUCCESS++))
    elif echo "$OUTPUT" | grep -qF "TIMEOUT"; then
        echo "$i,0,0,0,TIMEOUT" >> $CSV_FILE
        echo "[$i/$COUNT] TIMEOUT"
        ((TIMEOUT++))
    else
        echo "$i,0,0,0,FAIL" >> $CSV_FILE
        echo "[$i/$COUNT] FAIL"
        ((FAIL++))
    fi
done

echo ""
echo "=== Results ==="
echo "Success: $SUCCESS / $COUNT ($(echo "scale=1; $SUCCESS * 100 / $COUNT" | bc)%)"
echo "Timeout: $TIMEOUT / $COUNT"
echo "Failed:  $FAIL / $COUNT"
echo ""
echo "Data saved to: $CSV_FILE"

if [ "$VISUALIZE" = true ]; then
    python3 << 'PYEOF'
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

df = pd.read_csv('/tmp/rpc_first_call_latency.csv')
ok = df[df['status'] == 'OK']

if len(ok) == 0:
    print("No successful calls")
    exit(0)

print("")
print("=== Latency Statistics ===")
print(f"Count:   {len(ok)}")
print(f"Min:     {ok['total_us'].min():.0f} us")
print(f"Max:     {ok['total_us'].max():.0f} us")
print(f"Mean:    {ok['total_us'].mean():.0f} us")
print(f"Median:  {ok['total_us'].median():.0f} us")
print(f"P90:     {ok['total_us'].quantile(0.90):.0f} us")
print(f"P99:     {ok['total_us'].quantile(0.99):.0f} us")
print("")
print("=== Breakdown ===")
print(f"Discovery: {ok['discovery_us'].mean():.0f} us ({100*ok['discovery_us'].mean()/ok['total_us'].mean():.1f}%)")
print(f"RPC:       {ok['rpc_us'].mean():.0f} us ({100*ok['rpc_us'].mean()/ok['total_us'].mean():.1f}%)")

fig, axes = plt.subplots(2, 2, figsize=(12, 8))

ax1 = axes[0, 0]
ax1.bar(range(len(ok)), ok['discovery_us'], label='Discovery', color='orange')
ax1.bar(range(len(ok)), ok['rpc_us'], bottom=ok['discovery_us'], label='RPC', color='green')
ax1.set_xlabel('Call Index')
ax1.set_ylabel('Latency (us)')
ax1.set_title('Latency Breakdown')
ax1.legend()
ax1.grid(True, alpha=0.3)

ax2 = axes[0, 1]
ax2.hist(ok['total_us'], bins=30, color='blue', edgecolor='black', alpha=0.7)
ax2.axvline(ok['total_us'].mean(), color='red', linestyle='--', label=f"Mean: {ok['total_us'].mean():.0f}us")
ax2.set_xlabel('Total Latency (us)')
ax2.set_ylabel('Count')
ax2.set_title('Latency Distribution')
ax2.legend()
ax2.grid(True, alpha=0.3)

ax3 = axes[1, 0]
ax3.scatter(ok['discovery_us'], ok['rpc_us'], alpha=0.6)
ax3.set_xlabel('Discovery (us)')
ax3.set_ylabel('RPC (us)')
ax3.set_title('Discovery vs RPC')
ax3.grid(True, alpha=0.3)

ax4 = axes[1, 1]
status = df['status'].value_counts()
colors = {'OK': '#2ecc71', 'TIMEOUT': '#e74c3c', 'FAIL': '#95a5a6'}
ax4.pie(status.values, labels=[f"{s}\n{v}" for s,v in zip(status.index, status.values)],
        colors=[colors.get(s,'gray') for s in status.index])
ax4.set_title('Status')

plt.suptitle('DDS RPC First Call Test', fontsize=14, fontweight='bold')
plt.tight_layout()
plt.savefig('/tmp/rpc_first_call_latency.png', dpi=150)
print("")
print("Chart: /tmp/rpc_first_call_latency.png")
PYEOF
else
    python3 << 'PYEOF'
import pandas as pd
df = pd.read_csv('/tmp/rpc_first_call_latency.csv')
ok = df[df['status'] == 'OK']
if len(ok) > 0:
    print("")
    print("=== Latency Statistics ===")
    print(f"Min:    {ok['total_us'].min():.0f} us")
    print(f"Max:    {ok['total_us'].max():.0f} us")
    print(f"Mean:   {ok['total_us'].mean():.0f} us")
    print(f"Median: {ok['total_us'].median():.0f} us")
PYEOF
fi
