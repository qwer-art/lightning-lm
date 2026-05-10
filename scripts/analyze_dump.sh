#!/bin/bash
#
# Lightning-LM dump 数据分析入口脚本
#
# 用法:
#   ./analyze_dump.sh
#
# 分析结果输出到 DUMP_DIR 同级的 _analysis 目录，不影响原始数据

set -e

DUMP_DIR="./data/dump_output"

if [ ! -d "$DUMP_DIR" ]; then
    echo "Error: $DUMP_DIR is not a directory"
    exit 1
fi

if [ ! -f "$DUMP_DIR/update_state.txt" ]; then
    echo "Error: $DUMP_DIR/update_state.txt not found"
    exit 1
fi

OUTPUT_DIR="${DUMP_DIR}/analysis"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PY_SCRIPT="$SCRIPT_DIR/analyze_dump.py"

if [ ! -f "$PY_SCRIPT" ]; then
    echo "Error: $PY_SCRIPT not found"
    exit 1
fi

echo "=========================================="
echo "Lightning-LM Dump Analysis"
echo "=========================================="
echo "Input:  $DUMP_DIR"
echo "Output: $OUTPUT_DIR"
echo "=========================================="

python3 "$PY_SCRIPT" "$DUMP_DIR" "$OUTPUT_DIR"

echo ""
echo "Analysis results saved to: $OUTPUT_DIR"
echo "  - trajectory_analysis.png"
echo "  - bg_convergence.png"
echo "  - health_overview.png"
echo "  - analysis.md"