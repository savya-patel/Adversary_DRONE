#!/bin/bash

# Analyze Cartographer logs for diagnostics

cd ~/Adversary_DRONE/slam_ws

echo "========================================="
echo "  Cartographer Log Analysis"
echo "========================================="
echo ""

LATEST_LOG=$(ls -t logs/cartographer_*.log | head -1)

if [ -z "$LATEST_LOG" ]; then
    echo "No log files found in logs/"
    exit 1
fi

echo "Analyzing: $LATEST_LOG"
echo ""

# 1. Check scan rate
echo "--- Scan Rate ---"
grep "scan rate:" "$LATEST_LOG" | tail -5
echo ""

# 2. Check submaps created
echo "--- Submaps Created ---"
grep "Inserted submap" "$LATEST_LOG"
echo ""

# 3. Check match quality (scores)
echo "--- Match Score Summary (last occurrence) ---"
grep "Score histogram" -A 12 "$LATEST_LOG" | tail -15
echo ""

# 4. Check translation/rotation errors (last 10)
echo "--- Translation/Rotation Errors (last 10) ---"
grep "differs by translation" "$LATEST_LOG" | tail -10
echo ""

# 5. Motion filter stats
echo "--- Motion Filter ---"
grep "Motion filter" "$LATEST_LOG"
echo ""

# 6. Check for warnings/errors
echo "--- Warnings ---"
grep -i "warn\|error" "$LATEST_LOG" | head -10
echo ""

echo "========================================="
echo "Full log: $LATEST_LOG"
echo "========================================="
