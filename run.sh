#!/usr/bin/env bash
set -euo pipefail

LOG="agent_$(date +%Y%m%d_%H%M%S).log"

echo "Logging to $LOG"

# Merge stderr into stdout, pipe both through tee → file + terminal.
cargo run 2>&1 | tee "$LOG"
