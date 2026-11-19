#!/usr/bin/env bash
# GEArm PC 侧桥接启动脚本（Ubuntu 24.04 / 任意 POSIX 发行版）。
# 只用标准库：不需要 pip install，也不需要虚拟环境。
set -euo pipefail
cd "$(dirname "$0")"
exec python3 bridge/serial_bridge.py "$@"
