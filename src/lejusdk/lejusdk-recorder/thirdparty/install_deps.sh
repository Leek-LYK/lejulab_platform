#!/bin/bash
# Script to download and install third-party dependencies

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "=== Installing Third-party Dependencies ==="
echo ""

# MCAP
echo "[1/3] Installing MCAP library..."
cd "$SCRIPT_DIR/mcap"
if [ -d "temp" ]; then
    rm -rf temp
fi
git clone --depth 1 https://github.com/foxglove/mcap.git temp
rm -rf include
cp -r temp/cpp/mcap/include .
rm -rf temp
echo "MCAP installed successfully."
echo ""

# nlohmann-json
echo "[2/3] Installing nlohmann-json..."
cd "$SCRIPT_DIR/nlohmann-json"
mkdir -p include/nlohmann
curl -sL https://github.com/nlohmann/json/releases/download/v3.11.3/json.hpp -o include/nlohmann/json.hpp
echo "nlohmann-json installed successfully."
echo ""

# optionparser
echo "[3/3] Installing optionparser..."
cd "$SCRIPT_DIR/optionparser"
mkdir -p include
# Download from a reliable source
curl -sL "https://sourceforge.net/projects/optionparser/files/optionparser-1.7.tar.gz/download" -o op.tar.gz
tar -xzf op.tar.gz
cp optionparser-1.7/src/optionparser.h include/
rm -rf optionparser-1.7 op.tar.gz
echo "optionparser installed successfully."
echo ""

echo "=== All dependencies installed successfully! ==="
echo ""
echo "You can now build the project:"
echo "  mkdir build && cd build"
echo "  cmake .."
echo "  make -j\$(nproc)"
