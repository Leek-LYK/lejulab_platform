#!/bin/bash
#
# stark_sdk_aarch64 (bc-stark-sdk v1.1.9+) prebuilt fetcher.
#
# Usage:
#   ./download-lib.sh                      # download for current host arch
#   STARK_TARGET_ARCH=aarch64 ./download-lib.sh   # cross-download for board
#   STARK_TARGET_ARCH=x86_64  ./download-lib.sh
#   STARK_TARGET_ARCH=all     ./download-lib.sh   # fetch both archs
#
# Layout produced (headers are arch-independent, .so is split):
#   dist/include/stark-sdk.h
#   dist/include/zlgcan/*.h
#   dist/shared/linux/x86_64/libbc_stark_sdk.so
#   dist/shared/linux/aarch64/libbc_stark_sdk.so
#
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DIST_DIR="${SCRIPT_DIR}/dist"
VERSION_FILE="${SCRIPT_DIR}/VERSION"

LIB_VERSION="v1.1.9"
BASE_URL="https://app.brainco.cn/universal/bc-stark-sdk/libs/${LIB_VERSION}"

echo_y() { echo -e "\033[1;33m$*\033[0m"; }
echo_r() { echo -e "\033[0;31m$*\033[0m"; }
echo_g() { echo -e "\033[1;32m$*\033[0m"; }

# -------- Arch selection --------
TARGET_ARCH="${STARK_TARGET_ARCH:-$(uname -m)}"

declare -a ARCHS
case "$TARGET_ARCH" in
  all)                ARCHS=(x86_64 aarch64) ;;
  aarch64|arm64)      ARCHS=(aarch64) ;;
  x86_64|amd64)       ARCHS=(x86_64) ;;
  *)
    echo_r "Unsupported arch: $TARGET_ARCH (expected: x86_64|aarch64|all)"
    exit 1
    ;;
esac

# -------- Per-arch fetch --------
fetch_one() {
  local arch="$1"
  local zip_name target_dir target_lib
  case "$arch" in
    aarch64) zip_name="linux-arm64.zip" ;;
    x86_64)  zip_name="linux.zip" ;;
  esac
  target_dir="${DIST_DIR}/shared/linux/${arch}"
  target_lib="${target_dir}/libbc_stark_sdk.so"

  # Idempotent: skip if already at this version for this arch
  if [ -f "$target_lib" ] && [ -f "$VERSION_FILE" ] \
     && grep -Fq "${arch}:${LIB_VERSION}" "$VERSION_FILE"; then
    echo_g "[stark-sdk] ${LIB_VERSION}/${arch} already installed, skip"
    return 0
  fi

  if ! command -v wget >/dev/null 2>&1; then
    echo_r "wget is required"; exit 1
  fi
  if ! command -v unzip >/dev/null 2>&1; then
    echo_r "unzip is required"; exit 1
  fi

  local tmpdir
  tmpdir="$(mktemp -d)"
  trap 'rm -rf "$tmpdir"' RETURN

  local url="${BASE_URL}/${zip_name}?$(date +%s)"
  echo_y "[stark-sdk] downloading ${LIB_VERSION}/${arch} ..."
  wget -q --show-progress "$url" -O "${tmpdir}/${zip_name}" || {
    echo_r "download failed: $url"
    rm -rf "$tmpdir"
    exit 1
  }

  echo_y "[stark-sdk] extracting ${zip_name} ..."
  unzip -o -q "${tmpdir}/${zip_name}" -d "$tmpdir" || {
    echo_r "extract failed"
    rm -rf "$tmpdir"
    exit 1
  }
  rm -rf "${tmpdir}/__MACOSX" "${tmpdir}/dist/__MACOSX" 2>/dev/null || true

  # Place .so into arch-separated dir
  mkdir -p "$target_dir"
  cp -f "${tmpdir}/dist/shared/linux/libbc_stark_sdk.so" "$target_lib"

  # Headers are arch-independent; install once (keep the latest)
  mkdir -p "${DIST_DIR}/include"
  cp -f "${tmpdir}/dist/include/stark-sdk.h" "${DIST_DIR}/include/"
  if [ -d "${tmpdir}/dist/include/zlgcan" ]; then
    mkdir -p "${DIST_DIR}/include/zlgcan"
    cp -f "${tmpdir}"/dist/include/zlgcan/*.h "${DIST_DIR}/include/zlgcan/"
  fi

  # Update VERSION bookkeeping
  local tmpv="${VERSION_FILE}.tmp"
  {
    echo "# bc-stark-sdk prebuilt versions (managed by download-lib.sh)"
    echo "version: ${LIB_VERSION}"
    # Keep entries for other archs (if any), then add/refresh this arch
    if [ -f "$VERSION_FILE" ]; then
      grep -E '^[a-z0-9_]+:v[0-9]' "$VERSION_FILE" 2>/dev/null | grep -v "^${arch}:" || true
    fi
    echo "${arch}:${LIB_VERSION}  $(date '+%Y-%m-%d %H:%M:%S %Z')"
  } > "$tmpv"
  mv "$tmpv" "$VERSION_FILE"

  echo_g "[stark-sdk] installed ${LIB_VERSION}/${arch} -> ${target_lib}"
  rm -rf "$tmpdir"
  trap - RETURN
}

for a in "${ARCHS[@]}"; do
  fetch_one "$a"
done

echo_g "[stark-sdk] done"
