# stark_sdk_aarch64 / examples / revo2
#
# Standalone build & run for BrainCo Revo2 — classic CAN demo.
#
# Quick usage:
#   make                      # build revo2_can_ctrl.exe for host arch
#   make run_revo2_can_ctrl   # build + run (SocketCAN default)
#   make clean
#
# Runtime env (set before `make run_*`):
#   STARK_CAN_BACKEND=socketcan|zlg   (default: socketcan)
#   STARK_SOCKETCAN_IFACE=can0        (socketcan interface)
#   STARK_ZLG_LIB_PATH=/path/libusbcanfd.so   (optional, ZLG only)
#

.DEFAULT_GOAL := all

# -------- Paths (arch-aware) --------
SCRIPT_DIR := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
SDK_ROOT   := $(abspath $(SCRIPT_DIR)/../..)
DIST_DIR   := $(SDK_ROOT)/dist
HEADER_DIR := $(DIST_DIR)/include
COMMON_DIR := $(abspath $(SCRIPT_DIR)/../common)

UNAME_M := $(shell uname -m)
ARCH    := $(if $(filter arm64,$(UNAME_M)),aarch64,$(UNAME_M))
LIB_DIR := $(DIST_DIR)/shared/linux/$(ARCH)
LIB_SO  := $(LIB_DIR)/libbc_stark_sdk.so

ifeq ($(wildcard $(LIB_SO)),)
$(error [examples/revo2] Prebuilt library not found at $(LIB_SO). Run $(SDK_ROOT)/download-lib.sh (STARK_TARGET_ARCH=$(ARCH)))
endif

# -------- Toolchain --------
CXX      := g++
CXXFLAGS := -g -Wall -std=c++17
# Both CAN backends compiled in; runtime selects via STARK_CAN_BACKEND
CAN_DEFS := -DSTARK_USE_ZLG=1 -DSTARK_USE_SOCKETCAN=1

INCLUDE  := -I$(HEADER_DIR) -I$(COMMON_DIR)

RPATH_DIRS := $(LIB_DIR):/usr/local/lib:/usr/lib
LDFLAGS    := -L$(LIB_DIR) -Wl,-rpath,$(RPATH_DIRS) -lbc_stark_sdk -ldl -lm -lpthread

# -------- Sources --------
COMMON_OBJS := $(COMMON_DIR)/stark_common.o $(COMMON_DIR)/can_common.o
TARGETS     := revo2_can_ctrl.exe

# -------- Rules --------
.PHONY: all clean help
all: $(TARGETS)
	@printf "\033[1;32m[examples/revo2] built for arch=%s\033[0m\n" "$(ARCH)"

$(COMMON_DIR)/stark_common.o: $(COMMON_DIR)/stark_common.cpp $(COMMON_DIR)/stark_common.h
	$(CXX) $(CXXFLAGS) $(CAN_DEFS) -c -o $@ $< $(INCLUDE)
	@printf "\033[1;32m[common] built stark_common.o\033[0m\n"

$(COMMON_DIR)/can_common.o: $(COMMON_DIR)/can_common.cpp $(COMMON_DIR)/can_common.h
	$(CXX) $(CXXFLAGS) $(CAN_DEFS) -c -o $@ $< $(INCLUDE)
	@printf "\033[1;32m[common] built can_common.o\033[0m\n"

revo2_can_ctrl.exe: revo2_can_ctrl.cpp $(COMMON_OBJS)
	$(CXX) $(CXXFLAGS) $(CAN_DEFS) -o $@ $< $(COMMON_OBJS) $(INCLUDE) $(LDFLAGS)
	@printf "\033[1;32m[Built] %s (arch=%s)\033[0m\n" "$@" "$(ARCH)"

# -------- Run --------
.PHONY: run_revo2_can_ctrl
run_revo2_can_ctrl: revo2_can_ctrl.exe
	@printf "\033[1;36m[Running] revo2_can_ctrl.exe  arch=%s  backend=%s  iface=%s\033[0m\n" \
		"$(ARCH)" "$${STARK_CAN_BACKEND:-socketcan}" "$${STARK_SOCKETCAN_IFACE:-can0}"
	LD_LIBRARY_PATH=$(LIB_DIR):$$LD_LIBRARY_PATH ./revo2_can_ctrl.exe

# -------- Utils --------
clean:
	rm -f *.exe $(COMMON_OBJS)

help:
	@echo "stark_sdk_aarch64 / examples / revo2"
	@echo ""
	@echo "Targets:"
	@echo "  make                     build for host arch ($(ARCH))"
	@echo "  make run_revo2_can_ctrl  build + run classic CAN demo"
	@echo "  make clean               remove binaries + objects"
	@echo ""
	@echo "Runtime env vars:"
	@echo "  STARK_CAN_BACKEND=socketcan|zlg   (default: socketcan)"
	@echo "  STARK_SOCKETCAN_IFACE=can0        (socketcan iface)"
	@echo "  STARK_ZLG_LIB_PATH=/path/libusbcanfd.so   (zlg only)"
	@echo ""
	@echo "Paths:"
	@echo "  SDK_ROOT    = $(SDK_ROOT)"
	@echo "  LIB_SO      = $(LIB_SO)"
	@echo "  HEADER_DIR  = $(HEADER_DIR)"
