# TinySDP-CT planner app for the Crazyflie 2.x (out-of-tree app layer).
# Builds against this repository's crazyflie-firmware submodule (../../crazyflie-firmware).
#   make            -> build/cf21bl.bin (Crazyflie 2.1 Brushless; brushed: make OOT_CONFIG=$(CURDIR)/app-config-cf2 -> cf2.bin)
#   make cload      -> flash over radio (set CLOAD_CMDS="-w radio://0/80/2M/E7E7E7E7E7")
CRAZYFLIE_BASE ?= $(CURDIR)/../../crazyflie-firmware

# On macOS, Crazyflie's merge_config.sh requires GNU readlink/sed/cp (Homebrew coreutils, gnu-sed).
COREUTILS_GNUBIN := $(firstword $(wildcard /opt/homebrew/opt/coreutils/libexec/gnubin) $(wildcard /usr/local/opt/coreutils/libexec/gnubin))
GNU_SED_GNUBIN := $(firstword $(wildcard /opt/homebrew/opt/gnu-sed/libexec/gnubin) $(wildcard /usr/local/opt/gnu-sed/libexec/gnubin))
ifneq ($(COREUTILS_GNUBIN),)
export PATH := $(COREUTILS_GNUBIN):$(PATH)
endif
ifneq ($(GNU_SED_GNUBIN),)
export PATH := $(GNU_SED_GNUBIN):$(PATH)
endif

# The embedded planner headers (float, no heap), copied into include/ from sdp-diffusion (see README).
EXTRA_CFLAGS += -I$(CURDIR)/include
# Planner capacities (see embedded/footprint.cpp and benchmarks/data/compress/cortex_m4_footprint.csv).
EXTRA_CFLAGS += -DTSDP_MAX_OBS=4 -DTSDP_MAX_WORLD=8 -DTSDP_MEMORY=0
EXTRA_CFLAGS += -Wno-double-promotion

OOT_CONFIG ?= $(CURDIR)/app-config
OOT_USES_CXX := 1

include $(CRAZYFLIE_BASE)/tools/make/oot.mk
