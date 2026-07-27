# The firmware uses the Kbuild build system. There are 'Kbuild' files in this
# example that outlays what needs to be built. (check src/Kbuild).
#
# The firmware is configured using options in Kconfig files, the
# values of these end up in the .config file in the firmware directory.
#
# By setting the OOT_CONFIG (it is '$(PWD)/oot-config' by default) environment
# variable you can provide a custom configuration. It is important that you
# enable the app-layer. See app-config in this directory for example.

#
# We want to execute the main Makefile for the firmware project,
# it will handle the build for us.
#
CRAZYFLIE_BASE := ../../crazyflie-firmware

# crazyflie-firmware's config merger uses GNU readlink/cp/sed options. Put
# Homebrew's unprefixed GNU tools first on macOS so a clean OOT build actually
# enables CONFIG_APP_ENABLE and CONFIG_CONTROLLER_OOT.
ifeq ($(shell uname -s),Darwin)
DARWIN_COREUTILS_GNUBIN := $(shell brew --prefix coreutils 2>/dev/null)/libexec/gnubin
DARWIN_GNU_SED_GNUBIN := $(shell brew --prefix gnu-sed 2>/dev/null)/libexec/gnubin
ifeq ($(wildcard $(DARWIN_COREUTILS_GNUBIN)/readlink),)
$(error macOS builds require Homebrew coreutils)
endif
ifeq ($(wildcard $(DARWIN_GNU_SED_GNUBIN)/sed),)
$(error macOS builds require Homebrew gnu-sed)
endif
export PATH := $(DARWIN_GNU_SED_GNUBIN):$(DARWIN_COREUTILS_GNUBIN):$(PATH)
endif

#
# To include header files from other directories
#
# EXTRA_CFLAGS += -lstdc++
# EXTRA_CFLAGS += -lgcc
# EXTRA_CFLAGS += -lc
# EXTRA_CFLAGS += -lm
EXTRA_CFLAGS += -I$(PWD)/TinyMPC/include/Eigen
EXTRA_CFLAGS += -I$(PWD)/TinyMPC/src
EXTRA_CFLAGS += -DEIGEN_INITIALIZE_MATRICES_BY_ZERO
EXTRA_CFLAGS += -DEIGEN_NO_MALLOC
EXTRA_CFLAGS += -DNDEBUG
EXTRA_CFLAGS += -DEIGEN_FAST_MATH
EXTRA_CFLAGS += -Wno-error
EXTRA_CXXFLAGS += -Wno-error

# Optional compile-time controller selection:
#   make FIRMWARE_MODE=0  # nominal TinyMPC
#   make FIRMWARE_MODE=1  # MPC-CBF
#   make FIRMWARE_MODE=2  # LIMO posthoc
#   make FIRMWARE_MODE=3  # LIMO embedded
# If omitted, controller_tinympc.cpp's TINYMPC_FIRMWARE_MODE default is used.
ifdef FIRMWARE_MODE
ifneq ($(filter $(FIRMWARE_MODE),0 1 2 3),$(FIRMWARE_MODE))
$(error FIRMWARE_MODE must be 0, 1, 2, or 3)
endif
endif

# Optional compile-time trajectory override:
#   make TRAJECTORY=0  # hover
#   make TRAJECTORY=1  # X-axis line
#   make TRAJECTORY=2  # Y-axis line
#   make TRAJECTORY=3  # circle
#   make TRAJECTORY=4  # figure eight
# If omitted, controller_tinympc.cpp's TINYMPC_TRAJECTORY default is used.
ifdef TRAJECTORY
ifneq ($(filter $(TRAJECTORY),0 1 2 3 4),$(TRAJECTORY))
$(error TRAJECTORY must be 0, 1, 2, 3, or 4)
endif
endif

# Optional numeric-only optimization experiment:
#   make FIRMWARE_MODE=3 NUMERIC_O2=0  # firmware-wide -Os baseline
#   make FIRMWARE_MODE=3 NUMERIC_O2=1  # -O2 for controller + ADMM only
# Keep the default at zero until an on-target DWT A/B test demonstrates the
# cycle/flash tradeoff.
NUMERIC_O2 ?= 0
ifneq ($(filter $(NUMERIC_O2),0 1),$(NUMERIC_O2))
$(error NUMERIC_O2 must be 0 or 1)
endif

# Safe, deterministic on-target profiling image:
#   make FIRMWARE_MODE=3 BENCH_PROFILE=1 NUMERIC_O2=0
#   make FIRMWARE_MODE=3 BENCH_PROFILE=1 NUMERIC_O2=1
# Add PROFILE_TANH=1 for a separate activation-attribution run. Keeping it
# off preserves clean bar_cyc and total_cyc measurements.
# The controller task receives a canned state sequence and controller 6 always
# commands motors off. BENCH_PROFILE=0 is the normal flight build.
BENCH_PROFILE ?= 0
ifneq ($(filter $(BENCH_PROFILE),0 1),$(BENCH_PROFILE))
$(error BENCH_PROFILE must be 0 or 1)
endif
PROFILE_TANH ?= 0
ifneq ($(filter $(PROFILE_TANH),0 1),$(PROFILE_TANH))
$(error PROFILE_TANH must be 0 or 1)
endif
ifeq ($(PROFILE_TANH),1)
ifneq ($(BENCH_PROFILE),1)
$(error PROFILE_TANH=1 requires BENCH_PROFILE=1)
endif
endif
#
# We override the default OOT_CONFIG here, we could also name our config
# to oot-config and that would be the default.
#
OOT_CONFIG := $(PWD)/app-config

OOT_USES_CXX := 1

# Forward the selector to the firmware sub-make without putting it in the
# global CFLAGS. src/Kbuild applies it only to controller_tinympc.o.
OOT_ARGS := -C $(CRAZYFLIE_BASE) OOT=$(PWD) EXTRA_CFLAGS="$(EXTRA_CFLAGS)"
# Pin the firmware link flags as a command-line make value. This prevents the
# recursive Crazyflie build from appending nosys/nano specs more than once on
# macOS Arm GNU Toolchain 14, while retaining the normal embedded link setup.
OOT_ARGS += LDFLAGS="--specs=nosys.specs --specs=nano.specs -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -nostdlib"
OOT_ARGS += NUMERIC_O2=$(NUMERIC_O2)
OOT_ARGS += BENCH_PROFILE=$(BENCH_PROFILE)
OOT_ARGS += PROFILE_TANH=$(PROFILE_TANH)
ifdef FIRMWARE_MODE
OOT_ARGS += FIRMWARE_MODE=$(FIRMWARE_MODE)
endif
ifdef TRAJECTORY
OOT_ARGS += TRAJECTORY=$(TRAJECTORY)
endif

include $(CRAZYFLIE_BASE)/tools/make/oot.mk
