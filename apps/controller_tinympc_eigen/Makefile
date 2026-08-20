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

# On macOS, Crazyflie's merge_config.sh requires GNU readlink/sed/cp.
# If Homebrew gnubin shims exist, prepend them automatically.
COREUTILS_GNUBIN := $(firstword $(wildcard /opt/homebrew/opt/coreutils/libexec/gnubin) $(wildcard /usr/local/opt/coreutils/libexec/gnubin))
GNU_SED_GNUBIN := $(firstword $(wildcard /opt/homebrew/opt/gnu-sed/libexec/gnubin) $(wildcard /usr/local/opt/gnu-sed/libexec/gnubin))
ifneq ($(COREUTILS_GNUBIN),)
export PATH := $(COREUTILS_GNUBIN):$(PATH)
endif
ifneq ($(GNU_SED_GNUBIN),)
export PATH := $(GNU_SED_GNUBIN):$(PATH)
endif

#
# To include header files from other directories
#
# EXTRA_CFLAGS += -lstdc++
# EXTRA_CFLAGS += -lgcc
# EXTRA_CFLAGS += -lc
# EXTRA_CFLAGS += -lm
EXTRA_CFLAGS += -I$(CURDIR)/TinyMPC-ADMM/ext/Eigen
EXTRA_CFLAGS += -I$(CURDIR)/TinyMPC-ADMM/src
EXTRA_CFLAGS += -DEIGEN_INITIALIZE_MATRICES_BY_ZERO
EXTRA_CFLAGS += -DEIGEN_NO_MALLOC
EXTRA_CFLAGS += -DNDEBUG
EXTRA_CFLAGS += -Wno-unused-result
# The capture-time state channel uses a dedicated UART1 DMA sender task.
EXTRA_CFLAGS += -DENABLE_UART1_DMA

# Select the compiled stored reference.  The default remains the conservative
# circle; the four acrobatic choices enable reference-centered quaternion-error
# TinyMPC and include per-motor feedforward in the trajectory header.
TINYMPC_TRAJECTORY ?= circle
TINYMPC_STORED_LTV ?= 0
ifeq ($(TINYMPC_TRAJECTORY),roll_flip_360)
EXTRA_CFLAGS += -DTINYMPC_TRAJECTORY_ROLL_FLIP_360=1
else ifeq ($(TINYMPC_TRAJECTORY),front_flip_360)
EXTRA_CFLAGS += -DTINYMPC_TRAJECTORY_FRONT_FLIP_360=1
else ifeq ($(TINYMPC_TRAJECTORY),backflip_360)
EXTRA_CFLAGS += -DTINYMPC_TRAJECTORY_BACKFLIP_360=1
else ifeq ($(TINYMPC_TRAJECTORY),barrel_roll_forward_360)
EXTRA_CFLAGS += -DTINYMPC_TRAJECTORY_BARREL_ROLL_FORWARD_360=1
else ifeq ($(TINYMPC_TRAJECTORY),straight)
EXTRA_CFLAGS += -DTINYMPC_TRAJECTORY_STRAIGHT=1
else ifeq ($(TINYMPC_TRAJECTORY),figure8)
EXTRA_CFLAGS += -DTINYMPC_TRAJECTORY_FIGURE8=1
else ifneq ($(TINYMPC_TRAJECTORY),circle)
$(error Unsupported TINYMPC_TRAJECTORY '$(TINYMPC_TRAJECTORY)')
endif
ifeq ($(TINYMPC_STORED_LTV),1)
ifeq ($(TINYMPC_TRAJECTORY),circle)
$(error TINYMPC_STORED_LTV=1 requires an acrobatic TINYMPC_TRAJECTORY)
endif
EXTRA_CFLAGS += -DTINYMPC_USE_STORED_LTV=1
else ifneq ($(TINYMPC_STORED_LTV),0)
$(error TINYMPC_STORED_LTV must be 0 or 1)
endif
#
# We override the default OOT_CONFIG here, we could also name our config
# to oot-config and that would be the default.
#
OOT_CONFIG := $(CURDIR)/app-config

OOT_USES_CXX := 1

include $(CRAZYFLIE_BASE)/tools/make/oot.mk

# The upstream OOT `cload` target only transfers the existing binary.  Make
# flashing safe after a source update: build the app first so `make cload`
# cannot silently reinstall a stale cf21bl.bin lacking new parameters/logs.
cload: all
