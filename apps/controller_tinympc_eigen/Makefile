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
# Absorb receive bursts while the synchronous MPC occupies the stabilizer.
EXTRA_CFLAGS += -DSYSLINK_RX_QUEUE_SIZE=32 -DRADIOLINK_CRTP_QUEUE_SIZE=32
TINYMPC_PROFILE ?= espnet_straight_stop
ifeq ($(TINYMPC_PROFILE),espnet_straight_stop)
EXTRA_CFLAGS += -DTINYMPC_ESPNET_STRAIGHT_TEST=1
else ifeq ($(TINYMPC_PROFILE),legacy)
EXTRA_CFLAGS += -DTINYMPC_ESPNET_STRAIGHT_TEST=0
else
$(error TINYMPC_PROFILE must be espnet_straight_stop or legacy)
endif
#
# We override the default OOT_CONFIG here, we could also name our config
# to oot-config and that would be the default.
#
OOT_CONFIG := $(CURDIR)/app-config
ifeq ($(TINYMPC_DEPTHGATE_BENCHMARK),1)
# Generate from the live profile so only benchmark task stack size differs.
OOT_CONFIG := $(CURDIR)/build-depthgate-benchmark.config
$(shell sed 's/^CONFIG_APP_STACKSIZE=.*/CONFIG_APP_STACKSIZE=1024/' $(CURDIR)/app-config > $(OOT_CONFIG))
endif

OOT_USES_CXX := 1

include $(CRAZYFLIE_BASE)/tools/make/oot.mk

cload: all

ifeq ($(TINYMPC_BENCHMARK),1)
EXTRA_CFLAGS += -DTINYMPC_BENCHMARK_ONLY=1
endif

ifeq ($(TINYMPC_DEPTHGATE_BENCHMARK),1)
EXTRA_CFLAGS += -DTINYMPC_BENCHMARK_ONLY=1 -DTINYMPC_DEPTHGATE_BENCHMARK_ONLY=1
endif
