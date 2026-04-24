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
#
# We override the default OOT_CONFIG here, we could also name our config
# to oot-config and that would be the default.
#
OOT_CONFIG := $(CURDIR)/app-config

OOT_USES_CXX := 1

TINYMPC_CODEGEN_BUILD_DIR ?= /tmp/tinympc-codegen-build
TINYMPC_CODEGEN_TARGET ?= codegen_quadrotor_100hz
TINYMPC_CODEGEN_OUTPUT_DIR := $(TINYMPC_CODEGEN_BUILD_DIR)/examples/tinympc_generated_code_quadrotor_100hz
GENERATED_PARAMS_DIR := $(CURDIR)/src/generated
GENERATED_PARAMS_HEADER := $(CURDIR)/src/params_100hz.h
GENERATED_PROBLEM_HEADER := $(GENERATED_PARAMS_DIR)/quadrotor_100hz_generated.hpp

include $(CRAZYFLIE_BASE)/tools/make/oot.mk

.PHONY: codegen-params generated-build

codegen-params:
	cmake -S $(CURDIR)/../../TinyMPC -B $(TINYMPC_CODEGEN_BUILD_DIR)
	cmake --build $(TINYMPC_CODEGEN_BUILD_DIR) --target $(TINYMPC_CODEGEN_TARGET)
	cd $(TINYMPC_CODEGEN_BUILD_DIR)/examples && ./$(TINYMPC_CODEGEN_TARGET)
	mkdir -p $(GENERATED_PARAMS_DIR)
	cp $(TINYMPC_CODEGEN_OUTPUT_DIR)/crazyflie/params_100hz.h $(GENERATED_PARAMS_HEADER)
	cp $(TINYMPC_CODEGEN_OUTPUT_DIR)/problem_data/quadrotor_100hz_generated.hpp $(GENERATED_PROBLEM_HEADER)
	@echo "Generated Crazyflie params refreshed in $(GENERATED_PARAMS_HEADER)"

generated-build: codegen-params
	$(MAKE)
