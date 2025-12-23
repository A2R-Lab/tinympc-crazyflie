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
CRAZYFLIE_BASE := $(abspath ../../crazyflie-firmware)
TINYMPC_BASE := $(abspath ../../TinyMPC)
CXX := clang++
CODEGEN_CXXFLAGS := -std=c++17 -Wall -Wextra -O2 -I$(TINYMPC_BASE)/src -I$(TINYMPC_BASE)/include -I$(TINYMPC_BASE)/include/Eigen
TINYMPC_CPP := $(shell find $(TINYMPC_BASE)/src/tinympc -name '*.cpp')

#
# To include header files from other directories
#
# EXTRA_CFLAGS += -lstdc++
# EXTRA_CFLAGS += -lgcc
# EXTRA_CFLAGS += -lc
# EXTRA_CFLAGS += -lm
EXTRA_CFLAGS += -I$(TINYMPC_BASE)/include
EXTRA_CFLAGS += -I$(TINYMPC_BASE)/include/Eigen
EXTRA_CFLAGS += -I$(TINYMPC_BASE)/src
EXTRA_CFLAGS += -I$(TINYMPC_BASE)/src/tinympc
EXTRA_CFLAGS += -DEIGEN_INITIALIZE_MATRICES_BY_ZERO
EXTRA_CFLAGS += -DEIGEN_NO_MALLOC
EXTRA_CFLAGS += -DNDEBUG
EXTRA_CFLAGS += -Wno-unused-result
#
# We override the default OOT_CONFIG here, we could also name our config
# to oot-config and that would be the default.
#
OOT_CONFIG := $(PWD)/app-config

OOT_USES_CXX := 1

.PHONY: controller codegen

controller: all

codegen:
	$(CXX) $(CODEGEN_CXXFLAGS) $(EXTRA_CFLAGS) \
		src/codegen_crazyflie.cpp \
		$(TINYMPC_CPP) \
		-o src/codegen_crazyflie

include $(CRAZYFLIE_BASE)/tools/make/oot.mk
