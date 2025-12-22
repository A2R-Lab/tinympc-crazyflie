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
CXX := clang++
CODEGEN_CXXFLAGS := -std=c++17 -Wall -Wextra -O2 -I../../TinyMPC/src -I../../TinyMPC/include


TINYMPC_CPP := $(shell find ../../TinyMPC/src/tinympc -name '*.cpp')
CRAZYFLIE_BASE := ../../crazyflie-firmware
TINYMPC_BASE := ../../TinyMPC

# Target-specific config for the firmware build
EXTRA_CFLAGS += -I$(TINYMPC_BASE)/include/Eigen
EXTRA_CFLAGS += -I$(TINYMPC_BASE)/src
EXTRA_CFLAGS += -DEIGEN_INITIALIZE_MATRICES_BY_ZERO
EXTRA_CFLAGS += -DEIGEN_NO_MALLOC
EXTRA_CFLAGS += -DNDEBUG
EXTRA_CFLAGS += -Wno-unused-result

controller: OOT_CONFIG := $(PWD)/app-config
controller: OOT_USES_CXX := 1

controller:
	$(MAKE) -C $(CRAZYFLIE_BASE) O=$(PWD) OOT=$(PWD)

# Pull in Crazyflie OOT build rules (must be top-level, not indented)
include $(CRAZYFLIE_BASE)/tools/make/oot.mk

codegen:
	$(CXX) $(CODEGEN_CXXFLAGS) $(EXTRA_CFLAGS) \
		src/codegen_crazyflie.cpp \
		$(TINYMPC_CPP) \
		-o src/codegen_crazyflie

