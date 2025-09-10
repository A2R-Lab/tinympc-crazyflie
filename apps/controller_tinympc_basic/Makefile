# TinyMPC Controller Out-of-Tree Application Makefile
# Based on the Crazyflie firmware out-of-tree controller template

#
# We want to execute the main Makefile for the firmware project,
# it will handle the build for us.
#
CRAZYFLIE_BASE := ../../crazyflie-firmware

#
# Include TinyMPC and dependencies (using old TinyMPC API)
#
EXTRA_CFLAGS += -I$(PWD)/TinyMPC/include/Eigen
EXTRA_CFLAGS += -I$(PWD)/TinyMPC/src
EXTRA_CFLAGS += -DEIGEN_INITIALIZE_MATRICES_BY_ZERO
EXTRA_CFLAGS += -DEIGEN_NO_MALLOC
EXTRA_CFLAGS += -DNDEBUG
EXTRA_CFLAGS += -DEIGEN_FAST_MATH

#
# We override the default OOT_CONFIG here, we could also name our config
# to oot-config and that would be the default.
#
OOT_CONFIG := $(PWD)/app-config

# Enable C++ compilation
OOT_USES_CXX := 1

include $(CRAZYFLIE_BASE)/tools/make/oot.mk
