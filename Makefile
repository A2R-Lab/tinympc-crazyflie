CRAZYFLIE_BASE := $(PWD)/crazyflie-firmware

# Add TinyMPC directory to the build
# PROJ_ROOT = $(PWD)/src/TinyMPC/src/tinympc

# Include paths
EXTRA_CFLAGS += -I$(PWD)/src/TinyMPC/include
EXTRA_CFLAGS += -I$(PWD)/src/TinyMPC/include/Eigen
EXTRA_CFLAGS += -I$(PWD)/src/TinyMPC/src

# Eigen flags
EXTRA_CFLAGS += -DEIGEN_INITIALIZE_MATRICES_BY_ZERO
EXTRA_CFLAGS += -DEIGEN_NO_MALLOC
EXTRA_CFLAGS += -DNDEBUG
EXTRA_CFLAGS += -DEIGEN_FAST_MATH
EXTRA_CFLAGS += -Wno-error

#
# We override the default OOT_CONFIG here, we could also name our config
# to oot-config and that would be the default.
#
OOT_CONFIG := $(PWD)/app-config
OOT_USES_CXX := 1

include $(CRAZYFLIE_BASE)/tools/make/oot.mk