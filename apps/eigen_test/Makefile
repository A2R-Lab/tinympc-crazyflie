# Eigen Test App for Crazyflie
# This app tests Eigen operations needed for TinyMPC control

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

#
# We override the default OOT_CONFIG here, we could also name our config
# to oot-config and that would be the default.
#
OOT_CONFIG := $(PWD)/app-config

# Use C++ linker and add local include paths for Eigen headers
OOT_USES_CXX := 1
EXTRA_CFLAGS += -I$(PWD)/Eigen
EXTRA_CFLAGS += -DEIGEN_INITIALIZE_MATRICES_BY_ZERO -DEIGEN_NO_MALLOC -DNDEBUG 
EXTRA_CFLAGS += -Wno-psabi
EXTRA_CFLAGS += -Wno-error
EXTRA_CFLAGS += -DEMBEDDED_BUILD # Disable iostream for embedded compatibility

include $(CRAZYFLIE_BASE)/tools/make/oot.mk
