# Vision Guidance App for Crazyflie
#
# Visual standoff/centering on top of the STOCK PID controller. Pairs with the
# GAP8 CV module in esp_color_object/comms/comms_deck.c over CPX.
#
# The firmware uses the Kbuild build system (see Kbuild / src/Kbuild). The
# app-layer is enabled and configured via app-config (OOT_CONFIG below).

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
# We override the default OOT_CONFIG here, we could also name our config
# to oot-config and that would be the default.
#
OOT_CONFIG := $(CURDIR)/app-config

include $(CRAZYFLIE_BASE)/tools/make/oot.mk
