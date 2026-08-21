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
TINYMPC_ACTUATOR_LTI ?= 1
TINYMPC_REFERENCE_MODE ?= waypoint
TINYMPC_LEVEL_COST_MODE ?= baseline
TINYMPC_PROGRESS_SAMPLE_LIMIT ?= 0
TINYMPC_PROGRESS_SPEED_MPS ?= 0
TINYMPC_PROGRESS_REFERENCE_LIMITS ?= default
TINYMPC_COURSE_ACRO ?= none
TINYMPC_COURSE ?= none
EXTRA_CFLAGS += -DTINYMPC_PROGRESS_SAMPLE_LIMIT=$(TINYMPC_PROGRESS_SAMPLE_LIMIT)
EXTRA_CFLAGS += -DTINYMPC_PROGRESS_SPEED_MPS=$(TINYMPC_PROGRESS_SPEED_MPS)
ifneq ($(TINYMPC_PROGRESS_SPEED_MPS),0)
ifneq ($(TINYMPC_REFERENCE_MODE),progress)
$(error nonzero TINYMPC_PROGRESS_SPEED_MPS requires TINYMPC_REFERENCE_MODE=progress)
endif
endif
ifeq ($(TINYMPC_PROGRESS_REFERENCE_LIMITS),uncapped)
ifneq ($(TINYMPC_REFERENCE_MODE),progress)
$(error TINYMPC_PROGRESS_REFERENCE_LIMITS=uncapped requires TINYMPC_REFERENCE_MODE=progress)
endif
EXTRA_CFLAGS += -DTINYMPC_PROGRESS_REFERENCE_UNCAPPED=1
else ifneq ($(TINYMPC_PROGRESS_REFERENCE_LIMITS),default)
$(error TINYMPC_PROGRESS_REFERENCE_LIMITS must be default or uncapped)
endif
ifeq ($(TINYMPC_LEVEL_COST_MODE),yaw_angle_4x)
EXTRA_CFLAGS += -DTINYMPC_LEVEL_COST_YAW_ANGLE_4X=1
else ifeq ($(TINYMPC_LEVEL_COST_MODE),yaw_rate_4x)
EXTRA_CFLAGS += -DTINYMPC_LEVEL_COST_YAW_RATE_4X=1
else ifeq ($(TINYMPC_LEVEL_COST_MODE),yaw_diff_r_quarter)
EXTRA_CFLAGS += -DTINYMPC_LEVEL_COST_YAW_DIFF_R_QUARTER=1
else ifneq ($(TINYMPC_LEVEL_COST_MODE),baseline)
$(error TINYMPC_LEVEL_COST_MODE must be baseline, yaw_angle_4x, yaw_rate_4x, or yaw_diff_r_quarter)
endif
ifneq ($(TINYMPC_LEVEL_COST_MODE),baseline)
ifneq ($(TINYMPC_ACTUATOR_LTI),1)
$(error non-baseline TINYMPC_LEVEL_COST_MODE requires TINYMPC_ACTUATOR_LTI=1)
endif
endif
ifeq ($(TINYMPC_COURSE),canonical_circle)
EXTRA_CFLAGS += -DTINYMPC_COURSE_CANONICAL_CIRCLE=1
endif
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
else ifeq ($(TINYMPC_TRAJECTORY),straight_long)
EXTRA_CFLAGS += -DTINYMPC_TRAJECTORY_STRAIGHT_LONG=1
else ifeq ($(TINYMPC_TRAJECTORY),canonical_corridor)
EXTRA_CFLAGS += -DTINYMPC_TRAJECTORY_CANONICAL_CORRIDOR=1
else ifeq ($(TINYMPC_TRAJECTORY),canonical_circle)
EXTRA_CFLAGS += -DTINYMPC_TRAJECTORY_CANONICAL_CIRCLE=1
else ifeq ($(TINYMPC_TRAJECTORY),figure8)
EXTRA_CFLAGS += -DTINYMPC_TRAJECTORY_FIGURE8=1
else ifeq ($(TINYMPC_TRAJECTORY),oval)
EXTRA_CFLAGS += -DTINYMPC_TRAJECTORY_OVAL=1
else ifeq ($(TINYMPC_TRAJECTORY),canonical_figure8)
EXTRA_CFLAGS += -DTINYMPC_TRAJECTORY_CANONICAL_FIGURE8=1
else ifeq ($(TINYMPC_TRAJECTORY),chicane)
EXTRA_CFLAGS += -DTINYMPC_TRAJECTORY_CHICANE=1
else ifeq ($(TINYMPC_TRAJECTORY),canonical_chicane)
EXTRA_CFLAGS += -DTINYMPC_TRAJECTORY_CANONICAL_CHICANE=1
else ifeq ($(TINYMPC_TRAJECTORY),hairpin_180)
EXTRA_CFLAGS += -DTINYMPC_TRAJECTORY_HAIRPIN_180=1
else ifeq ($(TINYMPC_TRAJECTORY),canonical_hairpin)
EXTRA_CFLAGS += -DTINYMPC_TRAJECTORY_CANONICAL_HAIRPIN=1
else ifneq ($(TINYMPC_TRAJECTORY),circle)
$(error Unsupported TINYMPC_TRAJECTORY '$(TINYMPC_TRAJECTORY)')
endif
ifeq ($(TINYMPC_REFERENCE_MODE),trajectory)
EXTRA_CFLAGS += -DTINYMPC_REFERENCE_MODE_TRAJECTORY=1
else ifeq ($(TINYMPC_REFERENCE_MODE),progress)
EXTRA_CFLAGS += -DTINYMPC_REFERENCE_MODE_PROGRESS=1
else ifneq ($(TINYMPC_REFERENCE_MODE),waypoint)
$(error TINYMPC_REFERENCE_MODE must be waypoint, trajectory, or progress)
endif
ifeq ($(TINYMPC_COURSE_ACRO),barrel_roll_forward_360)
EXTRA_CFLAGS += -DTINYMPC_COURSE_ACRO_BARREL_ROLL_FORWARD_360=1
else ifeq ($(TINYMPC_COURSE_ACRO),backflip_360)
EXTRA_CFLAGS += -DTINYMPC_COURSE_ACRO_BACKFLIP_360=1
else ifneq ($(TINYMPC_COURSE_ACRO),none)
$(error Unsupported TINYMPC_COURSE_ACRO '$(TINYMPC_COURSE_ACRO)')
endif
ifneq ($(TINYMPC_COURSE_ACRO),none)
ifneq ($(TINYMPC_STORED_LTV),1)
$(error TINYMPC_COURSE_ACRO requires TINYMPC_STORED_LTV=1)
endif
ifneq ($(TINYMPC_ACTUATOR_LTI),1)
$(error TINYMPC_COURSE_ACRO requires TINYMPC_ACTUATOR_LTI=1)
endif
endif
ifeq ($(TINYMPC_STORED_LTV),1)
ifeq ($(TINYMPC_COURSE_ACRO),none)
ifeq ($(TINYMPC_TRAJECTORY),circle)
$(error TINYMPC_STORED_LTV=1 requires an acrobatic TINYMPC_TRAJECTORY)
endif
endif
EXTRA_CFLAGS += -DTINYMPC_USE_STORED_LTV=1
else ifneq ($(TINYMPC_STORED_LTV),0)
$(error TINYMPC_STORED_LTV must be 0 or 1)
endif
ifeq ($(TINYMPC_ACTUATOR_LTI),1)
EXTRA_CFLAGS += -DTINYMPC_USE_ACTUATOR_LTI=1
else ifneq ($(TINYMPC_ACTUATOR_LTI),0)
$(error TINYMPC_ACTUATOR_LTI must be 0 or 1)
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
