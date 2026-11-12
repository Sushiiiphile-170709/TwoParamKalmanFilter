# GPS + Kalman filter (C11), Makefile tuned for Garuda Linux (Arch based).
#
#   make             build the program          -> build/gps_kalman
#   make run         build and run the demo on the files in data/
#   make test        build and run the Unity test suite
#   make sanitize    run the tests under AddressSanitizer + UBSan
#   make analyze     static analysis (gcc -fanalyzer, or clang --analyze)
#   make clean       remove build/ and output/
#   make help        list the targets and the options below
#
# Options (all optional, e.g. `make NATIVE=0 LTO=0`):
#   CC=gcc|clang   compiler (default gcc)
#   NATIVE=1|0     -march=native for this CPU; use 0 for a binary you will copy
#   LTO=1|0        link-time optimisation
#   DEBUG=1        -O0 -g3 and no hardening, for gdb
#   WERROR=1       treat warnings as errors
#   FASTLD=mold    use mold, lld or gold instead of GNU ld (must be installed)
#   V=1            print the full commands
#
# Needs only: sudo pacman -S --needed base-devel

ifeq ($(origin CC),default)
CC := gcc
endif

NATIVE ?= 1
LTO    ?= 1
DEBUG  ?= 0
WERROR ?= 0
SAN    ?= 0
FASTLD ?=
V      ?= 0

# Use every core unless the caller already chose -j.
ifeq ($(findstring -j,$(MAKEFLAGS)),)
MAKEFLAGS += -j$(shell nproc 2>/dev/null || echo 1)
endif

ifeq ($(V),1)
QUIET =
say   =
else
QUIET = @
say   = @printf '  %-8s %s\n' '$(1)' '$(2)'
endif

# ---- layout ----------------------------------------------------------------
SRC_DIR   := src
INC_DIR   := include
TEST_DIR  := tests
UNITY_DIR := $(TEST_DIR)/vendor/unity
BUILD     := build
OBJ_DIR   := $(BUILD)/obj
APP       := $(BUILD)/gps_kalman
TEST_BIN  := $(BUILD)/run_tests

LIB_SRC   := $(filter-out $(SRC_DIR)/main.c,$(wildcard $(SRC_DIR)/*.c))
LIB_OBJ   := $(patsubst $(SRC_DIR)/%.c,$(OBJ_DIR)/%.o,$(LIB_SRC))
MAIN_OBJ  := $(OBJ_DIR)/main.o
TEST_SRC  := $(wildcard $(TEST_DIR)/*.c)
TEST_OBJ  := $(patsubst $(TEST_DIR)/%.c,$(OBJ_DIR)/tests/%.o,$(TEST_SRC))
UNITY_OBJ := $(OBJ_DIR)/tests/unity.o

# ---- flags -----------------------------------------------------------------
ARCH     := $(shell uname -m)
IS_CLANG := $(shell $(CC) --version 2>/dev/null | grep -ci clang)

DEFS  := -D_POSIX_C_SOURCE=200809L
INCS  := -I$(INC_DIR)
STD   := -std=c11

WARN  := -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wdouble-promotion \
         -Wstrict-prototypes -Wmissing-prototypes -Wformat=2 -Wcast-qual \
         -Wwrite-strings -Wundef -Wvla
ifeq ($(WERROR),1)
WARN += -Werror
endif

# Results must not depend on the CPU: no fused multiply-add (-ffp-contract=off)
# and no fast-math. -fno-math-errno lets sqrt() compile to a single instruction.
FP := -fno-math-errno -ffp-contract=off

ifeq ($(SAN),1)
OPT   := -O1 -g -fno-omit-frame-pointer -fsanitize=address,undefined -fno-sanitize-recover=undefined
LDOPT := -fsanitize=address,undefined
else ifeq ($(DEBUG),1)
OPT   := -O0 -g3 -fno-omit-frame-pointer
LDOPT :=
else
# Same hardening Arch uses for its own packages, plus -O3 and the CPU flags.
OPT   := -O3 -DNDEBUG -pipe -fPIE -fno-plt -D_FORTIFY_SOURCE=3 \
         -fstack-protector-strong -fstack-clash-protection
ifneq ($(filter x86_64 i686,$(ARCH)),)
OPT   += -fcf-protection=full
endif
ifeq ($(NATIVE),1)
OPT   += -march=native -mtune=native
endif
ifeq ($(LTO),1)
OPT   += $(if $(filter 1,$(IS_CLANG)),-flto=thin,-flto=auto)
endif
LDOPT := -pie -Wl,-O1,--sort-common,--as-needed,-z,relro,-z,now,-z,noexecstack
ifneq ($(FASTLD),)
LDOPT += -fuse-ld=$(FASTLD)
endif
endif

# User-supplied CFLAGS/LDFLAGS are appended last.
BASE_CFLAGS := $(STD) $(OPT) $(FP) $(CFLAGS)
CPPFLAGS    += $(INCS) $(DEFS) -MMD -MP
LDLIBS      := -lm

UNITY_DEFS  := -DUNITY_INCLUDE_DOUBLE
TEST_DEFS   := $(UNITY_DEFS) -I$(UNITY_DIR) '-DKF_TEST_DATA_DIR="$(CURDIR)/data"'

# ---- targets ---------------------------------------------------------------
.PHONY: all run test sanitize analyze check-deps clean help

all: $(APP)

$(APP): $(MAIN_OBJ) $(LIB_OBJ)
	$(call say,LD,$@)
	$(QUIET)$(CC) $(BASE_CFLAGS) $(LDOPT) $(LDFLAGS) $^ $(LDLIBS) -o $@

$(TEST_BIN): $(TEST_OBJ) $(UNITY_OBJ) $(LIB_OBJ)
	$(call say,LD,$@)
	$(QUIET)$(CC) $(BASE_CFLAGS) $(LDOPT) $(LDFLAGS) $^ $(LDLIBS) -o $@

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c | check-deps
	@mkdir -p $(@D)
	$(call say,CC,$<)
	$(QUIET)$(CC) $(CPPFLAGS) $(BASE_CFLAGS) $(WARN) -c $< -o $@

$(OBJ_DIR)/tests/unity.o: $(UNITY_DIR)/unity.c | check-deps
	@mkdir -p $(@D)
	$(call say,CC,$<)
	$(QUIET)$(CC) $(UNITY_DEFS) $(BASE_CFLAGS) -w -c $< -o $@

$(OBJ_DIR)/tests/%.o: $(TEST_DIR)/%.c | check-deps
	@mkdir -p $(@D)
	$(call say,CC,$<)
	$(QUIET)$(CC) $(CPPFLAGS) $(TEST_DEFS) $(BASE_CFLAGS) $(WARN) -c $< -o $@

run: $(APP)
	./$(APP) --route data/default_route.csv --dropouts data/default_dropouts.csv

test: $(TEST_BIN)
	./$(TEST_BIN)

sanitize:
	$(MAKE) --no-print-directory BUILD=$(BUILD)/sanitize SAN=1 test

analyze: check-deps
	@set -e; for f in $(SRC_DIR)/*.c; do \
	  printf '  %-8s %s\n' ANALYZE $$f; \
	  if [ "$(IS_CLANG)" = "1" ]; then \
	    $(CC) $(INCS) $(DEFS) $(STD) --analyze -Xclang -analyzer-output=text $$f -o /dev/null; \
	  else \
	    $(CC) $(INCS) $(DEFS) $(STD) -O1 $(FP) -fanalyzer -c $$f -o /dev/null; \
	  fi; \
	done

# Stop early with an install hint if there is no compiler.
check-deps:
	@command -v $(CC) >/dev/null 2>&1 || { \
	  echo "$(CC) was not found. On Garuda / Arch Linux install the toolchain with:"; \
	  echo "  sudo pacman -S --needed base-devel"; \
	  exit 1; }

clean:
	rm -rf build output

help:
	@sed -n '2,/^$$/p' $(firstword $(MAKEFILE_LIST)) | sed 's/^# \{0,1\}//'

-include $(LIB_OBJ:.o=.d) $(MAIN_OBJ:.o=.d) $(TEST_OBJ:.o=.d)
