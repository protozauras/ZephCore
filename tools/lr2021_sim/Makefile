# SPDX-License-Identifier: MIT
#
# Makefile for the LR2021 driver sandbox.
#
# Compiles the stub + driver-under-test + unit tests with vanilla GCC
# on Linux, macOS, MSYS2/Cygwin/WSL on Windows. No Zephyr, no RTOS,
# no flash. Output is a single executable: lr2021_sim_tests.
#
# Usage:
#   make            # build and run the tests
#   make test       # run only
#   make clean      # remove generated files
#   make verbose=1  # show extra diagnostics
#
# rtl_433 phase-3 (2026-09-19): the vendored upstream core + decoders
# under ../zephcore/src/rtl433 compile with RELAXED warnings (upstream
# code, upstream style); the glue (sniffer_rtl433.c), the tests and the
# sim sources stay -Wall -Wextra -Werror clean.  The vendored files are
# force-included the static-arena shim rtl433_mem.h exactly like the
# firmware build does.
#
# WM-BUS F1 (2026-09-19): the wM-Bus header parser
# (../zephcore/src/sniffer_wmbus_parse.c) is own code compiled into both
# the firmware and this sandbox — strict flags like the sim sources.

CC      ?= cc
CFLAGS  ?= -std=c99 -Wall -Wextra -Werror -O2 -g
LDFLAGS ?=
TARGET  := lr2021_sim_tests

RTL_DIR    := ../../zephcore/src/rtl433
BASE_CFLAGS := $(CFLAGS) -DSNF_RTL433_HOST -I$(RTL_DIR)
RTL_RELAX  := -Wno-unused-parameter -Wno-sign-compare -Wno-unused-variable

SIM_SRCS := stub_lr2021.c driver_under_test.c test_lr2021_driver.c \
            tdm_wedge_model.c test_tdm_wedge.c test_rtl433.c
GLUE_SRC := $(RTL_DIR)/sniffer_rtl433.c
PARSE_SRC := ../../zephcore/src/sniffer_wmbus_parse.c
RTL_SRCS := $(RTL_DIR)/bitbuffer.c $(RTL_DIR)/bit_util.c $(RTL_DIR)/data.c \
            $(RTL_DIR)/abuf.c $(RTL_DIR)/list.c $(RTL_DIR)/decoder_util.c \
            $(RTL_DIR)/pulse_slicer.c \
            $(RTL_DIR)/devices/acurite.c \
            $(RTL_DIR)/devices/lacrosse_tx141x.c \
            $(RTL_DIR)/devices/oregon_scientific.c \
            $(RTL_DIR)/devices/oregon_scientific_v1.c \
            $(RTL_DIR)/devices/fineoffset.c \
            $(RTL_DIR)/devices/prologue.c \
            $(RTL_DIR)/devices/hideki.c

OBJS := $(addprefix obj/,$(notdir $(SIM_SRCS:.c=.o) $(GLUE_SRC:.c=.o) $(PARSE_SRC:.c=.o) $(RTL_SRCS:.c=.o)))

.PHONY: all test clean

all: $(TARGET)

obj:
	mkdir -p obj

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

# Sim sources: strict flags, no shim (the glue #includes it explicitly).
obj/%.o: %.c stub_lr2021.h driver_under_test.h | obj
	$(CC) $(BASE_CFLAGS) -c -o $@ $<

# Glue (own code): STRICT flags like the sim sources; force-included
# shim so it matches the firmware TU setup.
define GLUE_RULE
obj/$(notdir $(1:.c=.o)): $(1) | obj
	$(CC) $(BASE_CFLAGS) -include $(RTL_DIR)/rtl433_mem.h -c -o $$@ $$<
endef
$(eval $(call GLUE_RULE,$(GLUE_SRC)))

# wM-Bus header parser (own code, strict flags; no shim needed).
define PARSE_RULE
obj/$(notdir $(1:.c=.o)): $(1) | obj
	$(CC) $(BASE_CFLAGS) -c -o $$@ $$<
endef
$(eval $(call PARSE_RULE,$(PARSE_SRC)))

# Vendored upstream: relaxed warnings + forced static-arena shim.
define RTL_RULE
obj/$(notdir $(1:.c=.o)): $(1) | obj
	$(CC) $(BASE_CFLAGS) $(RTL_RELAX) -include $(RTL_DIR)/rtl433_mem.h -c -o $$@ $$<
endef
$(foreach s,$(RTL_SRCS),$(eval $(call RTL_RULE,$(s))))

test: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(OBJS) $(TARGET)
