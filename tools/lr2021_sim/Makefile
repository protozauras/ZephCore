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

CC      ?= cc
CFLAGS  ?= -std=c99 -Wall -Wextra -Werror -O2 -g
LDFLAGS ?=
TARGET  := lr2021_sim_tests

SRCS := stub_lr2021.c driver_under_test.c test_lr2021_driver.c tdm_wedge_model.c test_tdm_wedge.c
OBJS := $(SRCS:.c=.o)

.PHONY: all test clean

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

%.o: %.c stub_lr2021.h driver_under_test.h
	$(CC) $(CFLAGS) -c -o $@ $<

test: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(OBJS) $(TARGET)
