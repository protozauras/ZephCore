# Copyright (c) 2025 ZephCore
# SPDX-License-Identifier: MIT

zephyr_library()
zephyr_library_sources(lr20xx_lora.c)
zephyr_library_add_dependencies(offsets_h)
