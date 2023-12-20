#
# Copyright (c) 2020 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(milo_fw)

# NORDIC SDK APP START
target_sources(app PRIVATE main.c)

add_subdirectory(memfault)
add_subdirectory(wifi_provisioning)
# NORDIC SDK APP END
