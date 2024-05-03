# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
set(BOARD rpi_pico)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(button_prj)
target_include_directories(app PRIVATE
    driver  # Only specify the drivers directory
)
target_sources(app PRIVATE src/main.c driver/ds18b20.c driver/analogsensors.c)
