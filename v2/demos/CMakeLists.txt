# Copyright 2024 - 2025 Khalil Estell and the libhal contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 4.0)

project(demos LANGUAGES CXX)

find_package(LibhalCMakeUtil REQUIRED)
find_package(libhal REQUIRED)
find_package(libhal-util REQUIRED)
find_package(libhal-arm-mcu REQUIRED)
find_package(libhal-usb REQUIRED)

libhal_project_init()

if(NOT LIBHAL_PLATFORM)
    message(FATAL_ERROR
        "LIBHAL_PLATFORM must be set (see conanfile.py's `platform` option)")
endif()

# Startup code shared by every demo: the resource module (resource_list.cppm)
# that supplies each driver, the selected platform's initialize_platform(),
# and main().
add_library(startup_code STATIC)
target_sources(startup_code PUBLIC
    FILE_SET CXX_MODULES
    TYPE CXX_MODULES
    FILES resource_list.cppm
)
target_sources(startup_code PRIVATE
    main.cpp
    platforms/${LIBHAL_PLATFORM}.cpp
)
target_link_libraries(startup_code PUBLIC
    libhal::libhal
    libhal::util
    libhal::usb
    libhal::arm-mcu
)
target_compile_features(startup_code PUBLIC cxx_std_23)
libhal_apply_compile_options(startup_code)

set(DEMOS
    blank
    blinker
    adc
    uart
    can
    usb_cdc_raw
    usb
)

foreach(demo IN LISTS DEMOS)
    libhal_add_executable(${demo}
        SOURCES applications/${demo}.cpp
        LINK_LIBRARIES startup_code
    )

    target_link_libraries(${demo} PUBLIC
        libhal::libhal
        libhal::util
        libhal::usb
        libhal::arm-mcu
    )

    if(CMAKE_CROSSCOMPILING)
        libhal_create_hex_file_from(${demo})
        libhal_create_binary_file_from(${demo})
        libhal_print_size_of(${demo})
        libhal_create_disassembly_from(${demo})
        libhal_create_disassembly_with_source_from(${demo})
    endif()
endforeach()
