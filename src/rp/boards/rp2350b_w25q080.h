/*
 * Copyright (c) 2025 Shin Umeda
 *  
 * This is a board file that gets pico-sdk headers to work at the bare minimum.
 * This doesn't define much by itself, and nothing should depend too much on definitions
 * here.
 * 
 * */

#ifndef _rp2350_micromod_h
#define _rp2350_micromod_h

pico_board_cmake_set(PICO_PLATFORM, rp2350)

#define PICO_RP2350A 0 // use RP2350B

#define PICO_BOOT_STAGE2_CHOOSE_W25Q080 1

#ifndef PICO_FLASH_SPI_CLKDIV
#define PICO_FLASH_SPI_CLKDIV 2
#endif

pico_board_cmake_set_default(PICO_FLASH_SIZE_BYTES, (8 * 1024 * 1024))
#ifndef PICO_FLASH_SIZE_BYTES
#define PICO_FLASH_SIZE_BYTES (8 * 1024 * 1024)
#endif

pico_board_cmake_set_default(PICO_RP2350_A2_SUPPORTED, 1)
#ifndef PICO_RP2350_A2_SUPPORTED
#define PICO_RP2350_A2_SUPPORTED 1
#endif

#endif
