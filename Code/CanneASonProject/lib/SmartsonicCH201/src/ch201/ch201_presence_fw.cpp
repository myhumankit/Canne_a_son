//
// Chirp Microsystems Firmware Header Generator v2.4 (Python 3.8.5)
//
// File generated at 2022-03-29 16:04:37.383148 by klong
// Script input parameters:
//   - Input file:                 invn.chirpmicro.asic.ch201.presence.v24.hex
//   - Output file:                ch201_presence_fw.c
//   - Part number:                201
//   - Program size:               2048
//   - DMEM start address:         0x0
//   - PMEM start address:         0xf800
//   - Firmware name:              presence
//   - Firmware name (sanitized):  presence
//   - Firmware git version:       presence_v24
//   - Firmware git sha1:          dd1f4705bbf207cecc430552a77c0ebe88a14985
//
// Copyright (c) 2022, Chirp Microsystems. All rights reserved.
//

#include <stdint.h>
#include "CH201.h"
#include "ch201_presence.h"

const char * ch201_presence_version = "presence_presence_v24";
const char * ch201_presence_gitsha1 = "dd1f4705bbf207cecc430552a77c0ebe88a14985";

#define RAM_INIT_ADDRESS 1772
#define RAM_INIT_WRITE_SIZE   15

uint16_t get_ch201_presence_fw_ram_init_addr(void) { return (uint16_t)RAM_INIT_ADDRESS;}
uint16_t get_ch201_presence_fw_ram_init_size(void) { return (uint16_t)RAM_INIT_WRITE_SIZE;}

const unsigned char ram_ch201_presence_init[RAM_INIT_WRITE_SIZE] = {
0x00, 0x00, 0x64, 0x00, 0x06, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x00, };

const unsigned char * get_ram_ch201_presence_init_ptr(void) { return &ram_ch201_presence_init[0];}

#define	CH201_PRESENCE_TEXT_SIZE	1572
#define	CH201_PRESENCE_VEC_SIZE	32

const uint16_t  ch201_presence_text_size	= CH201_PRESENCE_TEXT_SIZE;
const uint16_t  ch201_presence_vec_size	= CH201_PRESENCE_VEC_SIZE;

const unsigned char ch201_presence_fw_text[CH201_PRESENCE_TEXT_SIZE] = {
0x0f, 0x12, 0x0e, 0x12, 0x0d, 0x12, 0x0c, 0x12, 0x0b, 0x12, 0x0a, 0x12, 0x09, 0x12, 0x08, 0x12, 
0x07, 0x12, 0x06, 0x12, 0xb2, 0x40, 0x77, 0x01, 0xa6, 0x01, 0xd2, 0xc3, 0xec, 0x06, 0xc2, 0x93, 
0x14, 0x02, 0x09, 0x20, 0xb0, 0x12, 0xb2, 0xfb, 0x4c, 0x93, 0x17, 0x20, 0xd2, 0x43, 0x14, 0x02, 
0xc2, 0x43, 0xea, 0x06, 0x12, 0x3c, 0xd2, 0x93, 0x14, 0x02, 0x0d, 0x20, 0xc2, 0x93, 0xea, 0x06, 
0x0c, 0x20, 0xd2, 0x43, 0x01, 0x02, 0xe2, 0x43, 0x14, 0x02, 0xe2, 0xd3, 0xec, 0x06, 0xb2, 0x40, 
0x7e, 0x14, 0xd0, 0x01, 0x02, 0x3c, 0x82, 0x43, 0xf0, 0x01, 0xf2, 0x90, 0x03, 0x00, 0xea, 0x06, 
0x42, 0x2c, 0xf2, 0x90, 0x21, 0x00, 0x12, 0x02, 0x03, 0x28, 0xf2, 0x40, 0x20, 0x00, 0x12, 0x02, 
0x59, 0x42, 0x07, 0x02, 0x09, 0x59, 0x5f, 0x42, 0x12, 0x02, 0x5f, 0x42, 0xea, 0x06, 0x58, 0x42, 
0x13, 0x02, 0xb2, 0x43, 0x24, 0x02, 0x0a, 0x43, 0x38, 0xb0, 0x80, 0xff, 0x01, 0x24, 0x1a, 0x43, 
0x3f, 0x40, 0x03, 0x00, 0x48, 0xff, 0x4f, 0x48, 0x0f, 0xda, 0x0f, 0x93, 0x24, 0x24, 0x1f, 0x43, 
0x48, 0x5f, 0x09, 0x93, 0x20, 0x24, 0x37, 0x40, 0x28, 0x02, 0x06, 0x43, 0x3e, 0x40, 0x28, 0x02, 
0x0f, 0x46, 0x0f, 0x5f, 0x0f, 0x5f, 0x0f, 0x5e, 0x2c, 0x4f, 0x0f, 0x46, 0x0f, 0x5f, 0x1f, 0x53, 
0x0f, 0x5f, 0x0e, 0x5f, 0x2d, 0x4e, 0x0a, 0x93, 0x05, 0x20, 0x27, 0x53, 0x87, 0x4c, 0xfe, 0xff, 
0x0c, 0x4d, 0x02, 0x3c, 0xb0, 0x12, 0xb2, 0xfc, 0x27, 0x53, 0x87, 0x4c, 0xfe, 0xff, 0x4f, 0x48, 
0x06, 0x5f, 0x06, 0x99, 0xe3, 0x2b, 0xe2, 0x93, 0x14, 0x02, 0x08, 0x28, 0xc2, 0x43, 0xf5, 0x06, 
0xc2, 0x93, 0x0e, 0x02, 0x08, 0x20, 0xb0, 0x12, 0x96, 0xfd, 0x05, 0x3c, 0x5c, 0x43, 0xb0, 0x12, 
0x26, 0xf9, 0xa2, 0xc2, 0x92, 0x01, 0xa2, 0xd2, 0x92, 0x01, 0xb1, 0xc0, 0xf0, 0x00, 0x14, 0x00, 
0x36, 0x41, 0x37, 0x41, 0x38, 0x41, 0x39, 0x41, 0x3a, 0x41, 0x3b, 0x41, 0x3c, 0x41, 0x3d, 0x41, 
0x3e, 0x41, 0x3f, 0x41, 0x00, 0x13, 0x0a, 0x12, 0xd2, 0xd3, 0xec, 0x06, 0x1f, 0x42, 0xee, 0x06, 
0x3f, 0x50, 0x00, 0x10, 0x82, 0x4f, 0xf0, 0x01, 0xf2, 0x90, 0x40, 0x00, 0x01, 0x02, 0x61, 0x24, 
0xd2, 0x92, 0x07, 0x02, 0xf2, 0x06, 0x04, 0x20, 0xd2, 0x92, 0x04, 0x02, 0xf3, 0x06, 0x3b, 0x24, 
0xd2, 0x42, 0x07, 0x02, 0xf2, 0x06, 0xd2, 0x42, 0x04, 0x02, 0xf3, 0x06, 0x5f, 0x42, 0x04, 0x02, 
0x0f, 0x5f, 0x3f, 0x80, 0x0b, 0x00, 0x5e, 0x42, 0x07, 0x02, 0x0e, 0x5e, 0x0e, 0x8f, 0x3e, 0x80, 
0x0b, 0x00, 0xc2, 0x93, 0xea, 0x06, 0x04, 0x20, 0xb2, 0x40, 0x58, 0x18, 0xd8, 0x06, 0x03, 0x3c, 
0xb2, 0x40, 0x58, 0x24, 0xd8, 0x06, 0x3a, 0x40, 0xf8, 0x2f, 0x3d, 0x40, 0xda, 0x06, 0x5b, 0x43, 
0x3f, 0xb0, 0x80, 0xff, 0x2f, 0x20, 0x2d, 0x53, 0x0f, 0x5f, 0x0f, 0x5f, 0x0f, 0x5f, 0x3f, 0x50, 
0x00, 0x2c, 0x8d, 0x4f, 0xfe, 0xff, 0x5b, 0x53, 0x3f, 0x40, 0xf8, 0x4f, 0x3e, 0xb0, 0x80, 0xff, 
0x1a, 0x20, 0x0e, 0x5e, 0x0e, 0x5e, 0x0e, 0x5e, 0x3e, 0x50, 0x00, 0x4c, 0x8d, 0x4e, 0x00, 0x00, 
0x5b, 0x53, 0xc2, 0x4b, 0xeb, 0x06, 0x4c, 0x93, 0x04, 0x20, 0xb2, 0x40, 0x8a, 0x10, 0xa2, 0x01, 
0x23, 0x3c, 0xb2, 0x40, 0x8e, 0x10, 0xa2, 0x01, 0x5f, 0x42, 0x10, 0x02, 0x3f, 0x50, 0x00, 0x38, 
0x82, 0x4f, 0xa0, 0x01, 0x19, 0x3c, 0x2d, 0x53, 0x8d, 0x4f, 0xfe, 0xff, 0x5b, 0x53, 0x3e, 0x80, 
0x7f, 0x00, 0xdc, 0x3f, 0x2d, 0x53, 0x8d, 0x4a, 0xfe, 0xff, 0x5b, 0x53, 0x3f, 0x80, 0x7f, 0x00, 
0xc7, 0x3f, 0xb2, 0x40, 0x40, 0x20, 0xd8, 0x06, 0xd2, 0x43, 0xeb, 0x06, 0xb2, 0x40, 0x1e, 0x18, 
0xa0, 0x01, 0xb2, 0x40, 0x86, 0x10, 0xa2, 0x01, 0x5f, 0x42, 0xeb, 0x06, 0x0f, 0x93, 0x06, 0x24, 
0x3e, 0x40, 0xd8, 0x06, 0xb2, 0x4e, 0xa4, 0x01, 0x1f, 0x83, 0xfc, 0x23, 0x92, 0x43, 0xae, 0x01, 
0xa2, 0x43, 0xae, 0x01, 0x3a, 0x41, 0x30, 0x41, 0x0a, 0x12, 0xb2, 0x40, 0x80, 0x5a, 0x20, 0x01, 
0xe2, 0x42, 0xe0, 0x01, 0xd2, 0x43, 0xe2, 0x01, 0xc2, 0x43, 0x0e, 0x02, 0xf2, 0x40, 0x40, 0x00, 
0x01, 0x02, 0xf2, 0x40, 0x3c, 0x00, 0x07, 0x02, 0xf2, 0x40, 0x10, 0x00, 0x04, 0x02, 0xf2, 0x40, 
0x09, 0x00, 0x00, 0x02, 0xc2, 0x43, 0x13, 0x02, 0xc2, 0x43, 0x12, 0x02, 0xd2, 0x43, 0x05, 0x02, 
0xc2, 0x43, 0x11, 0x02, 0xf2, 0x40, 0x1e, 0x00, 0x10, 0x02, 0xb2, 0x40, 0x80, 0x00, 0x02, 0x02, 
0xf2, 0x40, 0x03, 0x00, 0xc2, 0x01, 0xb2, 0x40, 0x00, 0x02, 0xa6, 0x01, 0xb2, 0x40, 0x00, 0x06, 
0xa6, 0x01, 0xb2, 0x40, 0x28, 0x02, 0xb0, 0x01, 0xb2, 0x40, 0x12, 0x00, 0xb2, 0x01, 0xb2, 0x40, 
0x77, 0x01, 0xa6, 0x01, 0xb2, 0x40, 0x80, 0x00, 0x90, 0x01, 0xb2, 0x40, 0x07, 0x00, 0x92, 0x01, 
0x0a, 0x43, 0x05, 0x3c, 0xc2, 0x93, 0xec, 0x06, 0x02, 0x24, 0x32, 0xd0, 0x18, 0x00, 0x5f, 0x42, 
0x01, 0x02, 0x0a, 0x9f, 0x22, 0x24, 0x5a, 0x42, 0x01, 0x02, 0x0f, 0x4a, 0x3f, 0x80, 0x10, 0x00, 
0x18, 0x24, 0x3f, 0x80, 0x10, 0x00, 0x15, 0x24, 0x3f, 0x80, 0x20, 0x00, 0x0d, 0x20, 0xc2, 0x43, 
0x14, 0x02, 0xe2, 0x42, 0xea, 0x06, 0x1f, 0x42, 0xee, 0x06, 0x3f, 0x50, 0x00, 0x10, 0x82, 0x4f, 
0xf0, 0x01, 0x5c, 0x43, 0xb0, 0x12, 0x26, 0xf9, 0xe2, 0x42, 0xe8, 0x06, 0xe2, 0xc3, 0xe0, 0x01, 
0x04, 0x3c, 0xe2, 0xd3, 0xe0, 0x01, 0xf2, 0x43, 0xf5, 0x06, 0x32, 0xc2, 0x03, 0x43, 0xc2, 0x93, 
0xec, 0x06, 0xd0, 0x23, 0x32, 0xd0, 0x58, 0x00, 0xd2, 0x3f, 0x0f, 0x12, 0x0e, 0x12, 0x0d, 0x12, 
0x0c, 0x12, 0x0b, 0x12, 0x0a, 0x12, 0x1a, 0x42, 0x02, 0x02, 0xe2, 0x93, 0x01, 0x02, 0x03, 0x20, 
0xd2, 0x83, 0xf4, 0x06, 0x1a, 0x24, 0xf2, 0x90, 0x10, 0x00, 0x01, 0x02, 0x0c, 0x20, 0xd2, 0x92, 
0x0e, 0x02, 0xf5, 0x06, 0x08, 0x2c, 0xd2, 0x53, 0xf5, 0x06, 0xd2, 0x92, 0x0e, 0x02, 0xf5, 0x06, 
0x02, 0x20, 0xb0, 0x12, 0x96, 0xfd, 0xb2, 0x40, 0x77, 0x06, 0xa6, 0x01, 0x3c, 0x42, 0xb0, 0x12, 
0x04, 0xfe, 0xb2, 0x40, 0x77, 0x01, 0xa6, 0x01, 0x18, 0x3c, 0xd2, 0x42, 0x05, 0x02, 0xf4, 0x06, 
0x5e, 0x42, 0x07, 0x02, 0x5f, 0x42, 0x07, 0x02, 0x0f, 0x5f, 0x0f, 0x5f, 0x0f, 0x5f, 0x0f, 0x8e, 
0x04, 0x3c, 0x1a, 0x52, 0x02, 0x02, 0xd2, 0x83, 0xf4, 0x06, 0xe2, 0x93, 0xf4, 0x06, 0x02, 0x28, 
0x0a, 0x9f, 0xf7, 0x2b, 0x5c, 0x43, 0xb0, 0x12, 0x26, 0xf9, 0x82, 0x4a, 0x90, 0x01, 0xb1, 0xc0, 
0xf0, 0x00, 0x0c, 0x00, 0x3a, 0x41, 0x3b, 0x41, 0x3c, 0x41, 0x3d, 0x41, 0x3e, 0x41, 0x3f, 0x41, 
0x00, 0x13, 0x1d, 0x42, 0x3a, 0x02, 0x1e, 0x42, 0x38, 0x02, 0x1d, 0x93, 0x04, 0x34, 0x0f, 0x4d, 
0x3f, 0xe3, 0x1f, 0x53, 0x01, 0x3c, 0x0f, 0x4d, 0x1e, 0x93, 0x02, 0x34, 0x3e, 0xe3, 0x1e, 0x53, 
0x0e, 0x9f, 0x03, 0x2c, 0x0c, 0x4e, 0x0e, 0x4f, 0x0f, 0x4c, 0x12, 0xc3, 0x0f, 0x10, 0x0f, 0x11, 
0x0f, 0x5e, 0x1c, 0x43, 0x1f, 0x92, 0xf8, 0x06, 0x17, 0x28, 0x0d, 0x11, 0x1d, 0x82, 0x38, 0x02, 
0x1d, 0x93, 0x02, 0x38, 0x3f, 0x43, 0x01, 0x3c, 0x1f, 0x43, 0xc2, 0x93, 0xfa, 0x06, 0x07, 0x24, 
0x5e, 0x42, 0xfa, 0x06, 0x8e, 0x11, 0x0f, 0x9e, 0x02, 0x24, 0x0c, 0x43, 0x02, 0x3c, 0x82, 0x5f, 
0xee, 0x06, 0xc2, 0x4f, 0xfa, 0x06, 0x30, 0x41, 0xb2, 0x50, 0x14, 0x00, 0xee, 0x06, 0xb2, 0x90, 
0x2d, 0x01, 0xee, 0x06, 0x06, 0x28, 0xb2, 0x80, 0xc8, 0x00, 0xee, 0x06, 0x12, 0xc3, 0x12, 0x10, 
0xf8, 0x06, 0xc2, 0x43, 0xfa, 0x06, 0x30, 0x41, 0x0f, 0x12, 0x5f, 0x42, 0xf7, 0x06, 0x0f, 0x93, 
0x15, 0x24, 0x1f, 0x83, 0x26, 0x24, 0x1f, 0x83, 0x29, 0x20, 0xb2, 0x90, 0x22, 0x00, 0xe6, 0x06, 
0x07, 0x2c, 0x1f, 0x42, 0xe6, 0x06, 0xdf, 0x42, 0xc1, 0x01, 0x00, 0x02, 0x92, 0x53, 0xe6, 0x06, 
0xd2, 0x83, 0xe9, 0x06, 0x1b, 0x20, 0xc2, 0x43, 0xf7, 0x06, 0x18, 0x3c, 0x5f, 0x42, 0xc1, 0x01, 
0x82, 0x4f, 0xe6, 0x06, 0xd2, 0x43, 0xf7, 0x06, 0xd2, 0x4f, 0x00, 0x02, 0xc0, 0x01, 0x3f, 0x90, 
0x06, 0x00, 0x0c, 0x20, 0xf2, 0x40, 0x24, 0x00, 0xe0, 0x01, 0xb2, 0x40, 0x03, 0x00, 0xd8, 0x01, 
0x05, 0x3c, 0xd2, 0x42, 0xc1, 0x01, 0xe9, 0x06, 0xe2, 0x43, 0xf7, 0x06, 0xf2, 0xd0, 0x10, 0x00, 
0xc2, 0x01, 0xf2, 0xd0, 0x20, 0x00, 0xc2, 0x01, 0xb1, 0xc0, 0xf0, 0x00, 0x02, 0x00, 0x3f, 0x41, 
0x00, 0x13, 0x0a, 0x12, 0x1d, 0x93, 0x03, 0x34, 0x3d, 0xe3, 0x1d, 0x53, 0x02, 0x3c, 0x3c, 0xe3, 
0x1c, 0x53, 0x0e, 0x4d, 0x0f, 0x4c, 0x0e, 0x11, 0x0f, 0x11, 0x0b, 0x43, 0x0c, 0x4e, 0x0d, 0x4b, 
0xb0, 0x12, 0x6a, 0xfd, 0x0a, 0x4c, 0x0c, 0x4f, 0x0d, 0x4b, 0xb0, 0x12, 0x6a, 0xfd, 0x1f, 0x93, 
0x03, 0x34, 0x0e, 0x8c, 0x0f, 0x5a, 0x02, 0x3c, 0x0e, 0x5c, 0x0f, 0x8a, 0x1b, 0x53, 0x2b, 0x92, 
0xed, 0x3b, 0x0c, 0x4e, 0x3a, 0x41, 0x30, 0x41, 0x0f, 0x12, 0x0e, 0x12, 0x0d, 0x12, 0x0c, 0x12, 
0x0b, 0x12, 0xe2, 0xb3, 0xe0, 0x01, 0x12, 0x24, 0xd2, 0x42, 0xe0, 0x01, 0xe8, 0x06, 0xe2, 0xc3, 
0xe0, 0x01, 0xa2, 0xc2, 0x92, 0x01, 0x4c, 0x43, 0xf2, 0x90, 0x20, 0x00, 0x01, 0x02, 0x01, 0x24, 
0x5c, 0x43, 0xb0, 0x12, 0x26, 0xf9, 0xb1, 0xc0, 0xf0, 0x00, 0x0a, 0x00, 0x3b, 0x41, 0x3c, 0x41, 
0x3d, 0x41, 0x3e, 0x41, 0x3f, 0x41, 0x00, 0x13, 0x0f, 0x12, 0xc2, 0x43, 0xf7, 0x06, 0x92, 0x53, 
0xe6, 0x06, 0xb2, 0x90, 0xd8, 0x04, 0xe6, 0x06, 0x03, 0x28, 0x82, 0x43, 0xe6, 0x06, 0x05, 0x3c, 
0x1f, 0x42, 0xe6, 0x06, 0xd2, 0x4f, 0x00, 0x02, 0xc0, 0x01, 0xf2, 0xd0, 0x20, 0x00, 0xc2, 0x01, 
0xb1, 0xc0, 0xf0, 0x00, 0x02, 0x00, 0x3f, 0x41, 0x00, 0x13, 0x3d, 0xf0, 0x0f, 0x00, 0x3d, 0xe0, 
0x0f, 0x00, 0x0d, 0x5d, 0x00, 0x5d, 0x0c, 0x11, 0x0c, 0x11, 0x0c, 0x11, 0x0c, 0x11, 0x0c, 0x11, 
0x0c, 0x11, 0x0c, 0x11, 0x0c, 0x11, 0x0c, 0x11, 0x0c, 0x11, 0x0c, 0x11, 0x0c, 0x11, 0x0c, 0x11, 
0x0c, 0x11, 0x0c, 0x11, 0x30, 0x41, 0xd2, 0xd3, 0xe0, 0x01, 0xd2, 0xc3, 0xe0, 0x01, 0xb2, 0x40, 
0x77, 0x06, 0xa6, 0x01, 0x3c, 0x42, 0xb0, 0x12, 0x04, 0xfe, 0xb2, 0x40, 0x77, 0x01, 0xa6, 0x01, 
0xd2, 0x42, 0xe8, 0x06, 0xe0, 0x01, 0x30, 0x41, 0xe2, 0xc3, 0xec, 0x06, 0x92, 0x42, 0xd2, 0x01, 
0x22, 0x02, 0xe2, 0x42, 0x14, 0x02, 0xb1, 0xc0, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x13, 0x92, 0x42, 
0xda, 0x01, 0x0a, 0x02, 0x82, 0x43, 0xd8, 0x01, 0xe2, 0x42, 0xe0, 0x01, 0xb1, 0xc0, 0xf0, 0x00, 
0x00, 0x00, 0x00, 0x13, 0x31, 0x40, 0x00, 0x0a, 0xb0, 0x12, 0x1a, 0xfe, 0x0c, 0x43, 0xb0, 0x12, 
0x38, 0xfa, 0xb0, 0x12, 0x1e, 0xfe, 0xb2, 0x40, 0x77, 0x13, 0xa6, 0x01, 0xb1, 0xc0, 0xf0, 0x00, 
0x00, 0x00, 0x00, 0x13, 0x1c, 0x83, 0x03, 0x43, 0xfd, 0x23, 0x30, 0x41, 0xb1, 0xc0, 0xf0, 0x00, 
0x00, 0x00, 0x00, 0x13, 0x32, 0xd0, 0x10, 0x00, 0xfd, 0x3f, 0x1c, 0x43, 0x30, 0x41, 0x03, 0x43, 
0xff, 0x3f, 0x00, 0x13, };

const unsigned char ch201_presence_fw_vec[CH201_PRESENCE_VEC_SIZE] = {
0x38, 0xfd, 0x38, 0xfc, 0x22, 0xfe, 0xce, 0xfd, 0xf8, 0xfc, 0x00, 0x00, 0x14, 0xfe, 0x00, 0xf8, 
0x0c, 0xfe, 0xf6, 0xfd, 0x14, 0xfe, 0x00, 0x00, 0xb8, 0xfd, 0x1a, 0xfb, 0x14, 0xfe, 0xe4, 0xfd, 
};

