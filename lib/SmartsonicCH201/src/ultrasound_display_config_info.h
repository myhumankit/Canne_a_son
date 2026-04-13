/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2021 InvenSense Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively “Software”) is subject
 * to InvenSense and its licensors' intellectual property rights under U.S. and international copyright
 * and other intellectual property rights laws.
 *
 * InvenSense and its licensors retain all intellectual property and proprietary rights in and to the Software
 * and any use, reproduction, disclosure or distribution of the Software without an express license agreement
 * from InvenSense is strictly prohibited.
 *
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, THE SOFTWARE IS
 * PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, IN NO EVENT SHALL
 * INVENSENSE BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, OR ANY
 * DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 * ________________________________________________________________________________________________________
 */

/*! \file ultrasound_display_config_info.h */
#ifndef ULTRASOUND_DISPLAY_CONFIG_INFO_H
#define ULTRASOUND_DISPLAY_CONFIG_INFO_H

#include <stdint.h>

#include "soniclib.h"

/*
 * Display the configuration values for a sensor
 *
 * This function displays the current configuration settings for an individual
 * sensor.  The operating mode, maximum range, and static target rejection
 * range (if used) are displayed.
 *
 * For CH201 sensors only, the multiple detection threshold values are also
 * displayed.
 */
uint8_t ultrasound_display_config_info(ch_dev_t *dev_ptr);

/*
 * For every board sensor print :
 * part number, fop, calibration results, bandwidth, fw version
 */
void ultrasound_display_connected_sensors_measures_info(ch_group_t *grp_ptr);

/*
 * Print for all connected sensors their OTP information
 */
void ultrasound_display_connected_sensors_otp_info(ch_group_t *grp_ptr);

#endif /* ULTRASOUND_DISPLAY_CONFIG_INFO_H */
