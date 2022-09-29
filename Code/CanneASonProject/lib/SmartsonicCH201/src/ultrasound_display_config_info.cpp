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
#include <stdbool.h>
#include <string.h>

#include "ultrasound_display_config_info.h"
#include "soniclib.h"

static bool is_part_number_icu(ch_dev_t *dev_ptr)
{
	return  ((ch_get_part_number(dev_ptr) == ICU10201_PART_NUMBER) ||
			 (ch_get_part_number(dev_ptr) == ICU20201_PART_NUMBER));
}

/*
 * display_config_info() - display the configuration values for a sensor
 *
 * This function displays the current configuration settings for an individual
 * sensor.  The operating mode, maximum range, and static target rejection
 * range (if used) are displayed.
 *
 * For CH201 and ICU202XX sensors, the multiple detection threshold values are also
 * displayed.
 */
uint8_t ultrasound_display_config_info(ch_dev_t *dev_ptr)
{
	ch_config_t 	read_config;
	uint8_t 		chirp_error;
	uint8_t 		dev_num = ch_get_dev_num(dev_ptr);
	const char 		*mode_string;

	/* Read configuration values for the device into ch_config_t structure */
	chirp_error = ch_get_config(dev_ptr, &read_config);

	if (!chirp_error) {
		switch (read_config.mode) {
			case CH_MODE_IDLE:
				mode_string = "IDLE";
				break;
			case CH_MODE_FREERUN:
				mode_string = "FREERUN";
				break;
			case CH_MODE_TRIGGERED_TX_RX:
				mode_string = "TRIGGERED_TX_RX";
				break;
			case CH_MODE_TRIGGERED_RX_ONLY:
				mode_string = "TRIGGERED_RX_ONLY";
				break;
			default:
				mode_string = "UNKNOWN";
		}

		if (!is_part_number_icu(dev_ptr)) {
			/* Display sensor number, mode and max range */
			printf("Sensor %u:\tmax_range=%dmm (%d decimated samples)\tmode=%s  ", dev_num,
				read_config.max_range, ch_get_num_samples(dev_ptr), mode_string);

			/* Display static target rejection range, if used */
			if (read_config.static_range != 0) {
				printf("static_range=%d samples", read_config.static_range);
			}
		}

#ifdef INCLUDE_SHASTA_SUPPORT
		if (is_part_number_icu(dev_ptr)) {
			/* Display measurement configuration */
			ch_meas_info_t meas_info;
			ch_meas_seg_info_t seg_info;

			printf("Sensor %u:\n", dev_num);
			for (uint8_t meas_num = 0; meas_num < MEAS_QUEUE_MAX_MEAS; meas_num++) {
				ch_meas_get_info(dev_ptr, meas_num, &meas_info);

				if (0 == meas_info.num_rx_samples)
					continue;
				printf("Measurement %u Configuration\n", meas_num);

				printf("  Total Samples = %u  (%u mm max range) at ODR = %u\tRingdown samples = %u\n",
						meas_info.num_rx_samples, ch_samples_to_mm(dev_ptr, meas_info.num_rx_samples),
						meas_info.odr, meas_info.ringdown_cancel_samples);

				printf("  Active Segments = %u\n", meas_info.num_segments);
				if (0 == meas_info.num_segments)
					continue;
				for (int seg_num = 0; seg_num <= meas_info.num_segments; seg_num++) { // also display EOF
					char *type_string;

					ch_meas_get_seg_info(dev_ptr, meas_num, seg_num, &seg_info);

					if (seg_info.type == CH_MEAS_SEG_TYPE_COUNT) {
						type_string = "Count";
					} else if (seg_info.type == CH_MEAS_SEG_TYPE_RX) {
						type_string = "RX";
					} else if (seg_info.type == CH_MEAS_SEG_TYPE_TX) {
						type_string = "TX";
					} else if (seg_info.type == CH_MEAS_SEG_TYPE_EOF) {
						continue;
					} else {
						type_string = "UNKNOWN";
					}

					printf("\tSeg %d  %s\t", seg_num, type_string);

					if (seg_info.type == CH_MEAS_SEG_TYPE_RX) {
						printf("%3d rx sample(s) periods = %5d cycles\t", seg_info.num_rx_samples, seg_info.num_cycles);
					} else if (seg_info.type == CH_MEAS_SEG_TYPE_TX) {
						printf("                 periods = %5d cycles\t", seg_info.num_cycles);
					}

					if (seg_info.type == CH_MEAS_SEG_TYPE_TX) {
						printf("Pulse width = %2d\tPhase = %d\t", seg_info.tx_pulse_width, seg_info.tx_phase);
					} else if (seg_info.type == CH_MEAS_SEG_TYPE_RX) {
						printf("Gain reduce = %2d\tAtten = %d\t", seg_info.rx_gain, seg_info.rx_atten);
					}

					if (seg_info.rdy_int_en) {
						printf("Rdy Int\t");
					}
					if (seg_info.done_int_en) {
						printf("Done Int");
					}
					printf("\n");
				}
			}
		}
#endif // INCLUDE_SHASTA_SUPPORT

		/* If embedded ASIC FW integrates target detection */
#if defined (CH_NUM_THRESHOLDS)
		/* Display detection thresholds (not supported on CH101) */
		if ((ch_get_part_number(dev_ptr) != CH101_PART_NUMBER) && (dev_ptr->max_num_thresholds != 0)) {
			ch_thresholds_t read_thresholds;

			/* Get threshold values in structure */
			chirp_error = ch_get_thresholds(dev_ptr, &read_thresholds);

			if (!chirp_error) {
				printf("Detection thresholds:\n");
				for (int thresh_num = 0; thresh_num < CH_NUM_THRESHOLDS; thresh_num++) {
					uint16_t start_sample = read_thresholds.threshold[thresh_num].start_sample;
					uint16_t start_mm = ch_samples_to_mm(dev_ptr, start_sample);

					if ((thresh_num == 0) || (start_sample != 0)) {	// unused thresholds have start=0
						printf("     %d\tstart sample: %3d  = %4d mm\tlevel: %d",
							thresh_num, start_sample, start_mm,
							read_thresholds.threshold[thresh_num].level);
	#ifdef INCLUDE_SHASTA_SUPPORT
						if (read_thresholds.threshold[thresh_num].level == CH_THRESH_LEVEL_HOLDOFF) {
							printf(" (Rx Holdoff)");
						}
	#endif // INCLUDE_SHASTA_SUPPORT
						printf("\n");
					}
				}
			} else {
				printf(" Device %d: Error during ch_get_thresholds()", dev_num);
			}
		}
		printf("\n");
#endif /* CH_NUM_THRESHOLDS */
	} else {
		printf(" Device %d: Error during ch_get_config()\n", dev_num);
	}

	return chirp_error;
}

/* Get and display the initialization results for each connected sensor.
 *   This loop checks each device number in the sensor group to determine
 *   if a sensor is actually connected.  If so, it makes a series of
 *   function calls to get different operating values, including the
 *   operating frequency, clock calibration values, and firmware version.
 */
void ultrasound_display_connected_sensors_measures_info(ch_group_t *grp_ptr)
{
	uint8_t num_ports = ch_get_num_ports(grp_ptr);

	printf("Sensor  Type       Freq      RTC Cal     B/W   Firmware\r\n");
	for (uint8_t dev_num = 0; dev_num < num_ports; dev_num++) {
		ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);
		if (ch_sensor_is_connected(dev_ptr)) {
			printf("  %u     %s%d  %u Hz  %u@%ums  %u  %s\r\n", dev_num,
				(is_part_number_icu(dev_ptr)) ? "ICU-" : "    CH",
				ch_get_part_number(dev_ptr),
				(unsigned int) ch_get_frequency(dev_ptr),
				ch_get_rtc_cal_result(dev_ptr),
				ch_get_rtc_cal_pulselength(dev_ptr),
				ch_get_bandwidth(dev_ptr),
				ch_get_fw_version_string(dev_ptr));
		}
	}
	printf("\n");
}

void ultrasound_display_connected_sensors_otp_info(__attribute__((unused)) ch_group_t *grp_ptr)
{
#ifdef INCLUDE_SHASTA_SUPPORT
	uint8_t num_ports = ch_get_num_ports(grp_ptr);
	const char *sensor_id_str;
	ch_mfg_info_t manufacturing_info;

	printf("Sensor\tId\tYear\tWeek\tSite\tProduct\tPackage\tMEMS\tModule\r\n");
	for (uint8_t dev_num = 0; dev_num < num_ports; dev_num++) {
		ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);

		if (ch_sensor_is_connected(dev_ptr)) {
			sensor_id_str = ch_get_sensor_id(dev_ptr);
			if (0 == strcmp(sensor_id_str, "DEFAULT")) {
				printf("%u\tNot programmed\r\n", dev_num);
			} else {
				ch_get_mfg_info(dev_ptr, &manufacturing_info);
				printf("%u\t%s\t%04u\tW%02u\t%u\t%03u\t%u\t%u\t%u\r\n",
					dev_num, sensor_id_str,
					manufacturing_info.mfg_year,manufacturing_info.mfg_week,
					manufacturing_info.mfg_site, manufacturing_info.product_code,
					manufacturing_info.package_code, manufacturing_info.mems_code,
					manufacturing_info.module_code);
			}
		}
	}
#endif // INCLUDE_SHASTA_SUPPORT
}
