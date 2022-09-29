/*! \file ch101_liquid.c
 *
 * \brief Chirp CH101 Liquid Level Sensing firmware interface
 * 
 * This file contains function definitions to interface a specific sensor firmware 
 * package to SonicLib, including the main initialization routine for the firmware.  
 * That routine initializes various fields within the \a ch_dev_t device descriptor 
 * and specifies the proper functions to implement SonicLib API calls.  Those may 
 * either be common implementations or firmware-specific routines located in this file.
 */

/*
 Copyright © 2019-2022, Chirp Microsystems.  All rights reserved.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL CHIRP MICROSYSTEMS BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 You can contact the authors of this program by email at support@chirpmicro.com
 or by mail at 2560 Ninth Street, Suite 220, Berkeley, CA 94710.
 */


#include "soniclib.h"
#include "ch101_liquid.h"
#include "ch_common.h"
#include "ch_math_utils.h"
#include "chirp_bsp.h"

void ch101_liquid_store_op_freq(ch_dev_t *dev_ptr);
void ch101_liquid_store_scale_factor(ch_dev_t *dev_ptr);
uint16_t ch101_liquid_get_amplitude(ch_dev_t *dev_ptr);
uint32_t ch101_liquid_get_range(ch_dev_t *dev_ptr, ch_range_t range_type);
uint8_t  ch101_liquid_get_iq_data(ch_dev_t *dev_ptr, ch_iq_sample_t *buf_ptr, uint16_t start_sample, uint16_t num_samples,
ch_io_mode_t mode);
static uint8_t get_sample_data(ch_dev_t *dev_ptr, ch_iq_sample_t *buf_ptr, uint16_t start_sample, uint16_t num_samples,
ch_io_mode_t mode, uint8_t sample_size_in_bytes);
uint8_t ch101_liquid_set_thresholds(ch_dev_t *dev_ptr, ch_thresholds_t *lib_thresh_buf_ptr);
uint8_t ch101_liquid_get_thresholds(ch_dev_t *dev_ptr, ch_thresholds_t *lib_thresh_buf_ptr);

uint8_t ch101_liquid_init(ch_dev_t *dev_ptr, ch_group_t *grp_ptr, uint8_t i2c_addr, uint8_t io_index, uint8_t bus_index) {
	
	(void)grp_ptr;

	dev_ptr->part_number = CH101_PART_NUMBER;
	dev_ptr->app_i2c_address = i2c_addr;
	dev_ptr->io_index = io_index;
	dev_ptr->bus_index = bus_index;

	dev_ptr->freqCounterCycles = CH101_COMMON_FREQCOUNTERCYCLES;
	dev_ptr->freqLockValue     = CH101_COMMON_READY_FREQ_LOCKED;

	/* Init firmware-specific function pointers */
	dev_ptr->fw_text 					= ch101_liquid_fw_text;
	dev_ptr->fw_text_size 				= ch101_liquid_text_size;
	dev_ptr->fw_vec 					= ch101_liquid_fw_vec;
	dev_ptr->fw_vec_size 				= ch101_liquid_vec_size;
	dev_ptr->fw_version_string			= ch101_liquid_version;
	dev_ptr->ram_init 					= get_ram_ch101_liquid_init_ptr();
	dev_ptr->get_fw_ram_init_size 		= get_ch101_liquid_fw_ram_init_size;
	dev_ptr->get_fw_ram_init_addr 		= get_ch101_liquid_fw_ram_init_addr;

	dev_ptr->prepare_pulse_timer 		= ch_common_prepare_pulse_timer;
	dev_ptr->store_pt_result 			= ch_common_store_pt_result;
	dev_ptr->store_op_freq 				= ch101_liquid_store_op_freq;
	dev_ptr->store_bandwidth 			= NULL;
	dev_ptr->store_scalefactor 			= ch101_liquid_store_scale_factor;
	dev_ptr->get_locked_state 			= ch_common_get_locked_state;

	/* Init API function pointers */
	dev_ptr->api_funcs.fw_load          = ch_common_fw_load;
	dev_ptr->api_funcs.set_mode         = ch_common_set_mode;
	dev_ptr->api_funcs.set_sample_interval  = ch_common_set_sample_interval;
	dev_ptr->api_funcs.set_num_samples  = ch_common_set_num_samples;
	dev_ptr->api_funcs.set_max_range    = ch_common_set_max_range;
	dev_ptr->api_funcs.set_static_range = NULL;
	dev_ptr->api_funcs.get_range        = ch101_liquid_get_range;
	dev_ptr->api_funcs.get_amplitude    = ch101_liquid_get_amplitude;
	dev_ptr->api_funcs.get_iq_data      = ch101_liquid_get_iq_data;
	dev_ptr->api_funcs.get_amplitude_data = NULL; // Not supported
	dev_ptr->api_funcs.samples_to_mm    = ch_common_samples_to_mm;
	dev_ptr->api_funcs.mm_to_samples    = ch_common_mm_to_samples;
	dev_ptr->api_funcs.set_thresholds   = ch101_liquid_set_thresholds;
	dev_ptr->api_funcs.get_thresholds   = ch101_liquid_get_thresholds;

	/* Init max sample count */
	dev_ptr->max_samples = CH101_LIQUID_MAX_SAMPLES;

	/* This firmware uses oversampling */
	dev_ptr->oversample = 2;			// 4x oversampling (value is power of 2)

	return 0;
}

void ch101_liquid_store_op_freq(ch_dev_t *dev_ptr){
	uint8_t	 tof_sf_reg;
	uint16_t raw_freq;		// aka scale factor
	uint32_t freq_counter_cycles;
	uint32_t num;
	uint32_t den;
	uint32_t op_freq;

	tof_sf_reg = CH101_LIQUID_REG_TOF_SF;

	freq_counter_cycles = dev_ptr->freqCounterCycles;

	chdrv_read_word(dev_ptr, tof_sf_reg, &raw_freq);

	num = (uint32_t)(((dev_ptr->rtc_cal_result)*1000U) / (16U * freq_counter_cycles)) * (uint32_t)(raw_freq);
	den = (uint32_t)(dev_ptr->group->rtc_cal_pulse_ms);
	op_freq = (num/den);

	dev_ptr->op_frequency = op_freq;
}

void ch101_liquid_store_scale_factor(ch_dev_t *dev_ptr) {
	uint8_t	err;
	uint8_t	tof_sf_reg;
	uint16_t scale_factor;

	tof_sf_reg = CH101_LIQUID_REG_TOF_SF;

	err = chdrv_read_word(dev_ptr, tof_sf_reg, &scale_factor);
	if (!err) {
		dev_ptr->scale_factor = scale_factor;
		} else {
		dev_ptr->scale_factor = 0;
	}
}

uint16_t ch101_liquid_get_amplitude(ch_dev_t *dev_ptr) {
	uint16_t amplitude = 0;

	uint16_t  amplitude_reg = 0;
	if (dev_ptr->sensor_connected) {
		amplitude_reg = CH101_LIQUID_REG_AMPLITUDE;

		chdrv_read_word(dev_ptr, amplitude_reg, &amplitude);
	}

	return amplitude;
}

uint32_t ch101_liquid_get_range(ch_dev_t *dev_ptr, ch_range_t range_type) {
	uint8_t		tof_reg;
	uint32_t	range = CH_NO_TARGET;
	uint32_t 	time_of_flight = 0;
	int 		err;
	uint16_t 	scale_factor;

	if (dev_ptr->sensor_connected) {
		tof_reg = CH101_LIQUID_REG_TOF;

		uint16_t time_of_flight_16bit;
		err = chdrv_read_word(dev_ptr, tof_reg, &time_of_flight_16bit);
		time_of_flight = (uint32_t)time_of_flight_16bit;

		if (!err && (time_of_flight != UINT16_MAX)) { // If object detected

			if (dev_ptr->scale_factor == 0) {
				ch101_liquid_store_scale_factor(dev_ptr);
			}
			scale_factor = dev_ptr->scale_factor;

			if (scale_factor != 0) {
				uint32_t num = (CH_SPEEDOFSOUND_MPS * dev_ptr->group->rtc_cal_pulse_ms * (uint32_t) time_of_flight);
				uint32_t den = ((uint32_t) dev_ptr->rtc_cal_result * (uint32_t) scale_factor) >> 11;		// XXX need define

				range = (num / den);

				if (range_type == CH_RANGE_ECHO_ONE_WAY) {
					range /= 2;
				}

				/* Adjust for oversampling, if used */
				range >>= dev_ptr->oversample;

				/* If rx-only node, adjust for pre-trigger time included in ToF */
				if (dev_ptr->mode == CH_MODE_TRIGGERED_RX_ONLY) {
					uint32_t pretrig_adj = (CH_SPEEDOFSOUND_MPS * dev_ptr->group->pretrig_delay_us * 32) / 1000;

					if (range > pretrig_adj) {
						range -= pretrig_adj;			// subtract adjustment from calculated range
						} else {
						range = CH_MIN_RANGE_VAL;		// underflow - range is very close to zero, use minimum value
					}
				}
			}
		}
	}
	return range;
}

uint8_t  ch101_liquid_get_iq_data(ch_dev_t *dev_ptr, ch_iq_sample_t *buf_ptr, uint16_t start_sample, uint16_t num_samples,
ch_io_mode_t mode) {

	return get_sample_data(dev_ptr, buf_ptr, start_sample, num_samples, mode, sizeof(ch_iq_sample_t));
}

static uint8_t get_sample_data(ch_dev_t *dev_ptr, ch_iq_sample_t *buf_ptr, uint16_t start_sample, uint16_t num_samples,
ch_io_mode_t mode, uint8_t sample_size_in_bytes) {

	uint16_t iq_data_addr;
	uint16_t num_bytes = (num_samples * sample_size_in_bytes);
	ch_group_t *grp_ptr = dev_ptr->group;
	int      err = 0;

	if ((num_samples == 0) || ((start_sample + num_samples) > dev_ptr->max_samples)) {
		return 1;
	}
	uint8_t	   use_prog_read = 0;		// default = do not use low-level programming interface

	#ifndef USE_STD_I2C_FOR_IQ
	if (grp_ptr->num_connected[dev_ptr->bus_index] == 1) {		// if only one device on this bus
		use_prog_read = 1;											//   use low-level interface
	}
	#endif

	iq_data_addr = CH101_LIQUID_REG_DATA;

	iq_data_addr += (start_sample * sample_size_in_bytes);

	if (mode == CH_IO_MODE_BLOCK) {
		/* blocking transfer */

		if (use_prog_read) {
			/* use low-level programming interface for speed */

			int num_transfers = (num_bytes + (CH_PROG_XFER_SIZE - 1)) / CH_PROG_XFER_SIZE;
			int bytes_left = num_bytes;       // remaining bytes to read

			/* Convert register offsets to full memory addresses */
			iq_data_addr += CH101_DATA_MEM_ADDR + CH101_COMMON_I2CREGS_OFFSET;

			chbsp_program_enable(dev_ptr);					// assert PROG pin

			for (int xfer = 0; xfer < num_transfers; xfer++) {
				int bytes_to_read;
				uint8_t message[] = { (0x80 | CH_PROG_REG_CTL), 0x09 };      // read burst command

				if (bytes_left > CH_PROG_XFER_SIZE) {
					bytes_to_read = CH_PROG_XFER_SIZE;
					} else {
					bytes_to_read = bytes_left;
				}
				chdrv_prog_write(dev_ptr, CH_PROG_REG_ADDR, (iq_data_addr + (xfer * CH_PROG_XFER_SIZE)));
				chdrv_prog_write(dev_ptr, CH_PROG_REG_CNT, (bytes_to_read - 1));
				err = chdrv_prog_i2c_write(dev_ptr, message, sizeof(message));
				err |= chdrv_prog_i2c_read(dev_ptr, ((uint8_t *)buf_ptr + (xfer * CH_PROG_XFER_SIZE)), bytes_to_read);

				bytes_left -= bytes_to_read;
			}
			chbsp_program_disable(dev_ptr);					// de-assert PROG pin

			} else {	/* if (use_prog_read) */
			/* use standard I2C interface */

			err = chdrv_burst_read(dev_ptr, iq_data_addr, (uint8_t *) buf_ptr, num_bytes);
		}

		} else {
		/* non-blocking transfer - queue a read transaction (must be started using ch_io_start_nb() ) */

		if (use_prog_read && (grp_ptr->i2c_drv_flags & I2C_DRV_FLAG_USE_PROG_NB)) {
			/* Use low-level programming interface to read data */

			/* Convert register offsets to full memory addresses */
			iq_data_addr += (CH101_DATA_MEM_ADDR + CH101_COMMON_I2CREGS_OFFSET);

			err = chdrv_group_queue(grp_ptr, dev_ptr, 1, CHDRV_NB_TRANS_TYPE_PROG, iq_data_addr, num_bytes,
			(uint8_t *) buf_ptr);
			} else {
			/* Use regular I2C register interface to read data */
			err = chdrv_group_queue(grp_ptr, dev_ptr, 1, CHDRV_NB_TRANS_TYPE_STD, iq_data_addr, num_bytes,
			(uint8_t*) buf_ptr);
		}
	}

	return err;
}

uint8_t ch101_liquid_set_thresholds(ch_dev_t *dev_ptr, ch_thresholds_t *lib_thresh_buf_ptr) {	// XXX TODO only meas 0 for now
	int ret_val = 1;		// default is error return

	uint8_t	thresh_len_reg = 0;		// offset of register for this threshold's length
	uint8_t thresh_level_reg;	// threshold level reg (first in array)
	uint8_t max_num_thresholds = 0;
	uint8_t	thresh_num;
	uint8_t thresh_len;
	uint16_t thresh_level;
	uint16_t start_sample = 0;

	if (dev_ptr->sensor_connected) {
		
		thresh_level_reg = CH101_LIQUID_REG_THRESHOLDS;
		max_num_thresholds = CH101_LIQUID_NUM_THRESHOLDS;

		for (thresh_num = 0; thresh_num < max_num_thresholds; thresh_num++) {

			if (thresh_num < (max_num_thresholds - 1)) {
				uint16_t next_start_sample = lib_thresh_buf_ptr->threshold[thresh_num + 1].start_sample;

				thresh_len = (next_start_sample - start_sample);
				start_sample  = next_start_sample;
				} else {
				thresh_len = 0;
			}

			if (thresh_num == 0) {
				thresh_len_reg = CH101_LIQUID_REG_THRESH_LEN_0;
				} else if (thresh_num == 1) {
				thresh_len_reg = CH101_LIQUID_REG_THRESH_LEN_1;
				} else if (thresh_num == 2) {
				thresh_len_reg = CH101_LIQUID_REG_THRESH_LEN_2;
				} else if (thresh_num == 3) {
				thresh_len_reg = CH101_LIQUID_REG_THRESH_LEN_3;
				} else if (thresh_num == 4) {
				thresh_len_reg = CH101_LIQUID_REG_THRESH_LEN_4;
				} else if (thresh_num == 5) {
				thresh_len_reg = 0;			// last threshold does not have length field - assumed to extend to end of data
			}

			if (thresh_len_reg != 0) {
				ret_val = chdrv_write_byte(dev_ptr, thresh_len_reg, thresh_len); 	// set the length field (if any) for this threshold
			}
			// write level to this threshold's entry in register array
			thresh_level = lib_thresh_buf_ptr->threshold[thresh_num].level;
			ret_val |= chdrv_write_word(dev_ptr, (thresh_level_reg + (thresh_num * sizeof(uint16_t))), thresh_level);
		}	// end 	for (thresh_num = 0; thresh_num < max_num_thresholds; thresh_num++)

	}	// end 	if (dev_ptr->sensor_connected)
	return ret_val;
}

uint8_t ch101_liquid_get_thresholds(ch_dev_t *dev_ptr, ch_thresholds_t *lib_thresh_buf_ptr) {
	uint8_t ret_val = 1;		// default = error return
	uint8_t	thresh_len_reg = 0;		// offset of register for this threshold's length
	uint8_t thresh_level_reg;	// threshold level reg (first in array)
	uint8_t max_num_thresholds;
	uint8_t thresh_num;
	uint8_t	thresh_len = 0;		// number of samples described by each threshold
	uint16_t	start_sample = 0;	// calculated start sample for each threshold

	if (dev_ptr->sensor_connected && (lib_thresh_buf_ptr != NULL)) {
			thresh_level_reg = CH101_LIQUID_REG_THRESHOLDS;
			max_num_thresholds = CH101_LIQUID_NUM_THRESHOLDS;

		for (thresh_num = 0; thresh_num < max_num_thresholds; thresh_num++) {
				if (thresh_num == 0) {
					thresh_len_reg = CH101_LIQUID_REG_THRESH_LEN_0;
					} else if (thresh_num == 1) {
					thresh_len_reg = CH101_LIQUID_REG_THRESH_LEN_1;
					} else if (thresh_num == 2) {
					thresh_len_reg = CH101_LIQUID_REG_THRESH_LEN_2;
					} else if (thresh_num == 3) {
					thresh_len_reg = CH101_LIQUID_REG_THRESH_LEN_3;
					} else if (thresh_num == 4) {
					thresh_len_reg = CH101_LIQUID_REG_THRESH_LEN_4;
					} else if (thresh_num == 5) {
					thresh_len_reg = 0;			// last threshold does not have length field - assumed to extend to end of data
				}

			if (thresh_len_reg != 0) {
				// read the length field register for this threshold
				chdrv_read_byte(dev_ptr, thresh_len_reg, &thresh_len);
				} else {
				thresh_len = 0;
			}

			lib_thresh_buf_ptr->threshold[thresh_num].start_sample = start_sample;
			start_sample += thresh_len;				// increment start sample for next threshold

			// get level from this threshold's entry in register array
			chdrv_read_word(dev_ptr, (thresh_level_reg + (thresh_num * sizeof(uint16_t))),
			&(lib_thresh_buf_ptr->threshold[thresh_num].level));

		}
		ret_val = 0;	// return OK
	}
	return ret_val;
}