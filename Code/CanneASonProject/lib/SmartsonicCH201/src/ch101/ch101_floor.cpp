/*! \file ch101_floor.c
 *
 * \brief Chirp CH101 Floor Detection firmware interface
 * 
 * This file contains function definitions to interface a specific sensor firmware 
 * package to SonicLib, including the main initialization routine for the firmware.  
 * That routine initializes various fields within the \a ch_dev_t device descriptor 
 * and specifies the proper functions to implement SonicLib API calls.  Those may 
 * either be common implementations or firmware-specific routines located in this file.
 */

/*
 Copyright � 2019-2022, Chirp Microsystems.  All rights reserved.

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
#include "ch101_floor.h"
#include "ch_common.h"
#include "ch_math_utils.h"
#include "chirp_bsp.h"

/* Forward references */
uint8_t	 ch101_floor_set_sample_window(ch_dev_t *dev_ptr, uint16_t start_sample, uint16_t end_sample);
uint16_t ch101_floor_get_amplitude_avg(ch_dev_t *dev_ptr);
uint8_t  ch101_floor_set_data_output(ch_dev_t *dev_ptr, ch_output_t *output);
uint8_t  ch101_floor_get_amplitude_data(ch_dev_t *dev_ptr, uint16_t *buf_ptr, uint16_t start_sample, uint16_t num_samples, ch_io_mode_t mode);
uint8_t	 ch101_floor_set_tx_length(ch_dev_t *dev_ptr, uint16_t num_cycles);
uint16_t ch101_floor_get_tx_length(ch_dev_t *dev_ptr);
void ch101_floor_trigger_soft(ch_dev_t *dev_ptr);
ch_meas_status_t ch101_floor_meas_get_status(ch_dev_t *dev_ptr, uint8_t meas_num);

uint8_t ch101_floor_init(ch_dev_t *dev_ptr, ch_group_t *grp_ptr, uint8_t i2c_addr, uint8_t io_index, uint8_t bus_index) {
	
	(void)grp_ptr;

	dev_ptr->part_number = CH101_PART_NUMBER;
	dev_ptr->app_i2c_address = i2c_addr;
	dev_ptr->io_index = io_index;
	dev_ptr->bus_index = bus_index;

	dev_ptr->freqCounterCycles = CH101_COMMON_FREQCOUNTERCYCLES;
	dev_ptr->freqLockValue     = CH101_COMMON_READY_FREQ_LOCKED;

	/* Init firmware-specific function pointers */
	dev_ptr->fw_text 					= ch101_floor_fw_text;
	dev_ptr->fw_text_size 				= ch101_floor_text_size;
	dev_ptr->fw_vec 					= ch101_floor_fw_vec;
	dev_ptr->fw_vec_size 				= ch101_floor_vec_size;
	dev_ptr->fw_version_string			= ch101_floor_version;
	dev_ptr->ram_init 					= get_ram_ch101_floor_init_ptr();
	dev_ptr->get_fw_ram_init_size 		= get_ch101_floor_fw_ram_init_size;
	dev_ptr->get_fw_ram_init_addr 		= get_ch101_floor_fw_ram_init_addr;

	dev_ptr->prepare_pulse_timer 		= ch_common_prepare_pulse_timer;
	dev_ptr->store_pt_result 			= ch_common_store_pt_result;
	dev_ptr->store_op_freq 				= ch_common_store_op_freq;
	dev_ptr->store_bandwidth 			= NULL;
	dev_ptr->store_scalefactor 			= ch_common_store_scale_factor;
	dev_ptr->get_locked_state 			= ch_common_get_locked_state;

	/* Init API function pointers */
	dev_ptr->api_funcs.fw_load          = ch_common_fw_load;
	dev_ptr->api_funcs.set_mode         = ch_common_set_mode;
	dev_ptr->api_funcs.set_sample_interval  = ch_common_set_sample_interval;
	dev_ptr->api_funcs.set_num_samples  = ch_common_set_num_samples;
	dev_ptr->api_funcs.set_max_range    = ch_common_set_max_range;
	dev_ptr->api_funcs.set_static_range = NULL;
	dev_ptr->api_funcs.get_range        = NULL;
	dev_ptr->api_funcs.get_amplitude    = NULL;
	dev_ptr->api_funcs.get_amplitude_avg    = ch101_floor_get_amplitude_avg;
	dev_ptr->api_funcs.set_sample_window    = ch101_floor_set_sample_window;
	dev_ptr->api_funcs.get_iq_data      = ch_common_get_iq_data;
	dev_ptr->api_funcs.get_amplitude_data = ch101_floor_get_amplitude_data;
	dev_ptr->api_funcs.samples_to_mm    = ch_common_samples_to_mm;
	dev_ptr->api_funcs.mm_to_samples    = ch_common_mm_to_samples;
	dev_ptr->api_funcs.set_thresholds   = NULL;								// not supported
	dev_ptr->api_funcs.get_thresholds   = NULL;								// not supported
	dev_ptr->api_funcs.set_data_output  = ch101_floor_set_data_output;
	dev_ptr->api_funcs.set_cal_result   = ch_common_set_cal_result;
	dev_ptr->api_funcs.get_cal_result   = ch_common_get_cal_result;
	dev_ptr->api_funcs.set_tx_length    = ch101_floor_set_tx_length;
	dev_ptr->api_funcs.get_tx_length    = ch101_floor_get_tx_length;	
	dev_ptr->api_funcs.trigger_soft		= ch101_floor_trigger_soft;
	dev_ptr->api_funcs.meas_get_status	= ch101_floor_meas_get_status;
	
	/* Init max sample count */
	dev_ptr->max_samples = CH101_FLOOR_MAX_SAMPLES;

	/* This firmware uses oversampling */
	dev_ptr->oversample = 2;			// 4x oversampling (value is power of 2)

	return 0;
}


uint8_t	 ch101_floor_set_sample_window(ch_dev_t *dev_ptr, uint16_t start_sample, uint16_t end_sample) {
	uint8_t	ret_val = 0;
	uint16_t num_win_samples = ((end_sample - start_sample) + 1);

	/* Check input parameters */
	if ((num_win_samples > CH101_FLOOR_MAX_SAMPLES) || 
		(start_sample >= CH101_FLOOR_MAX_SAMPLES) || 
		(end_sample > CH101_FLOOR_MAX_SAMPLES)) {

		ret_val = 1;	/* error */
	}

	/* Write window start and end registers */
	if (!ret_val) {
		chdrv_write_byte(dev_ptr, CH101_FLOOR_REG_RX_HOLDOFF, start_sample);

		chdrv_write_byte(dev_ptr, CH101_FLOOR_REG_RX_WIN_END, (end_sample + 1));

		dev_ptr->win_start_sample = start_sample;
		dev_ptr->num_win_samples = num_win_samples;
	}

	return ret_val;
}

#define	AMP_CORDIC_CORRECT_NUM	 (122)		/* numerator of Cordic amplitude correction (1.22) */
#define	AMP_CORDIC_CORRECT_DEN	 (100)		/* denominator of Cordic amplitude correction (1.22) */

uint16_t ch101_floor_get_amplitude_avg(ch_dev_t *dev_ptr) {
	uint32_t amp_total;
	uint16_t amp_lo;
	uint16_t amp_hi;
	uint16_t amp_avg = 0;
	uint8_t err = 0;

	/* Read total amplitude across window (two 16-bit halves) */

	err = chdrv_read_word(dev_ptr, CH101_FLOOR_REG_AMPLITUDE_LOW, &amp_lo);

	if (!err) {
		err = chdrv_read_word(dev_ptr, CH101_FLOOR_REG_AMPLITUDE_HIGH, &amp_hi);
	}


	/* Combine values and calculate average */
	if (!err) {
		amp_total = (amp_hi << 16) | amp_lo;

		amp_avg = (uint16_t) ((amp_total * AMP_CORDIC_CORRECT_NUM) / (dev_ptr->num_win_samples * AMP_CORDIC_CORRECT_DEN));
	}

	return amp_avg;
}

uint8_t ch101_floor_set_data_output(ch_dev_t *dev_ptr, ch_output_t *output_ptr) {
	uint8_t ret = RET_OK;
	uint8_t reg_value = 0;
	
	if (dev_ptr->sensor_connected) {
		uint8_t output_type = output_ptr->output_type;
		uint8_t decimation_factor = output_ptr->decimation_factor;
				
		ret = chdrv_read_byte(dev_ptr, CH101_FLOOR_REG_DECIMATION, &reg_value);
		reg_value &= ~(CH101_FLOOR_CORDIC_MASK | CH101_FLOOR_DECIMATION_MASK);

		/* cordic */
		if (output_type == CH_OUTPUT_AMP)
			reg_value |= CH101_FLOOR_CORDIC_EN;
		
		/* decimation factor */
		if (CH_DECIMATION_NONE == decimation_factor)
			reg_value |= CH101_FLOOR_DECIMATION_NONE;
		else if (CH_DECIMATION_2 == decimation_factor)
			reg_value |= CH101_FLOOR_DECIMATION_2;
		else if (CH_DECIMATION_3 == decimation_factor)
			reg_value |= CH101_FLOOR_DECIMATION_3;
		else
			reg_value |= CH101_FLOOR_DECIMATION_4;
		
		// Configure Decimation
		if (ret == RET_OK)
			ret |= chdrv_write_byte(dev_ptr, CH101_FLOOR_REG_DECIMATION, reg_value);		
	}
	
	return ret;
}

uint8_t  ch101_floor_get_amplitude_data(ch_dev_t *dev_ptr, uint16_t *buf_ptr, uint16_t start_sample, uint16_t num_samples,
ch_io_mode_t mode) {
	uint8_t reg_value = 0;
	uint16_t   iq_data_addr;
	ch_group_t *grp_ptr = dev_ptr->group;
	int        error = 1;
	uint8_t	   use_prog_read = 0;		// default = do not use low-level programming interface
	uint8_t	   sample_size_in_bytes = sizeof(uint16_t);
		
	error = chdrv_read_byte(dev_ptr, CH101_FLOOR_REG_DECIMATION, &reg_value);	

	if (!error && (reg_value & CH101_FLOOR_CORDIC_EN)) {	
		
		#ifndef USE_STD_I2C_FOR_IQ
		if (grp_ptr->num_connected[dev_ptr->bus_index] == 1) {		// if only one device on this bus
			use_prog_read = 1;											//   use low-level interface
		}
		#endif

		iq_data_addr = CH101_FLOOR_REG_DATA;

		iq_data_addr += (start_sample * sample_size_in_bytes);

		if ((num_samples != 0) && ((start_sample + num_samples) <= dev_ptr->max_samples)) {
			uint16_t num_bytes = (num_samples * sample_size_in_bytes);

			if (mode == CH_IO_MODE_BLOCK) {
				/* blocking transfer */

				if (use_prog_read) {
					/* use low-level programming interface for speed */

					int num_transfers = (num_bytes + (CH_PROG_XFER_SIZE - 1)) / CH_PROG_XFER_SIZE;
					int bytes_left = num_bytes;       // remaining bytes to read

					/* Convert register offsets to full memory addresses */
					if (dev_ptr->part_number == CH101_PART_NUMBER) {
						iq_data_addr += CH101_DATA_MEM_ADDR + CH101_COMMON_I2CREGS_OFFSET;
						} else {
						iq_data_addr += CH201_DATA_MEM_ADDR + CH201_COMMON_I2CREGS_OFFSET;
					}

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
						error = chdrv_prog_i2c_write(dev_ptr, message, sizeof(message));
						error |= chdrv_prog_i2c_read(dev_ptr, ((uint8_t *)buf_ptr + (xfer * CH_PROG_XFER_SIZE)), bytes_to_read);

						bytes_left -= bytes_to_read;
					}
					chbsp_program_disable(dev_ptr);					// de-assert PROG pin

					} else {	/* if (use_prog_read) */
					/* use standard I2C interface */

					error = chdrv_burst_read(dev_ptr, iq_data_addr, (uint8_t *) buf_ptr, num_bytes);
				}

				} else {
				/* non-blocking transfer - queue a read transaction (must be started using ch_io_start_nb() ) */

				if (use_prog_read && (grp_ptr->i2c_drv_flags & I2C_DRV_FLAG_USE_PROG_NB)) {
					/* Use low-level programming interface to read data */

					/* Convert register offsets to full memory addresses */
					if (dev_ptr->part_number == CH101_PART_NUMBER) {
						iq_data_addr += (CH101_DATA_MEM_ADDR + CH101_COMMON_I2CREGS_OFFSET);
						} else {
						iq_data_addr += (CH201_DATA_MEM_ADDR + CH201_COMMON_I2CREGS_OFFSET);
					}

					error = chdrv_group_queue(grp_ptr, dev_ptr, 1, CHDRV_NB_TRANS_TYPE_PROG, iq_data_addr, num_bytes,
					(uint8_t *) buf_ptr);
					} else {
					/* Use regular I2C register interface to read data */
					error = chdrv_group_queue(grp_ptr, dev_ptr, 1, CHDRV_NB_TRANS_TYPE_STD, iq_data_addr, num_bytes,
					(uint8_t*) buf_ptr);
				}
			}
		}

		return error;	
	}
	return RET_ERR;
}

uint8_t	ch101_floor_set_tx_length(ch_dev_t *dev_ptr, uint16_t num_cycles) {
	uint8_t ret_val = 0;
	
	/* Warning: This function should only be called directly after data is ready. 
	   If the ASIC mode is "Free running" then there is a potential race condition 
	   where the sensor is already performing a measurement when this API is called. 
	*/
	if (dev_ptr->part_number == CH101_PART_NUMBER) {
		ch_mode_t  saved_mode = ch_get_mode(dev_ptr);
		
		ch_set_mode(dev_ptr, CH_MODE_IDLE);
		ret_val = chdrv_write_byte(dev_ptr, CH101_FLOOR_REG_TX_LENGTH, num_cycles);	
		ret_val |= chdrv_write_byte(dev_ptr, CH101_FLOOR_REG_DCO_PERIOD, CH101_DCO_LOW);
		ret_val |= chdrv_write_word(dev_ptr, CH101_FLOOR_REG_REV_CYCLES, CH101_REVDRIVE_DEFAULT);			
		ret_val |= chdrv_write_byte(dev_ptr, CH101_FLOOR_REG_OPMODE, CH101_FLOOR_MODE_TUNING);		
		ret_val |= chdrv_wait_for_lock(dev_ptr, CHDRV_FREQLOCK_TIMEOUT_MS);
		ch_set_mode(dev_ptr, saved_mode);
	}
	return ret_val;
}

uint16_t	ch101_floor_get_tx_length(ch_dev_t *dev_ptr) {
	uint8_t num_cycles_8 = 0;
	if (dev_ptr->part_number == CH101_PART_NUMBER) {
		chdrv_read_byte(dev_ptr, CH101_FLOOR_REG_TX_LENGTH, &num_cycles_8);
	}

	return (uint16_t)num_cycles_8;
}

void ch101_floor_trigger_soft(ch_dev_t *dev_ptr) {

	/* Write special value to mode register to trigger meas */
	chdrv_write_byte(dev_ptr, CH101_FLOOR_REG_OPMODE, CH101_FLOOR_MODE_SOFT_TRIG);		
}

ch_meas_status_t ch101_floor_meas_get_status(ch_dev_t *dev_ptr, uint8_t meas_num) {
	uint8_t err = 0;
	uint8_t op_mode_value;
	ch_meas_status_t meas_status = CH_MEAS_STATUS_UNKNOWN;	// return value
	(void) meas_num;			// not used - only one measurement in ch101_floor

	/* Read status value from mode register */
	err = chdrv_read_byte(dev_ptr, CH101_FLOOR_REG_OPMODE, &op_mode_value);		

	if (!err) {
		switch (op_mode_value) {
			case CH101_FLOOR_MODE_DONE:
				meas_status = CH_MEAS_STATUS_DONE;
				break;

			case CH101_FLOOR_MODE_SOFT_TRIG:
			case CH101_FLOOR_MODE_PENDING:
				meas_status = CH_MEAS_STATUS_PENDING;
				break;

			default:
				break;
		}
	}
	
	return meas_status;
}



