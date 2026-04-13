/*!
 * \file ch101_finaltest.c
 *
 * \brief Chirp CH101 Finaltest firmware interface
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
#include "ch101_finaltest.h"

#include "ch_common.h"
#include "ch_math_utils.h"

uint8_t ch101_finaltest_init(ch_dev_t *dev_ptr, ch_group_t *grp_ptr, uint8_t i2c_addr, uint8_t io_index, uint8_t bus_index) {
	
	(void)grp_ptr;

	dev_ptr->part_number = CH101_PART_NUMBER;
	dev_ptr->app_i2c_address = i2c_addr;
	dev_ptr->io_index = io_index;
	dev_ptr->bus_index = bus_index;

	dev_ptr->freqCounterCycles = CH101_COMMON_FREQCOUNTERCYCLES;
	dev_ptr->freqLockValue     = CH101_COMMON_READY_FREQ_LOCKED;

	/* Init firmware-specific function pointers */
	dev_ptr->fw_text 					= ch101_finaltest_fw_text;
	dev_ptr->fw_text_size 				= ch101_finaltest_text_size;
	dev_ptr->fw_vec 					= ch101_finaltest_fw_vec;
	dev_ptr->fw_vec_size 				= ch101_finaltest_vec_size;
	dev_ptr->fw_version_string			= ch101_finaltest_version;
	dev_ptr->ram_init 					= get_ram_ch101_finaltest_init_ptr();
	dev_ptr->get_fw_ram_init_size 		= get_ch101_finaltest_fw_ram_init_size;
	dev_ptr->get_fw_ram_init_addr 		= get_ch101_finaltest_fw_ram_init_addr;

	dev_ptr->prepare_pulse_timer 		= ch_common_prepare_pulse_timer;
	dev_ptr->store_pt_result 			= ch101_finaltest_store_pt_result;
	dev_ptr->store_op_freq 				= ch101_finaltest_store_op_freq;
	dev_ptr->store_bandwidth 			= ch_common_store_bandwidth;
	dev_ptr->store_scalefactor 			= ch101_finaltest_store_scale_factor;
	dev_ptr->get_locked_state 			= ch101_finaltest_get_locked_state;

	/* Init API function pointers */
	dev_ptr->api_funcs.fw_load          = ch_common_fw_load;
	dev_ptr->api_funcs.set_mode         = ch_common_set_mode;
	dev_ptr->api_funcs.set_sample_interval  = ch_common_set_sample_interval;
	dev_ptr->api_funcs.set_num_samples  = ch_common_set_num_samples;
	dev_ptr->api_funcs.set_max_range    = ch_common_set_max_range;
	dev_ptr->api_funcs.set_static_range = NULL;
	dev_ptr->api_funcs.set_rx_holdoff   	= ch_common_set_rx_holdoff;
	dev_ptr->api_funcs.get_rx_holdoff   	= ch_common_get_rx_holdoff;
	dev_ptr->api_funcs.get_range        = (ch_get_range_func_t) ch101_finaltest_get_range;
	dev_ptr->api_funcs.get_amplitude    = ch101_finaltest_get_amplitude;
	dev_ptr->api_funcs.get_iq_data      = (ch_get_iq_data_func_t) ch101_finaltest_get_iq_data;
	dev_ptr->api_funcs.samples_to_mm    = ch_common_samples_to_mm;
	dev_ptr->api_funcs.mm_to_samples    = ch_common_mm_to_samples;
	dev_ptr->api_funcs.set_thresholds   = NULL;								// not supported
	dev_ptr->api_funcs.get_thresholds   = NULL;								// not supported

	/* Init max sample count */
	dev_ptr->max_samples = CH101_FINALTEST_MAX_SAMPLES;

	/* This firmware does not use oversampling */
	dev_ptr->oversample = 0;

	return 0;
}


uint8_t ch101_finaltest_set_rxqueue_item( ch_dev_t* dev_ptr, uint8_t queue_index,
		uint8_t samples, uint8_t attenuation, uint8_t gain )
{
	uint8_t ret = ! dev_ptr ||
			queue_index >=      CH101_FINALTEST_RXQUEUE_ITEMS ||
			samples     >= 1 << CH101_FINALTEST_RXQUEUE_BITS_SAMPLES ||
			attenuation >= 1 << CH101_FINALTEST_RXQUEUE_BITS_ATTEN ||
			gain        >= 1 << CH101_FINALTEST_RXQUEUE_BITS_GAIN;

	if( ! ret )
	{
		uint16_t item =
				(uint16_t)samples     << CH101_FINALTEST_RXQUEUE_BITPOS_SAMPLES |
				(uint16_t)attenuation << CH101_FINALTEST_RXQUEUE_BITPOS_ATTEN   |
				(uint16_t)gain        << CH101_FINALTEST_RXQUEUE_BITPOS_GAIN;

		ret = chdrv_write_word( dev_ptr, CH101_FINALTEST_REG_RXQUEUE + queue_index * sizeof(item), item );
	}

	return ret;
}


uint8_t ch101_finaltest_set_threshold(ch_dev_t *dev_ptr, uint16_t threshold) {
	uint8_t ret = 1;
	if (dev_ptr->sensor_connected) {
		ret = chdrv_write_word(dev_ptr, CH101_FINALTEST_REG_THRESHOLD, threshold);
	}
	return ret;
}

uint32_t ch101_finaltest_get_range(ch_dev_t *dev_ptr, ch101_finaltest_range_t range_type) {
	uint8_t		tof_reg;
	uint32_t	range = CH_NO_TARGET;
	uint16_t 	time_of_flight;
	uint16_t 	tof_scale_factor;
	int 		err;

	if (dev_ptr->sensor_connected) {

		tof_reg = CH101_FINALTEST_REG_TOF;

		err = chdrv_read_word(dev_ptr, tof_reg, &time_of_flight);

		if (!err && (time_of_flight != UINT16_MAX)) { // If object detected

			chdrv_read_word(dev_ptr, CH101_FINALTEST_REG_TOF_SF, &tof_scale_factor);
			
			if (tof_scale_factor != 0) {
				uint32_t num = (CH_SPEEDOFSOUND_MPS * dev_ptr->group->rtc_cal_pulse_ms * (uint32_t) time_of_flight);
				uint32_t den = ((uint32_t) dev_ptr->rtc_cal_result * (uint32_t) tof_scale_factor) >> 11;		// XXX need define

				range = (num / den);

				if (range_type == CH101_FINALTEST_RANGE_ECHO_ONE_WAY) {
					range /= 2;
				}

				/* Adjust for oversampling, if used */
				range >>= dev_ptr->oversample;

			}
		}
	}
	return range;
}



uint16_t ch101_finaltest_get_amplitude(ch_dev_t *dev_ptr) {
	uint16_t intensity = 0xFFFF;
	chdrv_read_word(dev_ptr, CH101_FINALTEST_REG_AMPLITUDE, &intensity);
	return intensity;
}


uint8_t  ch101_finaltest_get_iq_data(ch_dev_t *dev_ptr, uint8_t /*ch_iq_sample_t*/ *buf_ptr, uint16_t start_sample, 
									 uint16_t num_samples, uint8_t /*ch_io_mode_t*/ mode) {

	uint16_t iq_data_addr = CH101_FINALTEST_REG_DATA;
	uint16_t num_bytes = (num_samples * sizeof(ch_iq_sample_t));
	uint8_t  error = 0;

	iq_data_addr += (start_sample * sizeof(ch_iq_sample_t));

	if (iq_data_addr > UINT8_MAX) {
		error = 1;
	}
	
	if (mode != CH_IO_MODE_BLOCK) {
		error = 1;
	}

	if (!error) {
		error = chdrv_burst_read(dev_ptr, iq_data_addr, (uint8_t *) buf_ptr, num_bytes);
	}

	return error;
}



uint8_t ch101_finaltest_get_locked_state(ch_dev_t *dev_ptr) {
	uint8_t locked = CH101_FINALTEST_READY_NOTLOCKED;
	if (dev_ptr->sensor_connected) {
		chdrv_read_byte(dev_ptr, CH101_FINALTEST_REG_READY, &locked);
	}
	return (locked == CH101_FINALTEST_READY_FREQ_LOCKED_BM);
}


void ch101_finaltest_store_pt_result(ch_dev_t *dev_ptr) {
	chdrv_read_word(dev_ptr, CH101_FINALTEST_REG_CAL_RESULT, &(dev_ptr->rtc_cal_result));
}

void ch101_finaltest_store_op_freq(ch_dev_t *dev_ptr){
	dev_ptr->op_frequency = ch101_finaltest_get_op_freq(dev_ptr);
}


void ch101_finaltest_store_scale_factor(ch_dev_t *dev_ptr){
	ch_iq_sample_t QIData;

	chdrv_burst_read(dev_ptr, CH101_FINALTEST_REG_DATA + (CH101_SCALEFACTOR_INDEX * 4),(uint8_t *) &QIData, 4);
	dev_ptr->scale_factor = ch_iq_to_amplitude(&QIData);
}


int ch101_finaltest_set_pulse_width(ch_dev_t *dev_ptr,uint8_t pulse_width){
	return chdrv_write_byte(dev_ptr, CH101_FINALTEST_REG_PULSE_WIDTH, pulse_width);
}

int ch101_finaltest_set_tx_length(ch_dev_t *dev_ptr, uint8_t tx_length){
    return chdrv_write_byte(dev_ptr, CH101_FINALTEST_REG_TXLENGTH, tx_length);
}


uint32_t ch101_finaltest_get_op_freq(ch_dev_t *dev_ptr){
	uint32_t num,den,opfreq;
	uint16_t raw_freq;		// aka scale factor
	uint32_t freq_counter_cycles = dev_ptr->freqCounterCycles;

	chdrv_read_word(dev_ptr, CH101_FINALTEST_REG_TOF_SF, &raw_freq);
	num = (uint32_t)(((dev_ptr->rtc_cal_result)*1000U) / (16U * freq_counter_cycles))*(uint32_t)(raw_freq);
	den = dev_ptr->group->rtc_cal_pulse_ms;
	opfreq = (num/den);
	return opfreq;
}

