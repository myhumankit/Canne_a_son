/*! \file icu_presence.c
 *
 * \brief Chirp ICU Presence Detection firmware interface
 * 
 * This file contains function definitions to interface a specific sensor firmware 
 * package to SonicLib, including the main initialization routine for the firmware.  
 * That routine initializes various fields within the \a ch_dev_t device descriptor 
 * and specifies the proper functions to implement SonicLib API calls.  Those may 
 * either be common implementations or firmware-specific routines located in this file.
 */

/*
 Copyright (c) 2019-2022, Chirp Microsystems.  All rights reserved.

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
#include "icu_presence.h"
#include "ch_common.h"


uint8_t icu_presence_init(ch_dev_t *dev_ptr, ch_group_t *grp_ptr, uint8_t i2c_addr, uint8_t io_index, uint8_t bus_index) {
	
	dev_ptr->io_index = io_index;

	(void)grp_ptr;
	(void)i2c_addr;
	(void)bus_index;

	dev_ptr->restart_only		= 1;	/* This firmware can only restart - clock cal etc. must be done previously */

	dev_ptr->freqCounterCycles  = ICU_COMMON_FREQCOUNTERCYCLES;
	dev_ptr->freqLockValue      = ICU_COMMON_READY_FREQ_LOCKED;
	dev_ptr->max_num_thresholds = ICU_COMMON_NUM_THRESHOLDS;

	/* Init firmware-specific function pointers */
	dev_ptr->fw_text 					= icu_presence_fw_text;
	dev_ptr->fw_text_size 				= icu_presence_text_size;
	dev_ptr->fw_vec 					= icu_presence_fw_vec;
	dev_ptr->fw_vec_size 				= icu_presence_vec_size;
	dev_ptr->fw_version_string			= icu_presence_version;
	dev_ptr->ram_init 					= get_ram_icu_presence_init_ptr();
	dev_ptr->get_fw_ram_init_size 		= get_icu_presence_fw_ram_init_size;
	dev_ptr->get_fw_ram_init_addr 		= get_icu_presence_fw_ram_init_addr;

	dev_ptr->prepare_pulse_timer 		= ch_common_prepare_pulse_timer;
	dev_ptr->store_pt_result 			= ch_common_store_pt_result;
	dev_ptr->store_op_freq 				= ch_common_store_op_freq;
	dev_ptr->store_bandwidth 			= ch_common_store_bandwidth;
	dev_ptr->store_scalefactor 			= ch_common_store_scale_factor;
	dev_ptr->get_locked_state 			= ch_common_get_locked_state;

	/* Init API function pointers */
	dev_ptr->api_funcs.fw_load          	= ch_common_fw_load;
	dev_ptr->api_funcs.set_mode         	= ch_common_set_mode;
	dev_ptr->api_funcs.set_sample_interval  = ch_common_set_sample_interval;
	dev_ptr->api_funcs.set_num_samples  	= ch_common_set_num_samples;
	dev_ptr->api_funcs.set_max_range    	= ch_common_set_max_range;
	dev_ptr->api_funcs.set_static_range 	= NULL;
	dev_ptr->api_funcs.set_rx_holdoff   	= NULL;
	dev_ptr->api_funcs.get_rx_holdoff   	= NULL;
	dev_ptr->api_funcs.get_range        	= NULL;
	dev_ptr->api_funcs.get_amplitude    	= NULL;
	dev_ptr->api_funcs.get_iq_data      	= ch_common_get_iq_data;
	dev_ptr->api_funcs.get_amplitude_data  	= NULL;
	dev_ptr->api_funcs.get_tof_us       	= NULL;
	dev_ptr->api_funcs.samples_to_mm    	= ch_common_samples_to_mm;
	dev_ptr->api_funcs.mm_to_samples    	= ch_common_mm_to_samples;
	dev_ptr->api_funcs.get_thresholds   	= NULL;
	dev_ptr->api_funcs.set_thresholds   	= NULL;
	dev_ptr->api_funcs.set_data_output  	= NULL;
	dev_ptr->api_funcs.set_target_interrupt = (ch_set_target_interrupt_func_t)ch_common_set_target_interrupt;
	dev_ptr->api_funcs.get_target_interrupt = (ch_get_target_interrupt_func_t)ch_common_get_target_interrupt;
	dev_ptr->api_funcs.set_sample_window 	= NULL;
	dev_ptr->api_funcs.get_amplitude_avg 	= NULL;
	dev_ptr->api_funcs.set_tx_length		= ch_common_set_tx_length;
	dev_ptr->api_funcs.get_tx_length		= ch_common_get_tx_length;

	/* Init max sample count */
	dev_ptr->max_samples = ICU_PRESENCE_MAX_SAMPLES;

	/* This firmware does not use oversampling */
	dev_ptr->oversample = 0;

	return 0;
}

