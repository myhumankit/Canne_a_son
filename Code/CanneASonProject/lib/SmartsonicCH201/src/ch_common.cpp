/*! \file ch_common.c
 *
 * \brief Chirp SonicLib API function common implementations
 * 
 * This file contains standard implementations of functions required to support the
 * SonicLib API.  The sensor firmware, in it's init routine, specifies which of these
 * common implementations should be used by initializing a set of function pointers.
 * These pointers, contained in the ch_api_funcs_t structure within the device descriptor,
 * can either direct the API calls to the functions in this file or to firmware-specific
 * equivalents that are supplied as part of the sensor firmware release.
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
#include "ch_common.h"
#include "chirp_bsp.h"
#include "ch_math_utils.h"

#ifdef INCLUDE_SHASTA_SUPPORT
#include "shasta_pmut_cmds.h"
#endif


/* Local definitions */
#define CH_IQ_SAMPLES_PER_READ		64		// number of I/Q samples to read at a time
#define REF_PRE_RX_CYCLES			800		// reference number of pre-receive cycles, to adjust ToF


#ifdef INCLUDE_WHITNEY_SUPPORT
// Check that CH101 and CH201 prog mem are the same (assumed by ch_common_check_program())
#if ((CH101_PROG_MEM_ADDR != CH201_PROG_MEM_ADDR) || (CH101_PROG_MEM_SIZE != CH201_PROG_MEM_SIZE))
#error Mismatch in CH101 and CH201 program address or size
#endif
#endif

/* Forward references */
#ifdef INCLUDE_SHASTA_SUPPORT
#ifdef INCLUDE_ALGO_RANGEFINDER
static void thresh_lib_to_sensor(ch_dev_t *dev_ptr, ch_thresholds_t *lib_thresh_ptr, thresholds_t *sens_thresh_ptr);
static void thresh_sensor_to_lib(ch_dev_t *dev_ptr, thresholds_t *sens_thresh_ptr, ch_thresholds_t *lib_thresh_ptr);
static uint8_t read_target_list(ch_dev_t *dev_ptr);
#endif // INCLUDE_ALGO_RANGEFINDER
#endif // INCLUDE_SHASTA_SUPPORT

static uint8_t get_sample_data(ch_dev_t *dev_ptr, ch_iq_sample_t *buf_ptr, uint16_t start_sample, uint16_t num_samples,
							   ch_io_mode_t mode, uint8_t sample_size_in_byte);

/* Local variables */
#ifdef INCLUDE_SHASTA_SUPPORT
#ifdef INCLUDE_ALGO_RANGEFINDER
static ch_thresholds_t default_thresholds = {		// XXX Shasta-specific values - define in header or ??
	.threshold = {
		{0, 	800},		/* threshold 0 */
		{20,	400},		/* threshold 1 */
		{37,	200},		/* threshold 2 */
		{50,	100},		/* threshold 3 */
		{80,	70},		/* threshold 4 */
		{120,	50},		/* threshold 5 */
		{0,		0},			/* threshold 6 */
		{0,		0},			/* threshold 7 */
	}
};
#endif // INCLUDE_ALGO_RANGEFINDER
#endif // INCLUDE_SHASTA_SUPPORT


/* Functions */

uint8_t	ch_common_init(ch_dev_t *dev_ptr, ch_group_t *grp_ptr, uint8_t dev_num, ch_fw_init_func_t fw_init_func) {
	
	uint8_t	ret_val = RET_ERR;

	ch_i2c_info_t	i2c_info;

#ifdef INCLUDE_WHITNEY_SUPPORT
	/* Get I2C parameters from BSP */
	ret_val = chbsp_i2c_get_info(grp_ptr, dev_num, &i2c_info);
#else
	i2c_info.address = 0;
	i2c_info.bus_num = 0;
	i2c_info.drv_flags = 0;
	ret_val = RET_OK;		// XXX
#endif
	
	if (ret_val == RET_OK) {
		/* Save special handling flags for Chirp driver */
		grp_ptr->i2c_drv_flags = i2c_info.drv_flags;

		/* Call asic f/w init function passed in as parameter */	// XXX TODO change interface
		ret_val = (*fw_init_func)(dev_ptr, grp_ptr, i2c_info.address, dev_num, i2c_info.bus_num);
	}

	if (ret_val == RET_OK) {
		/* Init device and group descriptor linkage */
		dev_ptr->group = grp_ptr;					// set parent group pointer
		grp_ptr->device[dev_num] = dev_ptr;			// add device to parent group
		dev_ptr->fw_init_func = fw_init_func;		// store which f/w init was used

		/* Mark this group as pending initialization via ch_group_start() */
		grp_ptr->status = CH_GROUP_STAT_INIT_PENDING;
	}

#ifdef INCLUDE_SHASTA_SUPPORT
	if (ret_val == RET_OK) {
#ifdef CH_STD_INIT_FUNC	
		/* Check if f/w requires separate initialization */
		if (dev_ptr->restart_only && !(dev_ptr->restart_pending)) {
			dev_ptr->fw_reinit_func = fw_init_func;	// save original init routine to call again later

			/* Set up to use separate init f/w */
			ret_val = CH_STD_INIT_FUNC(dev_ptr, grp_ptr, 0, dev_num, 0);
			if (ret_val == RET_OK) {
				dev_ptr->restart_pending = 1;
			}
		}
#endif
	}
#endif  // INCLUDE_SHASTA_SUPPORT

	return ret_val;
}

uint8_t	ch_common_group_start(ch_group_t *grp_ptr) {
	uint8_t err = 0;

	err =  chdrv_group_start(grp_ptr, 0);

#ifdef INCLUDE_SHASTA_SUPPORT
	for (int dev_num = 0; dev_num < grp_ptr->num_ports; dev_num++) {
		ch_dev_t *dev_ptr = grp_ptr->device[dev_num];

		/* Check if restart is pending */
		if (dev_ptr->sensor_connected  && dev_ptr->restart_pending) {
			/* Call saved init function (again) for f/w that will execute */
			err |= ch_init(dev_ptr, grp_ptr, dev_num, dev_ptr->fw_reinit_func);

			/* Restart */
			if (!err) {
				err =  chdrv_restart(dev_ptr);
			}
			dev_ptr->restart_pending = 0;
		}
	}
#endif

	if (!err) {
		grp_ptr->status = CH_GROUP_STAT_INIT_OK;
	}
	
	return err;
}

uint8_t ch_common_set_mode(ch_dev_t *dev_ptr, ch_mode_t mode) {
	uint8_t ret_val = 0;

#ifdef INCLUDE_SHASTA_SUPPORT
	measurement_queue_t *meas_q_ptr = &(dev_ptr->meas_queue);

	if (dev_ptr->sensor_connected) {
		switch (mode) {
			case CH_MODE_IDLE:
				// XXX TODO more
				// disable trigger sources
				meas_q_ptr->trigsrc &= ~(TRIGSRC_HWTRIGGER_INT1 | TRIGSRC_HWTRIGGER_INT2 | TRIGSRC_RTC);
				break;

			case CH_MODE_FREERUN:
				ch_set_sample_interval(dev_ptr, dev_ptr->sample_interval);

				meas_q_ptr->trigsrc = TRIGSRC_RTC;			// enable timer-based trigger
				break;

			case CH_MODE_TRIGGERED_RX_ONLY:
				// XXX TODO more for Rx-only, then fall through to tx/rx
			case CH_MODE_TRIGGERED_TX_RX:
				if (dev_ptr->group->sensor_trig_pin == 1) {
					meas_q_ptr->trigsrc = TRIGSRC_HWTRIGGER_INT1;
				} else if (dev_ptr->group->sensor_trig_pin == 2) {
					meas_q_ptr->trigsrc = TRIGSRC_HWTRIGGER_INT2;
				}
				break;
			default:
				ret_val = RET_ERR;				// return non-zero to indicate error
				break;
		}

		if (ret_val == RET_OK) {
			ret_val = ch_meas_write_config(dev_ptr);
		}
		if (ret_val == RET_OK) {
			chdrv_event_trigger(dev_ptr, EVENT_CONFIG_TRIGGER);
		}
	}

#elif defined(INCLUDE_WHITNEY_SUPPORT)
	uint8_t	opmode_reg;
	uint8_t	period_reg;
	uint8_t	tick_interval_reg;
	uint16_t max_tick_interval;

	if (dev_ptr->part_number == CH101_PART_NUMBER) {
		opmode_reg = CH101_COMMON_REG_OPMODE;
		period_reg = CH101_COMMON_REG_PERIOD;
		tick_interval_reg = CH101_COMMON_REG_TICK_INTERVAL;
		max_tick_interval = CH101_MAX_TICK_INTERVAL;
	} else if (dev_ptr->part_number == CH201_PART_NUMBER) {
		opmode_reg = CH201_COMMON_REG_OPMODE;
		period_reg = CH201_COMMON_REG_PERIOD;
		tick_interval_reg = CH201_COMMON_REG_TICK_INTERVAL;
		max_tick_interval = CH201_MAX_TICK_INTERVAL;
	}

	if (dev_ptr->sensor_connected) {
		switch (mode) {
			case CH_MODE_IDLE:
				chdrv_write_byte(dev_ptr, period_reg, 0);
				chdrv_write_word(dev_ptr, tick_interval_reg, max_tick_interval);
				chdrv_write_byte(dev_ptr, opmode_reg, CH_MODE_IDLE);
				break;

			case CH_MODE_FREERUN:
				ch_set_sample_interval(dev_ptr, dev_ptr->sample_interval);
				chdrv_write_byte(dev_ptr, opmode_reg, CH_MODE_FREERUN);
				break;

			case CH_MODE_TRIGGERED_TX_RX:
				chdrv_write_byte(dev_ptr, opmode_reg, CH_MODE_TRIGGERED_TX_RX);
				break;

			case CH_MODE_TRIGGERED_RX_ONLY:
				chdrv_write_byte(dev_ptr, opmode_reg, CH_MODE_TRIGGERED_RX_ONLY);
				break;

			default:
				ret_val = RET_ERR;				// return non-zero to indicate error
				break;
		}
	}

#endif	// INCLUDE_SHASTA_SUPPORT

	return ret_val;
}

uint8_t ch_common_fw_load(ch_dev_t *dev_ptr) {
	uint8_t	 ch_err = 0;
	uint16_t prog_mem_addr = 0;		// current destination address in sensor mem
	uint8_t *src_addr = (uint8_t *) dev_ptr->fw_text;	// firmware image text to load
	uint16_t fw_size = 0;
	uint16_t num_xfers;
	uint16_t xfer_num = 0;
	uint16_t bytes_left = 0;
	uint16_t max_xfer_size = 0;		// max number of bytes per write transfer to sensor

	fw_size = dev_ptr->fw_text_size;

	if (dev_ptr->asic_gen == CH_ASIC_GEN_1_WHITNEY) {
		if (fw_size == 0) {			// handle missing size (old init)
			fw_size = CHX01_FW_SIZE;
		}
		prog_mem_addr = CHX01_PROG_MEM_ADDR;
	} else if (dev_ptr->asic_gen == CH_ASIC_GEN_2_SHASTA) {
		prog_mem_addr = ICU_PROG_MEM_ADDR;
	}

	max_xfer_size = fw_size;
#ifdef MAX_PROG_XFER_SIZE			// if user specified max xfer size in chirp_board_config.h
	if (fw_size > MAX_PROG_XFER_SIZE) {
		max_xfer_size = MAX_PROG_XFER_SIZE;
	}
#endif

	/* Write f/w to sensor */
	num_xfers = (fw_size + (max_xfer_size - 1)) / max_xfer_size;	// total # of transfers req'd
	bytes_left = fw_size;

	while (!ch_err && (xfer_num < num_xfers)) {
		uint16_t xfer_nbytes;			// number of bytes in this transfer

		if (bytes_left >= max_xfer_size) {
			xfer_nbytes = max_xfer_size;
		} else {
			xfer_nbytes = bytes_left;
		}

		if (dev_ptr->asic_gen == CH_ASIC_GEN_1_WHITNEY) {
			ch_err = chdrv_prog_mem_write(dev_ptr, prog_mem_addr, src_addr, xfer_nbytes);

		} else if (dev_ptr->asic_gen == CH_ASIC_GEN_2_SHASTA) {
			ch_err = chdrv_burst_write(dev_ptr, prog_mem_addr, src_addr, xfer_nbytes);
		}

		src_addr += xfer_nbytes;		// adjust source and destination addrs
		prog_mem_addr += xfer_nbytes;

		bytes_left -= xfer_nbytes;		// adjust remaining byte count
		xfer_num++;
	}

	/* Write separate interrupt vectors (last bytes in prog mem) */
	if ((dev_ptr->fw_vec != NULL) && (dev_ptr->fw_vec_size != 0)) {
		src_addr = (uint8_t *) dev_ptr->fw_vec;			// firmware image vectors to load
		uint16_t xfer_nbytes = dev_ptr->fw_vec_size;	// fit in single transfer

		if (dev_ptr->asic_gen == CH_ASIC_GEN_1_WHITNEY) {
			prog_mem_addr = CHX01_PROG_MEM_ADDR + CHX01_PROG_MEM_SIZE - dev_ptr->fw_vec_size;
			ch_err = chdrv_prog_mem_write(dev_ptr, prog_mem_addr, src_addr, xfer_nbytes);

		} else if (dev_ptr->asic_gen == CH_ASIC_GEN_2_SHASTA) {
			prog_mem_addr = ICU_PROG_MEM_ADDR + ICU_PROG_MEM_SIZE - dev_ptr->fw_vec_size;
			ch_err = chdrv_burst_write(dev_ptr, prog_mem_addr, src_addr, xfer_nbytes);
		}
	}
	return ch_err;
}


uint8_t ch_common_set_sample_interval(ch_dev_t *dev_ptr, uint16_t interval_ms) {
	uint8_t ret_val = 0;

	if (dev_ptr->sensor_connected) {
		uint32_t meas_interval = dev_ptr->rtc_cal_result * interval_ms / dev_ptr->group->rtc_cal_pulse_ms;

#ifdef INCLUDE_SHASTA_SUPPORT
		measurement_t *meas_ptr = (measurement_t *) &(dev_ptr->meas_queue.meas[CH_DEFAULT_MEAS_NUM]);

		meas_ptr->meas_period = meas_interval;
		ret_val = ch_meas_write_config(dev_ptr);

		if (ret_val == 0) {
			dev_ptr->sample_interval = interval_ms;
		}

#elif defined(INCLUDE_WHITNEY_SUPPORT)
		uint32_t period;
		uint8_t	period_reg;
		uint8_t	tick_interval_reg;
		uint16_t max_tick_interval;

		if (dev_ptr->part_number == CH101_PART_NUMBER) {
			period_reg 		  = CH101_COMMON_REG_PERIOD;
			tick_interval_reg = CH101_COMMON_REG_TICK_INTERVAL;
			max_tick_interval = CH101_MAX_TICK_INTERVAL;
		} else {
			period_reg 		  = CH201_COMMON_REG_PERIOD;
			tick_interval_reg = CH201_COMMON_REG_TICK_INTERVAL;
			max_tick_interval = CH201_MAX_TICK_INTERVAL;
		}

		if (interval_ms != 0) {
			period = (meas_interval / 2048) + 1;				// XXX need define
			if (period > UINT8_MAX) {					/* check if result fits in register */
				ret_val = 1;
			}
		} else {
			ret_val = 1;						// interval cannot be zero
		}

		if (ret_val == 0) {
			uint32_t tick_interval;

			if (period != 0) {
				tick_interval = meas_interval / period;
				
				while (tick_interval > max_tick_interval) {  // enforce max interval
					tick_interval >>= 1;
					period <<= 1;
				}
			} else {
				tick_interval = max_tick_interval;
			}

#ifdef CHDRV_DEBUG
			char cbuf[80];
			snprintf(cbuf, sizeof(cbuf), "Set period=%lu, tick_interval=%lu\n", period, tick_interval);
			chbsp_print_str(cbuf);
#endif
			chdrv_write_byte(dev_ptr, period_reg, (uint8_t) period);
			chdrv_write_word(dev_ptr, tick_interval_reg, (uint16_t) tick_interval);

			dev_ptr->sample_interval = interval_ms;
		}

#endif	// INCLUDE_SHASTA_SUPPORT
	}

	return ret_val;
}

// Note: uses actual num_samples, even for CH201
uint8_t ch_common_set_num_samples(ch_dev_t *dev_ptr, uint16_t num_samples ) {
	uint8_t ret_val = 1;		// default is error (not connected or num_samples too big)


#ifdef INCLUDE_SHASTA_SUPPORT
	measurement_t *meas_ptr = (measurement_t *)&(dev_ptr->meas_queue.meas[CH_DEFAULT_MEAS_NUM]);
	uint8_t	seg_num = (dev_ptr->meas_num_segments[CH_DEFAULT_MEAS_NUM] - 1);	// get index of last active segment
	ch_odr_t odr = (ch_odr_t) meas_ptr->odr;
	
	if (num_samples > dev_ptr->num_rx_samples) {
		/* New sample count is greater - extend last measurement segment */
		uint16_t added_samples = (num_samples - dev_ptr->num_rx_samples);
		uint16_t added_cycles = ch_common_samples_to_cycles(added_samples, odr);

		meas_ptr->trx_inst[seg_num].length += added_cycles;
		
		dev_ptr->num_rx_samples += added_samples;
		dev_ptr->meas_num_rx_samples[CH_DEFAULT_MEAS_NUM] += added_samples;

	} else if (num_samples < dev_ptr->num_rx_samples) {
		/* New sample count is less - shorten or eliminate last measurement segment(s) */
		uint16_t deleted_samples = (dev_ptr->num_rx_samples - num_samples);
		uint16_t deleted_cycles = ch_common_samples_to_cycles(deleted_samples, odr);
		uint16_t deleted_cycles_left = deleted_cycles;

		while (deleted_cycles_left > 0) {
			if (meas_ptr->trx_inst[seg_num].length > deleted_cycles_left) { 
				/* new end will be in this segment */
				meas_ptr->trx_inst[seg_num].length -= deleted_cycles_left;
				deleted_cycles_left = 0;
			} else { 
				/* need to delete last segment and move to previous */
				deleted_cycles_left -= meas_ptr->trx_inst[seg_num].length;

				meas_ptr->trx_inst[seg_num].cmd_config = PMUT_CMD_EOF;

				seg_num--;
				(dev_ptr->meas_num_segments[CH_DEFAULT_MEAS_NUM])--;	// one less active segment
			}
		}

		dev_ptr->meas_num_rx_samples[CH_DEFAULT_MEAS_NUM] -= deleted_samples;
	}

	dev_ptr->num_rx_samples = num_samples;

	/* Write to sensor */
	ret_val = ch_meas_write_config(dev_ptr);

#elif defined(INCLUDE_WHITNEY_SUPPORT)
	uint8_t max_range_reg;
	uint16_t num_rx_low_gain_samples = ch_get_rx_low_gain(dev_ptr);		// zero if unsupported

	if (dev_ptr->part_number == CH101_PART_NUMBER) {
		max_range_reg = CH101_COMMON_REG_MAX_RANGE;
	} else {
		max_range_reg = CH201_COMMON_REG_MAX_RANGE;
		num_samples /= 2;					// each internal count for CH201 represents 2 physical samples
		num_rx_low_gain_samples /= 2;
	}

	if (num_samples < (num_rx_low_gain_samples + 1)) {
		num_samples = (num_rx_low_gain_samples + 1);
	}

	if (dev_ptr->sensor_connected && (num_samples <= UINT8_MAX)) {
		ret_val = chdrv_write_byte(dev_ptr, max_range_reg, num_samples);
	}

	if (!ret_val) {
		if (dev_ptr->part_number == CH101_PART_NUMBER) {
			dev_ptr->num_rx_samples = num_samples;
		} else {
			dev_ptr->num_rx_samples = (num_samples * 2);	// store actual physical sample count
		}
	} else {
		dev_ptr->num_rx_samples = 0;
	}
#endif	// INCLUDE_SHASTA_SUPPORT
	
	return ret_val;
}


uint8_t ch_common_set_max_range(ch_dev_t *dev_ptr, uint16_t max_range_mm) {
	uint8_t ret_val;
	uint32_t num_samples;

	ret_val = (!dev_ptr->sensor_connected);

	if (!ret_val) {
		num_samples = dev_ptr->api_funcs.mm_to_samples(dev_ptr, max_range_mm);

		if (num_samples > dev_ptr->max_samples) {
			num_samples = dev_ptr->max_samples;
			dev_ptr->max_range = ch_samples_to_mm(dev_ptr, num_samples);	// store reduced max range
		} else {
			dev_ptr->max_range = max_range_mm;							// store user-specified max range
		}


#ifdef CHDRV_DEBUG
		char cbuf[80];
		snprintf(cbuf, sizeof(cbuf), "num_samples=%lu\n", num_samples);
		chbsp_print_str(cbuf);
#endif
	}


	if (!ret_val) {
		ret_val = ch_set_num_samples(dev_ptr, num_samples);
	}

#ifdef CHDRV_DEBUG
	printf("Set samples: ret_val: %u  dev_ptr->num_rx_samples: %u\n", ret_val, dev_ptr->num_rx_samples);
#endif
	return ret_val;
}


uint16_t ch_common_mm_to_samples(ch_dev_t *dev_ptr, uint16_t num_mm) {
	uint8_t err;
	uint32_t scale_factor;
	uint32_t num_samples = 0;
	uint32_t divisor1 = 1;
	uint32_t divisor2 = (dev_ptr->group->rtc_cal_pulse_ms * CH_SPEEDOFSOUND_MPS);
#ifdef INCLUDE_SHASTA_SUPPORT
	ch_odr_t odr = (ch_odr_t) dev_ptr->meas_queue.meas[CH_DEFAULT_MEAS_NUM].odr;
#endif

	err = (!dev_ptr) || (!dev_ptr->sensor_connected);

	if (!err) {
		if (dev_ptr->asic_gen == CH_ASIC_GEN_1_WHITNEY) {
			if (dev_ptr->part_number == CH101_PART_NUMBER) {
				divisor1 = 0x2000;			// (4*16*128)  XXX need define(s)
			} else if (dev_ptr->part_number == CH201_PART_NUMBER) {
				divisor1 = 0x4000;			// (4*16*128*2)  XXX need define(s)
			}
		} else if (dev_ptr->asic_gen == CH_ASIC_GEN_2_SHASTA) {
#ifdef INCLUDE_SHASTA_SUPPORT
            divisor1 = (16/2 * dev_ptr->fcount_cycles) << (7 - odr);
							// 1/2 range_mm_per_sound_path_mm * 16 clock_cycles_per_op_freq_cycle * fcount_cycles * 2^(7-cic_odr)
#endif // INCLUDE_SHASTA_SUPPORT
		}
#ifdef INCLUDE_SHASTA_SUPPORT
		scale_factor = (uint32_t) dev_ptr->pmut_clock_fcount;
#elif defined(INCLUDE_WHITNEY_SUPPORT)
		if (dev_ptr->scale_factor == 0) {
			dev_ptr->store_scalefactor(dev_ptr);
		}
		scale_factor = (uint32_t) dev_ptr->scale_factor;
#endif
	}

	if (!err) {
		// Two steps of division to avoid needing a type larger than 32 bits
		// Ceiling division to ensure result is at least enough samples to meet specified range
		// Oversample value is signed power of two for this firmware relative to standard f/8 sampling.

		num_samples = ((dev_ptr->rtc_cal_result * scale_factor) + (divisor1 - 1)) / divisor1;

		num_samples = (((num_samples * num_mm) << dev_ptr->oversample) + (divisor2 - 1)) / divisor2;

		err = (num_samples > UINT16_MAX);
	}

	if (!err) {
		if (dev_ptr->part_number == CH201_PART_NUMBER) {
			num_samples *= 2;			// each internal count for CH201 represents 2 physical samples
		}
	}

	if (err) {
		num_samples = 0;		// return zero if error
	}

	return (uint16_t) num_samples;
}


uint16_t ch_common_samples_to_mm(ch_dev_t *dev_ptr, uint16_t num_samples) {
	uint32_t	num_mm = 0;
	uint32_t	op_freq = dev_ptr->op_frequency;
#ifdef INCLUDE_SHASTA_SUPPORT
	ch_odr_t	odr = (ch_odr_t) dev_ptr->meas_queue.meas[CH_DEFAULT_MEAS_NUM].odr;

	if (op_freq != 0) {
		num_mm = (((uint32_t) num_samples * CH_SPEEDOFSOUND_MPS * 1000) << (7 - odr))  / (op_freq * 2);
	}

#elif defined(INCLUDE_WHITNEY_SUPPORT)
	if (op_freq != 0) {
		num_mm = ((uint32_t) num_samples * CH_SPEEDOFSOUND_MPS * 8 * 1000) / (op_freq * 2);
	}

	/* Adjust for oversampling, if used */
	num_mm >>= dev_ptr->oversample;
#endif

	return (uint16_t) num_mm;
}



uint8_t ch_common_set_static_range(ch_dev_t *dev_ptr, uint16_t num_samples) {
	uint8_t ret_val = 1;  	// default is error return

#ifdef INCLUDE_ALGO_RANGEFINDER
	if (dev_ptr->part_number == CH101_PART_NUMBER) {
		if (dev_ptr->sensor_connected) {
			ret_val = chdrv_write_byte(dev_ptr, CH101_COMMON_REG_STAT_RANGE, num_samples);

			if (!ret_val) {
				ret_val = chdrv_write_byte(dev_ptr, CH101_COMMON_REG_STAT_COEFF, 
						                   CH101_COMMON_STAT_COEFF_DEFAULT);
			}

			if (!ret_val) {
				dev_ptr->static_range = num_samples;
			}
		}
	} else if (dev_ptr->asic_gen == CH_ASIC_GEN_2_SHASTA) {
#ifdef INCLUDE_SHASTA_SUPPORT
		dev_ptr->algo_cfg.meas_cfg[CH_DEFAULT_MEAS_NUM].static_target_samples = num_samples;

		ret_val = ch_meas_write_config(dev_ptr);
#endif // INCLUDE_SHASTA_SUPPORT
	}

#else
	(void)dev_ptr;
	(void)num_samples;
#endif	// INCLUDE_ALGO_RANGEFINDER

	return ret_val;
}


uint8_t	ch_common_get_num_targets(ch_dev_t *dev_ptr) {			 // XXX TODO implement for whitney
	uint8_t		num_valid_targets = 0;

#ifdef INCLUDE_SHASTA_SUPPORT
#ifdef INCLUDE_ALGO_RANGEFINDER
	uint16_t num_targets_addr =  (uint16_t)(uintptr_t)&((dev_ptr->sens_algo_out_addr)->tL.num_valid_targets);

	chdrv_read_byte(dev_ptr, num_targets_addr, &num_valid_targets);
#else
	(void)dev_ptr;
#endif

#elif defined(INCLUDE_WHITNEY_SUPPORT)
	// XXX TODO implement for whitney
	(void)dev_ptr;
#endif // INCLUDE_SHASTA_SUPPORT

	return num_valid_targets;
}


#ifdef INCLUDE_ALGO_RANGEFINDER
uint32_t ch_common_get_target_range(ch_dev_t *dev_ptr, uint8_t target_num, ch_range_t range_type) {
	uint32_t	range = CH_NO_TARGET;
	uint32_t 	time_of_flight = 0;
	int 		err;
#ifdef INCLUDE_SHASTA_SUPPORT
	target_list_t *tgt_list_ptr = (target_list_t *) &(dev_ptr->algo_out.tL);
#elif defined(INCLUDE_WHITNEY_SUPPORT)
	uint8_t		tof_reg;
	uint16_t 	scale_factor;

	(void)target_num; /* parameter not used with whitney */
#endif

	if (dev_ptr->sensor_connected) {

#ifdef INCLUDE_SHASTA_SUPPORT
		/* Get current target list from sensor */		// XXX TODO optimize reads vs. get_amplitude etc.
		err = read_target_list(dev_ptr);

		if (tgt_list_ptr->num_valid_targets == 0) {
			/* No target detected */
			err = 1;
		}

		if (!err) {
			/* Report specified target from list */
			time_of_flight = (uint32_t) tgt_list_ptr->targets[target_num].range;

#ifdef CHDRV_DEBUG 
            uint8_t num_valid_targets = tgt_list_ptr->num_valid_targets;
            uint8_t odr_out = dev_ptr->odr_out;
            uint16_t raw_range = tgt_list_ptr->targets[0].range;
            uint16_t amplitude = tgt_list_ptr->targets[0].amplitude;
			printf("target list:  num_valid_targets=%d  odr_out=%d  range[0]=%d  amp[0]=%d\n\r",
					num_valid_targets, odr_out, raw_range, amplitude);
#endif
			/* Adjust ToF for cycles in measurement before rx begins (tx & count segments) */
			uint16_t pre_rx_cycles = dev_ptr->meas_pre_rx_cycles[dev_ptr->last_measurement];

#ifdef CHDRV_DEBUG 
			printf("orig time_of_flight = %ld  pre_rx_cycles = %u   ", time_of_flight, pre_rx_cycles);
#endif
			if (pre_rx_cycles > REF_PRE_RX_CYCLES) {						// if more than reference pre-rx cycle count
				time_of_flight += (pre_rx_cycles - REF_PRE_RX_CYCLES);		// apply adjustment for "late start"
			}

#ifdef CHDRV_DEBUG 
			printf("adjusted time_of_flight = %lu\n", time_of_flight);
#endif

			/* Calculate range */
			uint32_t odr = dev_ptr->odr_out;
			uint32_t rtc_cal_pulse_ms = (uint32_t) dev_ptr->group->rtc_cal_pulse_ms;

			uint32_t num = (CH_SPEEDOFSOUND_MPS * ((rtc_cal_pulse_ms * time_of_flight * (uint32_t)dev_ptr->fcount_cycles) >> (2+odr))); //bitshift down by 10 to handle overflo;
			uint32_t den = ((uint32_t) dev_ptr->rtc_cal_result * (uint32_t) dev_ptr->pmut_clock_fcount) >> 11; //this one can't overflow, bitshift down by 10 to match num

			range = (num / den);

#ifdef CHDRV_DEBUG  // formula debug
			printf("num = (%u * ((%lu * %lu * %lu) >> (2 + %lu)))) = %lu\n",
				CH_SPEEDOFSOUND_MPS, rtc_cal_pulse_ms, time_of_flight, (uint32_t)dev_ptr->fcount_cycles, odr, num);
			printf("den = ((%u * %u) >> 11) = %lu\n", dev_ptr->rtc_cal_result, dev_ptr->pmut_clock_fcount, den);
			printf("range = %u   num = %lu   den = %lu\n", range, num, den);
#endif

			if (range_type == CH_RANGE_ECHO_ONE_WAY) {
				range /= 2;
			}
		}

#elif defined(INCLUDE_WHITNEY_SUPPORT)		// XXX TODO this can co-exist with better if() structure
		if (dev_ptr->part_number == CH101_PART_NUMBER) {
			tof_reg = CH101_COMMON_REG_TOF;
		} else {
			tof_reg = CH201_COMMON_REG_TOF;
		}
  
        uint16_t time_of_flight_16bit;
		err = chdrv_read_word(dev_ptr, tof_reg, &time_of_flight_16bit);
        time_of_flight = (uint32_t)time_of_flight_16bit;

		if (!err && (time_of_flight != UINT16_MAX)) { // If object detected

			if (dev_ptr->scale_factor == 0) {
				dev_ptr->store_scalefactor(dev_ptr);
			}
			scale_factor = dev_ptr->scale_factor;

			if (scale_factor != 0) {
				uint32_t num = (CH_SPEEDOFSOUND_MPS * dev_ptr->group->rtc_cal_pulse_ms * (uint32_t) time_of_flight);
				uint32_t den = ((uint32_t) dev_ptr->rtc_cal_result * (uint32_t) scale_factor) >> 11;		// XXX need define

				range = (num / den);

				if (dev_ptr->part_number == CH201_PART_NUMBER) {
					range *= 2;
				}

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
#endif // INCLUDE_SHASTA_SUPPORT
	}
	return range;
}
#endif	// INCLUDE_ALGO_RANGEFINDER

uint32_t ch_common_get_range(ch_dev_t *dev_ptr, ch_range_t range_type) {
	uint32_t range = 0;

#ifdef INCLUDE_ALGO_RANGEFINDER
	range = ch_common_get_target_range(dev_ptr, CH_DEFAULT_TARGET_NUM, range_type);
#else
	(void)dev_ptr;
	(void)range_type;
#endif

	return range;
}

uint16_t ch_common_get_target_amplitude(ch_dev_t *dev_ptr, uint8_t target_num) {
	uint16_t amplitude = 0;
	(void)target_num;

#ifdef INCLUDE_ALGO_RANGEFINDER
	uint16_t  amplitude_reg = 0;

	if (dev_ptr->sensor_connected) {
		if (dev_ptr->asic_gen == CH_ASIC_GEN_1_WHITNEY) {
			if (dev_ptr->part_number == CH101_PART_NUMBER) {
				amplitude_reg = CH101_COMMON_REG_AMPLITUDE;
			} else if (dev_ptr->part_number == CH201_PART_NUMBER) {
				amplitude_reg = CH201_COMMON_REG_AMPLITUDE;
			}
		} else if (dev_ptr->asic_gen == CH_ASIC_GEN_2_SHASTA) {
#ifdef INCLUDE_SHASTA_SUPPORT
			amplitude_reg = (uint16_t)(uintptr_t)&((dev_ptr->sens_algo_out_addr)->tL.targets[target_num].amplitude);
#endif // INCLUDE_SHASTA_SUPPORT
		}

		chdrv_read_word(dev_ptr, amplitude_reg, &amplitude);
	}

#else   // INCLUDE_ALGO_RANGEFINDER
	(void)dev_ptr;
#endif // INCLUDE_ALGO_RANGEFINDER

	return amplitude;
}

uint16_t ch_common_get_amplitude(ch_dev_t *dev_ptr) {
	uint16_t amplitude;

	amplitude = ch_common_get_target_amplitude(dev_ptr, CH_DEFAULT_TARGET_NUM);

	return amplitude;
}

uint16_t ch_common_get_num_output_samples(ch_dev_t *dev_ptr) {
	uint16_t num_samples = 0;

	if (dev_ptr->sensor_connected) {
		if (dev_ptr->asic_gen == CH_ASIC_GEN_1_WHITNEY) {
			num_samples = dev_ptr->num_rx_samples;			// always return the max sample count

		} else if (dev_ptr->asic_gen == CH_ASIC_GEN_2_SHASTA) {
#ifdef INCLUDE_SHASTA_SUPPORT
			uint16_t num_iq_bytes_addr = (uint16_t)(uintptr_t)&((dev_ptr->sens_cfg_addr)->raw.num_iq_bytes);
			uint16_t num_iq_bytes = 0;

			uint8_t err = chdrv_read_word(dev_ptr, num_iq_bytes_addr, &num_iq_bytes);
			if (!err) {
				num_samples = (num_iq_bytes / sizeof(ch_iq_sample_t));
			}
#endif // INCLUDE_SHASTA_SUPPORT
		}
	}

	return num_samples;
}

uint8_t ch_common_get_locked_state(ch_dev_t *dev_ptr) {		// XXX TODO make Whitney-only
	uint8_t ready_reg;
	uint8_t lock_mask = dev_ptr->freqLockValue;
	uint8_t ret_val = 0;

	if (dev_ptr->part_number == CH101_PART_NUMBER) {
		ready_reg = CH101_COMMON_REG_READY;
	} else {
		ready_reg = CH201_COMMON_REG_READY;
	}

	if (dev_ptr->sensor_connected) {
		uint8_t ready_value = 0;
		chdrv_read_byte(dev_ptr, ready_reg, &ready_value);
		if (ready_value & lock_mask) {
			ret_val = 1;
		}
	}
	return ret_val;
}

void ch_common_prepare_pulse_timer(ch_dev_t *dev_ptr) {
#ifdef INCLUDE_SHASTA_SUPPORT
	chdrv_event_trigger(dev_ptr, EVENT_CONFIG_CLOCKS);

	chdrv_event_trigger(dev_ptr, EVENT_RTC_CAL);

#elif defined(INCLUDE_WHITNEY_SUPPORT)
	uint8_t cal_trig_reg;

	if (dev_ptr->part_number == CH101_PART_NUMBER) {
		cal_trig_reg = CH101_COMMON_REG_CAL_TRIG;
	} else if (dev_ptr->part_number == CH201_PART_NUMBER) {
		cal_trig_reg = CH201_COMMON_REG_CAL_TRIG;
	}

	chdrv_write_byte(dev_ptr, cal_trig_reg, 0);
#endif  // INCLUDE_SHASTA_SUPPORT
}

void ch_common_store_pt_result(ch_dev_t *dev_ptr) {
	uint16_t pt_result_reg = 0;
	uint16_t rtc_cal_result;

	if (dev_ptr->asic_gen == CH_ASIC_GEN_1_WHITNEY) {
		if (dev_ptr->part_number == CH101_PART_NUMBER) {
			pt_result_reg = CH101_COMMON_REG_CAL_RESULT;
		} else if (dev_ptr->part_number == CH201_PART_NUMBER) {
			pt_result_reg = CH201_COMMON_REG_CAL_RESULT;
		}
	} else if (dev_ptr->asic_gen == CH_ASIC_GEN_2_SHASTA) {
#ifdef INCLUDE_SHASTA_SUPPORT
		pt_result_reg = (uint16_t)(uintptr_t)&((dev_ptr->sens_cfg_addr)->raw.rtc_cal_result);
#endif // INCLUDE_SHASTA_SUPPORT
	}

	chdrv_read_word(dev_ptr, pt_result_reg, &rtc_cal_result);
	dev_ptr->rtc_cal_result = rtc_cal_result;

	dev_ptr->rtc_frequency = (rtc_cal_result * 1000) / dev_ptr->group->rtc_cal_pulse_ms;
}


void ch_common_store_op_freq(ch_dev_t *dev_ptr){
#ifdef INCLUDE_SHASTA_SUPPORT
	clock_control_t *clock_ctrl_ptr = (clock_control_t *)&((dev_ptr->sens_cfg_addr)->common.clock);
	raw_output_data_t *output_data_ptr = (raw_output_data_t *)&((dev_ptr->sens_cfg_addr)->raw);
	uint16_t raw_pmut_count;
	uint32_t pmut_freq;
	uint32_t cpu_freq;
	uint16_t reg_addr;

#ifdef CHDRV_DEBUG
	printf("\n\r");
	printf("ch_common_store_op_freq:  dev_ptr->fcount_cycles = %u\n", dev_ptr->fcount_cycles);
	printf("ch_common_store_op_freq:  dev_ptr->rtc_frequency = %u Hz\n", dev_ptr->rtc_frequency);
#endif

	/* Run BIST on sensor */
	chdrv_run_bist(dev_ptr);

	/* Calculate PMUT operating frequency */
	reg_addr = (uint16_t)(uintptr_t)&(clock_ctrl_ptr->fcount_cycles);			// fcount cycles
	chdrv_write_word(dev_ptr, reg_addr, dev_ptr->fcount_cycles);

	chdrv_event_trigger_and_wait(dev_ptr, EVENT_MUT_FCOUNT);					// start PMUT freq count

	reg_addr = (uint16_t)(uintptr_t)&(output_data_ptr->pmut_clock_fcount);	// PMUT freq count
	chdrv_read_word(dev_ptr, reg_addr, &raw_pmut_count);

	pmut_freq = ((dev_ptr->rtc_frequency * raw_pmut_count) / dev_ptr->fcount_cycles) / 16;
#ifdef CHDRV_DEBUG
	printf("ch_common_store_op_freq:  raw_pmut_count = %u pmut_freq = %lu Hz\n", raw_pmut_count, pmut_freq);
#endif

	/* Adjust cpu frequency to avoid multiple of pmut freq */
	cpu_freq = chdrv_cpu_freq_adjust(dev_ptr, pmut_freq);

	/* Store values in device descriptor */
	dev_ptr->op_frequency = pmut_freq;
	dev_ptr->cpu_frequency = cpu_freq;
	dev_ptr->pmut_clock_fcount = raw_pmut_count;

#elif defined(INCLUDE_WHITNEY_SUPPORT)
	uint8_t	 tof_sf_reg;
	uint16_t raw_freq;		// aka scale factor
	uint32_t freq_counter_cycles;
	uint32_t num;
	uint32_t den;
	uint32_t op_freq;

	if (dev_ptr->part_number == CH101_PART_NUMBER) {
		tof_sf_reg = CH101_COMMON_REG_TOF_SF;
	} else {
		tof_sf_reg = CH201_COMMON_REG_TOF_SF;
	}

	freq_counter_cycles = dev_ptr->freqCounterCycles;

	chdrv_read_word(dev_ptr, tof_sf_reg, &raw_freq);

	num = (uint32_t)(((dev_ptr->rtc_cal_result)*1000U) / (16U * freq_counter_cycles)) * (uint32_t)(raw_freq);
	den = (uint32_t)(dev_ptr->group->rtc_cal_pulse_ms);
	op_freq = (num/den);

	dev_ptr->op_frequency = op_freq;
	dev_ptr->cpu_frequency = chdrv_cpu_freq_calculate(dev_ptr);

#endif
}


void ch_common_store_bandwidth(ch_dev_t *dev_ptr) {
	uint32_t bandwidth = 0;
	ch_iq_sample_t iq_buf[2];		// XXX assumes two consecutive samples

	(void) ch_get_iq_data(dev_ptr, iq_buf, CH_COMMON_BANDWIDTH_INDEX_1, 2, CH_IO_MODE_BLOCK);

	uint32_t mag1sq = (uint32_t)(((int32_t)iq_buf[0].i * (int32_t)iq_buf[0].i) + ((int32_t)iq_buf[0].q * (int32_t)iq_buf[0].q));
	uint32_t mag2sq = (uint32_t)(((int32_t)iq_buf[1].i * (int32_t)iq_buf[1].i) + ((int32_t)iq_buf[1].q * (int32_t)iq_buf[1].q));

	//can perform below calculations using floating point for higher accuracy.
	bandwidth = FIXEDMUL(FP_log( FP_sqrt( FIXEDDIV(mag1sq,mag2sq))), 
			             (FIXEDDIV(INT2FIXED((uint64_t)dev_ptr->op_frequency),
								   (FIXED_PI * ((CH_COMMON_BANDWIDTH_INDEX_2 - CH_COMMON_BANDWIDTH_INDEX_1) * 8 )))));

	dev_ptr->bandwidth = (uint16_t) FIXED2INT(bandwidth);
}

void ch_common_store_scale_factor(ch_dev_t *dev_ptr) {
	uint8_t	err;
	uint8_t	tof_sf_reg;
	uint16_t scale_factor;

	if (dev_ptr->part_number == CH101_PART_NUMBER) {
		tof_sf_reg = CH101_COMMON_REG_TOF_SF;
	} else {
		tof_sf_reg = CH201_COMMON_REG_TOF_SF;
	}

	err = chdrv_read_word(dev_ptr, tof_sf_reg, &scale_factor);
	if (!err) {
		dev_ptr->scale_factor = scale_factor;
	} else {
		dev_ptr->scale_factor = 0;
	}
}


uint8_t ch_common_set_thresholds(ch_dev_t *dev_ptr, ch_thresholds_t *lib_thresh_buf_ptr) {	// XXX TODO only meas 0 for now
	int ret_val = 1;		// default is error return

#ifdef INCLUDE_ALGO_RANGEFINDER
#ifdef INCLUDE_WHITNEY_SUPPORT
	uint8_t thresh_level_reg;	// threshold level reg (first in array)
#endif // INCLUDE_WHITNEY_SUPPORT
	uint8_t max_num_thresholds = 0;
	uint8_t	thresh_num;

	if (dev_ptr->sensor_connected) {
		
		if (dev_ptr->asic_gen == CH_ASIC_GEN_1_WHITNEY) {
			if (dev_ptr->part_number == CH101_PART_NUMBER) {
				return ret_val;		// NOT SUPPORTED in CH101

			} else if (dev_ptr->part_number == CH201_PART_NUMBER) {
#ifdef INCLUDE_WHITNEY_SUPPORT
				thresh_level_reg = CH201_COMMON_REG_THRESHOLDS;
#endif // INCLUDE_WHITNEY_SUPPORT
				max_num_thresholds = CH201_COMMON_NUM_THRESHOLDS;
			}
		} else if (dev_ptr->asic_gen == CH_ASIC_GEN_2_SHASTA) {
			max_num_thresholds = ICU_COMMON_NUM_THRESHOLDS;
		}

		dev_ptr->max_num_thresholds = max_num_thresholds;		// store in ch_dev_t

		for (thresh_num = 0; thresh_num < max_num_thresholds; thresh_num++) {

#ifdef INCLUDE_SHASTA_SUPPORT
			thresholds_t *sens_thresh_ptr = (thresholds_t *)&(dev_ptr->algo_cfg.meas_cfg[CH_DEFAULT_MEAS_NUM].thresholds);	// XXX TODO handle other meas

			if (dev_ptr->asic_gen == CH_ASIC_GEN_2_SHASTA) {
				/* Write new threshold values to local copy of algo measurement config */

				sens_thresh_ptr->threshold[thresh_num] = lib_thresh_buf_ptr->threshold[thresh_num].level;

				if (thresh_num == (max_num_thresholds - 1)) {
					sens_thresh_ptr->stop_index[thresh_num] = dev_ptr->max_samples;
				} else {
					sens_thresh_ptr->stop_index[thresh_num] = lib_thresh_buf_ptr->threshold[thresh_num + 1].start_sample;
				}

				/* If last threshold, write to sensor */
				if (thresh_num == (max_num_thresholds - 1)) {		
					uint16_t thresholds_addr = 
						(uint16_t)(uintptr_t) &((dev_ptr->sens_algo_cfg_addr)->meas_cfg[CH_DEFAULT_MEAS_NUM].thresholds);

					ret_val = chdrv_burst_write(dev_ptr, thresholds_addr, (uint8_t *) sens_thresh_ptr, sizeof(thresholds_t));
				}
			}

#elif defined(INCLUDE_WHITNEY_SUPPORT)
			uint8_t	thresh_len_reg = 0;		// offset of register for this threshold's length
			uint8_t thresh_len;
			uint16_t thresh_level;
			uint16_t start_sample = 0;

			if (thresh_num < (max_num_thresholds - 1)) {
				uint16_t next_start_sample = lib_thresh_buf_ptr->threshold[thresh_num + 1].start_sample;

				thresh_len = (next_start_sample - start_sample);
				start_sample  = next_start_sample;
			} else {
				thresh_len = 0;
			}

			if (dev_ptr->part_number == CH201_PART_NUMBER) {
				if (thresh_num == 0) {
					thresh_len_reg = CH201_COMMON_REG_THRESH_LEN_0;
				} else if (thresh_num == 1) {
					thresh_len_reg = CH201_COMMON_REG_THRESH_LEN_1;
				} else if (thresh_num == 2) {
					thresh_len_reg = CH201_COMMON_REG_THRESH_LEN_2;
				} else if (thresh_num == 3) {
					thresh_len_reg = CH201_COMMON_REG_THRESH_LEN_3;
				} else if (thresh_num == 4) {
					thresh_len_reg = CH201_COMMON_REG_THRESH_LEN_4;
				} else if (thresh_num == 5) {
					thresh_len_reg = 0;			// last threshold does not have length field - assumed to extend to end of data
				}
			}

			if (thresh_len_reg != 0) {
				ret_val = chdrv_write_byte(dev_ptr, thresh_len_reg, thresh_len); 	// set the length field (if any) for this threshold
			}
			// write level to this threshold's entry in register array
			thresh_level = lib_thresh_buf_ptr->threshold[thresh_num].level;
			ret_val |= chdrv_write_word(dev_ptr, (thresh_level_reg + (thresh_num * sizeof(uint16_t))), thresh_level);
#endif	// INCLUDE_SHASTA_SUPPORT

		}	// end 	for (thresh_num = 0; thresh_num < max_num_thresholds; thresh_num++)

	}	// end 	if (dev_ptr->sensor_connected)

#else
	(void)dev_ptr;
	(void)lib_thresh_buf_ptr;
#endif 	// INCLUDE_ALGO_RANGEFINDER

	return ret_val;
}


uint8_t ch_common_get_thresholds(ch_dev_t *dev_ptr, ch_thresholds_t *lib_thresh_buf_ptr) {
		uint8_t ret_val = 1;		// default = error return
	if (dev_ptr->sensor_connected && (lib_thresh_buf_ptr != NULL)) {
		
#ifdef INCLUDE_SHASTA_SUPPORT
#ifdef INCLUDE_ALGO_RANGEFINDER
		thresholds_t *sens_thresh_ptr = (thresholds_t *) &(dev_ptr->algo_cfg.meas_cfg[CH_DEFAULT_MEAS_NUM].thresholds);	// XXX TODO handle other meas

		/* Refresh copy in ch_dev_t */
		uint16_t thresh_start_addr = (uint16_t)(uintptr_t) &((dev_ptr->sens_algo_cfg_addr)->meas_cfg[CH_DEFAULT_MEAS_NUM].thresholds);
		chdrv_burst_read(dev_ptr, thresh_start_addr, (uint8_t *)sens_thresh_ptr, sizeof(thresholds_t));

		/* Copy fields to user buffer */
		thresh_sensor_to_lib(dev_ptr, sens_thresh_ptr, lib_thresh_buf_ptr);		// convert format and copy

		ret_val = 0;
#endif	// INCLUDE_ALGO_RANGEFINDER

#elif defined(INCLUDE_WHITNEY_SUPPORT)
		uint8_t	thresh_len_reg = 0;		// offset of register for this threshold's length
		uint8_t thresh_level_reg;	// threshold level reg (first in array)
		uint8_t max_num_thresholds;
		uint8_t thresh_num;
		uint8_t	thresh_len = 0;		// number of samples described by each threshold
		uint16_t	start_sample = 0;	// calculated start sample for each threshold

		if (dev_ptr->part_number == CH101_PART_NUMBER) {
			return ret_val;		// NOT SUPPORTED in CH101
			
		} else {
			thresh_level_reg = CH201_COMMON_REG_THRESHOLDS;
			max_num_thresholds = CH201_COMMON_NUM_THRESHOLDS;
		}

		for (thresh_num = 0; thresh_num < max_num_thresholds; thresh_num++) {

			if (dev_ptr->part_number == CH201_PART_NUMBER) {
				if (thresh_num == 0) {
					thresh_len_reg = CH201_COMMON_REG_THRESH_LEN_0;
				} else if (thresh_num == 1) {
					thresh_len_reg = CH201_COMMON_REG_THRESH_LEN_1;
				} else if (thresh_num == 2) {
					thresh_len_reg = CH201_COMMON_REG_THRESH_LEN_2;
				} else if (thresh_num == 3) {
					thresh_len_reg = CH201_COMMON_REG_THRESH_LEN_3;
				} else if (thresh_num == 4) {
					thresh_len_reg = CH201_COMMON_REG_THRESH_LEN_4;
				} else if (thresh_num == 5) {
					thresh_len_reg = 0;			// last threshold does not have length field - assumed to extend to end of data
				}
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
#endif	// INCLUDE_SHASTA_SUPPORT
	}
	return ret_val;
}


#ifdef INCLUDE_SHASTA_SUPPORT
#ifdef INCLUDE_ALGO_RANGEFINDER

static void thresh_lib_to_sensor(ch_dev_t *dev_ptr, ch_thresholds_t *lib_thresh_ptr, thresholds_t *sens_thresh_ptr) {
	uint8_t	thresh_num;
	uint8_t max_num_thresholds = CH_NUM_THRESHOLDS;

	for (thresh_num = 0; thresh_num < max_num_thresholds; thresh_num++) {

		sens_thresh_ptr->threshold[thresh_num] = lib_thresh_ptr->threshold[thresh_num].level;

		if (thresh_num == (max_num_thresholds - 1)) {
			sens_thresh_ptr->stop_index[thresh_num] = dev_ptr->max_samples;
		} else {
			sens_thresh_ptr->stop_index[thresh_num] = lib_thresh_ptr->threshold[thresh_num + 1].start_sample;
		}
	}
}

static void thresh_sensor_to_lib(ch_dev_t *dev_ptr, thresholds_t *sens_thresh_ptr, ch_thresholds_t *lib_thresh_ptr)  {
	uint8_t	thresh_num;
	uint16_t start_sample;
	uint8_t max_num_thresholds = CH_NUM_THRESHOLDS;
	(void)dev_ptr;

	for (thresh_num = 0; thresh_num < max_num_thresholds; thresh_num++) {

		if (thresh_num == 0) {
			start_sample = 0;
		} else {
			start_sample = sens_thresh_ptr->stop_index[thresh_num-1];
		}

		lib_thresh_ptr->threshold[thresh_num].start_sample = start_sample;

		lib_thresh_ptr->threshold[thresh_num].level = sens_thresh_ptr->threshold[thresh_num];
	}
}
#endif	// INCLUDE_ALGO_RANGEFINDER
#endif // INCLUDE_SHASTA_SUPPORT


static uint8_t get_sample_data(ch_dev_t *dev_ptr, ch_iq_sample_t *buf_ptr, uint16_t start_sample, uint16_t num_samples,
							   ch_io_mode_t mode, uint8_t sample_size_in_bytes) {

	uint16_t iq_data_addr;
	uint16_t num_bytes = (num_samples * sample_size_in_bytes);
	ch_group_t *grp_ptr = dev_ptr->group;
	int      err = 0;

	if ((num_samples == 0) || ((start_sample + num_samples) > dev_ptr->max_samples)) {
		return 1;
	}

#ifdef INCLUDE_SHASTA_SUPPORT
	iq_data_addr = (uint16_t)(uintptr_t)&((dev_ptr->sens_cfg_addr)->raw.IQdata); 	// start of I/Q data
	iq_data_addr += (start_sample * sample_size_in_bytes);				   		// add sample offset

	if (mode == CH_IO_MODE_BLOCK) {
		/* blocking transfer */
		err = chdrv_burst_read(dev_ptr, iq_data_addr, (uint8_t *) buf_ptr, num_bytes);
	} else {
		/* non-blocking transfer */
		err = chdrv_group_queue(grp_ptr, dev_ptr, 1, CHDRV_NB_TRANS_TYPE_STD, iq_data_addr, num_bytes,
									  (uint8_t*) buf_ptr);
	}
	
#elif defined(INCLUDE_WHITNEY_SUPPORT)		// XXX TODO reconcile I2C vs SPI
	uint8_t	   use_prog_read = 0;		// default = do not use low-level programming interface

#ifndef USE_STD_I2C_FOR_IQ
	if (grp_ptr->num_connected[dev_ptr->bus_index] == 1) {		// if only one device on this bus
		use_prog_read = 1;											//   use low-level interface
	}
#endif

	if (dev_ptr->part_number == CH101_PART_NUMBER) {
		iq_data_addr = CH101_COMMON_REG_DATA;
	} else {
		iq_data_addr = CH201_COMMON_REG_DATA;
	}

	iq_data_addr += (start_sample * sample_size_in_bytes);

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
			if (dev_ptr->part_number == CH101_PART_NUMBER) {
				iq_data_addr += (CH101_DATA_MEM_ADDR + CH101_COMMON_I2CREGS_OFFSET);
			} else {
				iq_data_addr += (CH201_DATA_MEM_ADDR + CH201_COMMON_I2CREGS_OFFSET);
			}

			err = chdrv_group_queue(grp_ptr, dev_ptr, 1, CHDRV_NB_TRANS_TYPE_PROG, iq_data_addr, num_bytes,
										(uint8_t *) buf_ptr);
		} else {
			/* Use regular I2C register interface to read data */
			err = chdrv_group_queue(grp_ptr, dev_ptr, 1, CHDRV_NB_TRANS_TYPE_STD, iq_data_addr, num_bytes,
										  (uint8_t*) buf_ptr);
		}
	}

#endif	// INCLUDE_WHITNEY_SUPPORT

	return err;
}

uint8_t	 ch_common_set_sample_window(ch_dev_t *dev_ptr, uint16_t start_sample, uint16_t num_samples) {
	uint8_t err = 1;
	uint16_t max_samples = dev_ptr->num_rx_samples;

	if ((num_samples <= CH_WINDOW_MAX_SAMPLES) && ((start_sample + num_samples) < max_samples)) {
		dev_ptr->win_start_sample = start_sample;
		dev_ptr->num_win_samples = num_samples;
		err = 0;
	}

	return err;
}


uint16_t ch_common_get_amplitude_avg(ch_dev_t *dev_ptr) {
	uint16_t start_sample = dev_ptr->win_start_sample;
	uint16_t num_samples = dev_ptr->num_win_samples;
	uint32_t total_amp = 0;
	uint32_t avg_amp = 0;
	uint8_t err = 0;

	if ((start_sample != 0) && (num_samples != 0)) {

#ifdef INCLUDE_SHASTA_SUPPORT
		uint16_t window_amp_buf[CH_WINDOW_MAX_SAMPLES];

		err = ch_get_amplitude_data(dev_ptr, window_amp_buf, start_sample, num_samples, CH_IO_MODE_BLOCK);
		if (!err) {
			for (uint16_t idx = 0; idx < num_samples; idx++) {
				total_amp += window_amp_buf[idx];						// add amplitude for this sample
			}
		}
#elif defined(INCLUDE_WHITNEY_SUPPORT)
		ch_iq_sample_t window_iq_buf[CH_WINDOW_MAX_SAMPLES];

		err = ch_get_iq_data(dev_ptr, window_iq_buf, start_sample, num_samples, CH_IO_MODE_BLOCK);
		if (!err) {
			for (uint16_t idx = 0; idx < num_samples; idx++) {
				total_amp += ch_iq_to_amplitude(&(window_iq_buf[idx]));	// add calculated amplitude
			}
		}
#endif
		avg_amp = (total_amp / num_samples);
	}

	return (uint16_t) avg_amp;
}

uint8_t  ch_common_get_iq_data(ch_dev_t *dev_ptr, ch_iq_sample_t *buf_ptr, uint16_t start_sample, uint16_t num_samples,
							   ch_io_mode_t mode) {

	return get_sample_data(dev_ptr, buf_ptr, start_sample, num_samples, mode, sizeof(ch_iq_sample_t));
}


uint8_t ch_common_get_amplitude_data(ch_dev_t *dev_ptr, uint16_t *buf_ptr, uint16_t start_sample, uint16_t num_samples,
									 ch_io_mode_t mode) {
	uint8_t			error = 0;

#ifdef INCLUDE_SHASTA_SUPPORT
	if (ch_meas_get_iq_output(dev_ptr, CH_DEFAULT_MEAS_NUM) == IQ_OUTPUT_MAG) {
		/* Get amplitude values directly from sensor */
		error = get_sample_data(dev_ptr, (ch_iq_sample_t *)buf_ptr, start_sample, num_samples, mode, sizeof(uint16_t));
	} else {
		error = 1;						// sensor must be in mag output mode
	}
#elif defined(INCLUDE_WHITNEY_SUPPORT)
	ch_iq_sample_t	iq_buf[CH_IQ_SAMPLES_PER_READ];
	uint16_t		samples_in_chunk = 0;
	uint16_t		sample_num   = start_sample;
	uint16_t		samples_left = num_samples;
	uint8_t 		chunks_left  = (num_samples + CH_IQ_SAMPLES_PER_READ - 1) / CH_IQ_SAMPLES_PER_READ;

	/* Validate mode (only blocking mode is supported) and sample count/offset */
	if ((mode != CH_IO_MODE_BLOCK) || (start_sample + num_samples > dev_ptr->max_samples)) {
		error = 1;
	}

	while (!error && (chunks_left-- > 0)) {

		/* Read I/Q data */
		if (samples_left > CH_IQ_SAMPLES_PER_READ) {
			samples_in_chunk = CH_IQ_SAMPLES_PER_READ;
		} else {
			samples_in_chunk = samples_left;
		}

		samples_left -= samples_in_chunk;			// adjust remaining sample count for next pass

		error = get_sample_data(dev_ptr, iq_buf, sample_num, samples_in_chunk, mode, sizeof(ch_iq_sample_t));
		if (error) {
			break;
		}

		/* Calculate amplitudes and store in user buffer */
		for (uint16_t idx = 0; idx < samples_in_chunk; idx++) {
			buf_ptr[sample_num++] = ch_iq_to_amplitude(&iq_buf[idx]);
		}

	}
#endif  // INCLUDE_SHASTA_SUPPORT

	return error;
}


uint8_t ch_common_set_time_plan(ch_dev_t *dev_ptr, ch_time_plan_t time_plan) {
	uint8_t time_plan_reg;
	uint8_t ret_val = 1;		// default return is error

	if (dev_ptr->part_number == CH101_PART_NUMBER) {			// CH-101 only - SonicSync unsupported in CH-201
		time_plan_reg = CH101_COMMON_REG_TIME_PLAN;

		if (dev_ptr->sensor_connected) {
			chdrv_write_byte(dev_ptr, time_plan_reg, time_plan);
			ret_val = 0;
		}
	}

	return ret_val;			// error - SonicSync unsupported in CH-201
}


ch_time_plan_t ch_common_get_time_plan(ch_dev_t *dev_ptr) {
	uint8_t time_plan_reg;
	uint8_t time_plan = CH_TIME_PLAN_NONE;

	if (dev_ptr->part_number == CH101_PART_NUMBER) {			// CH-101 only - SonicSync unsupported in CH-201
		time_plan_reg = CH101_COMMON_REG_TIME_PLAN;

		if (dev_ptr->sensor_connected) {
			chdrv_read_byte(dev_ptr, time_plan_reg, &time_plan);
		}
	}

	return (ch_time_plan_t) time_plan;
}


uint8_t ch_common_set_rx_holdoff(ch_dev_t *dev_ptr, uint16_t num_samples) {
	uint8_t ret_val = RET_OK;

#ifdef INCLUDE_ALGO_RANGEFINDER
#ifdef INCLUDE_SHASTA_SUPPORT
	ch_thresholds_t	 thresholds;
	uint8_t	thresh_num;

	/* Get current threshold settings */
	ch_common_get_thresholds(dev_ptr, &thresholds);

	if (num_samples < thresholds.threshold[1].start_sample) {		// if rx holdoff is shorter than first threshold
		/* Need to insert a new threshold[0] for holdoff */
		thresh_num = (CH_NUM_THRESHOLDS - 2); 			// start at next-to-last entry
		for (uint8_t i = 0; i < (CH_NUM_THRESHOLDS - 1); i++) {
			// shuffle contents up one entry
			thresholds.threshold[thresh_num + 1].start_sample = thresholds.threshold[thresh_num].start_sample;
			thresholds.threshold[thresh_num + 1].level        = thresholds.threshold[thresh_num].level;
			thresh_num--;
		}

		thresholds.threshold[0].level = CH_THRESH_LEVEL_HOLDOFF;		// first thresh get holdoff (max) value
		thresholds.threshold[1].start_sample = num_samples;				// second thresh starts after holdoff samples

	} else {
		/* Increase/extend first threshold(s) to prevent detection */
		uint16_t holdoff_samples_left = num_samples;

		thresh_num = 0;

		while ((holdoff_samples_left > 0) && (thresh_num < (CH_NUM_THRESHOLDS - 1))) {
			uint16_t thresh_samples;

			if (thresholds.threshold[thresh_num + 1].start_sample <= num_samples) {
				/* this entire threshold is part of holdoff area */
				thresholds.threshold[thresh_num].level = CH_THRESH_LEVEL_HOLDOFF;

				thresh_samples = (thresholds.threshold[thresh_num + 1].start_sample - 
								  thresholds.threshold[thresh_num].start_sample);

				holdoff_samples_left -= thresh_samples;
			} else {
				/* end of holdoff is in this threshold - change start to extend previous holdoff thresh */
				thresholds.threshold[thresh_num].start_sample = num_samples;

				break;		// done now
			}

			thresh_num++;
		}

	}  // end if (num_samples < thresholds.threshold[1].start_sample)

	/* Write updated threshold values */
	ret_val = ch_common_set_thresholds(dev_ptr, &thresholds);


#elif defined(INCLUDE_WHITNEY_SUPPORT)
	uint8_t rx_holdoff_reg;
	uint16_t reg_value;

	if (dev_ptr->part_number == CH101_PART_NUMBER) {
		rx_holdoff_reg = CH101_COMMON_REG_RX_HOLDOFF;
		reg_value = num_samples;
	} else {
		rx_holdoff_reg = CH201_COMMON_REG_RX_HOLDOFF;
		reg_value = (num_samples / 2);			// CH201 value is 1/2 actual sample count
	}

	if (dev_ptr->sensor_connected) {
		ret_val |= chdrv_write_byte(dev_ptr, rx_holdoff_reg, (uint8_t) reg_value);
	}
#endif // INCLUDE_SHASTA_SUPPORT

#else
	(void)dev_ptr;
	(void)num_samples;
#endif	// INCLUDE_ALGO_RANGEFINDER

	return ret_val;
}


uint16_t ch_common_get_rx_holdoff(ch_dev_t *dev_ptr) {
	uint16_t rx_holdoff = 0;

#ifdef INCLUDE_ALGO_RANGEFINDER
#if defined(INCLUDE_SHASTA_SUPPORT)
	ch_thresholds_t	 thresholds;

	uint8_t ret_val = ch_common_get_thresholds(dev_ptr, &thresholds);
	if (RET_OK == ret_val) {
		/* first threshold ends at start of the second thresh */
		rx_holdoff = thresholds.threshold[1].start_sample;
	}
#elif defined(INCLUDE_WHITNEY_SUPPORT)
	uint8_t rx_holdoff_reg;
	uint8_t reg_val;

	if (dev_ptr->sensor_connected) {
		if (dev_ptr->part_number == CH101_PART_NUMBER) {
			rx_holdoff_reg = CH101_COMMON_REG_RX_HOLDOFF;
		} else {
			rx_holdoff_reg = CH201_COMMON_REG_RX_HOLDOFF;
		}
		chdrv_read_byte(dev_ptr, rx_holdoff_reg, &reg_val);

		rx_holdoff = (uint16_t) reg_val;

		if (dev_ptr->part_number == CH201_PART_NUMBER) {
			rx_holdoff *= 2; /* CH201 reports 1/2 actual sample count */
		}
	}
#endif /* INCLUDE_SHASTA_SUPPORT */

#else
	(void)dev_ptr;
#endif	// INCLUDE_ALGO_RANGEFINDER

	return rx_holdoff;
}

uint8_t	ch_common_set_rx_low_gain(ch_dev_t *dev_ptr, uint16_t num_samples) {
	uint8_t rx_lowgain_reg;
	uint8_t reg_value;
	uint8_t ret_val = 0;

	if (dev_ptr->part_number != CH201_PART_NUMBER) {
		ret_val = 1;		// error - only supported on CH201
	} else {
		if (num_samples > dev_ptr->num_rx_samples - 1) {		// do not extend past end of active range
			num_samples = (dev_ptr->num_rx_samples - 1);
		}

		if (num_samples < CH201_COMMON_RX_LOW_GAIN_MIN) {		// enforce minimum
			num_samples = CH201_COMMON_RX_LOW_GAIN_MIN;
		}

		rx_lowgain_reg = CH201_COMMON_REG_LOW_GAIN_RXLEN;
		reg_value = (num_samples / 2);			// CH201 value is 1/2 actual sample count
	}

	if (ret_val == 0) {
		ret_val = chdrv_write_byte(dev_ptr, rx_lowgain_reg, reg_value);
	}

	return ret_val;
}

uint16_t	ch_common_get_rx_low_gain(ch_dev_t *dev_ptr) {
	uint8_t reg_value = 0;
	uint16_t num_samples = 0;

	if (dev_ptr->part_number != CH201_PART_NUMBER) {
		num_samples = 0;		// error - only supported on CH201
	} else {
		chdrv_read_byte(dev_ptr, CH201_COMMON_REG_LOW_GAIN_RXLEN, &reg_value);

		num_samples = reg_value * 2;					// actual sample count is 2x register value
	}

	return num_samples;	
}


uint8_t	ch_common_set_tx_length(ch_dev_t *dev_ptr, uint16_t num_cycles) {
	uint8_t ret_val = 0;

#ifdef CHDRV_DEBUG
        printf("ch_common_set_tx_length: num_cycles=%d\n", num_cycles);	
#endif // CHDRV_DEBUG
	if (dev_ptr->asic_gen == CH_ASIC_GEN_1_WHITNEY) {
		if (dev_ptr->part_number == CH101_PART_NUMBER) {
			ret_val = chdrv_write_byte(dev_ptr, CH101_COMMON_REG_TX_LENGTH, num_cycles);
		} else if (dev_ptr->part_number == CH201_PART_NUMBER) {
			ret_val = chdrv_write_byte(dev_ptr, CH201_COMMON_REG_TX_LENGTH, num_cycles);
		}
	} else if (dev_ptr->asic_gen == CH_ASIC_GEN_2_SHASTA) {
#ifdef INCLUDE_SHASTA_SUPPORT          
		for (uint8_t seg_num = 0; seg_num < CH_MEAS_MAX_SEGMENTS; seg_num++) {
			pmut_transceiver_inst_t  *inst_ptr = (pmut_transceiver_inst_t  *)&(dev_ptr->meas_queue.meas[CH_DEFAULT_MEAS_NUM].trx_inst[seg_num]);
			ch_meas_seg_type_t seg_type;

			seg_type = (ch_meas_seg_type_t) ((inst_ptr->cmd_config >> PMUT_CMD_BITSHIFT) & PMUT_CMD_BITS);

			if (seg_type == CH_MEAS_SEG_TYPE_TX) {
				inst_ptr->length = num_cycles;

				break;			// only modify first tx found
			}
		}
		ret_val = ch_meas_write_config(dev_ptr);
#endif // INCLUDE_SHASTA_SUPPORT
	}
	return ret_val;
}

uint16_t	ch_common_get_tx_length(ch_dev_t *dev_ptr) {
	uint16_t num_cycles = 0;

	uint8_t num_cycles_8 = 0;
	if (dev_ptr->asic_gen == CH_ASIC_GEN_1_WHITNEY) {
		if (dev_ptr->part_number == CH101_PART_NUMBER) {
			chdrv_read_byte(dev_ptr, CH101_COMMON_REG_TX_LENGTH, &num_cycles_8);
		} else if (dev_ptr->part_number == CH201_PART_NUMBER) {
			chdrv_read_byte(dev_ptr, CH201_COMMON_REG_TX_LENGTH, &num_cycles_8);
		}
		num_cycles = num_cycles_8;
		
	} else if (dev_ptr->asic_gen == CH_ASIC_GEN_2_SHASTA) {
#ifdef INCLUDE_SHASTA_SUPPORT          
		for (uint8_t seg_num = 0; seg_num < CH_MEAS_MAX_SEGMENTS; seg_num++) {
			ch_meas_seg_type_t seg_type;
			uint16_t cmd_config = dev_ptr->meas_queue.meas[CH_DEFAULT_MEAS_NUM].trx_inst[seg_num].cmd_config;
			uint16_t seg_length = dev_ptr->meas_queue.meas[CH_DEFAULT_MEAS_NUM].trx_inst[seg_num].length;

			seg_type = (ch_meas_seg_type_t) ((cmd_config >> PMUT_CMD_BITSHIFT) & PMUT_CMD_BITS);

			if (seg_type == CH_MEAS_SEG_TYPE_TX) {
				num_cycles = seg_length;
				break;			// only report first Tx found
			}
		}
#endif // INCLUDE_SHASTA_SUPPORT          
	}
	return num_cycles;	
}

uint8_t	ch_common_set_cal_result(ch_dev_t *dev_ptr, ch_cal_result_t *cal_ptr) {
	uint8_t ret_val = RET_OK;

	if (dev_ptr->part_number == CH101_PART_NUMBER) {
		if (dev_ptr->sensor_connected && (cal_ptr != NULL)) {
			ret_val |= chdrv_write_word(dev_ptr, CH101_COMMON_REG_DCO_PERIOD, cal_ptr->dco_period);
			ret_val |= chdrv_write_word(dev_ptr, CH101_COMMON_REG_REV_CYCLES, cal_ptr->rev_cycles);
		}
		else {
			ret_val = RET_ERR;
		}
	}
	else {
		ret_val = RET_ERR;
	}
	return ret_val;
}

uint8_t ch_common_set_rtc(ch_dev_t *dev_ptr, ch_rtc_src_t rtc_source, uint16_t rtc_freq) {
	uint8_t ret_val = RET_OK;

	if ((rtc_freq == 0) && (rtc_source == CH_RTC_SRC_EXTERNAL))  {
		ret_val = RET_ERR;						// freq is required if using external RTC source
	}
	if ((rtc_freq == 0) && (dev_ptr->asic_gen == CH_ASIC_GEN_1_WHITNEY))  {
		ret_val = RET_ERR;						// zero is invalid on CH101 & CH201 - no factory test data 
	}
	
	if (ret_val == RET_OK) {
		dev_ptr->rtc_frequency = rtc_freq;	 	// if zero, will use factory test result (Shasta only)
		dev_ptr->group->manual_clock_cal = 1;	// indicate manual clock cal in group descriptor

		if (rtc_source == CH_RTC_SRC_INTERNAL) {
			if (rtc_freq == 0) {
				dev_ptr->rtc_status = CH_RTC_STATUS_CAL_FACTORY;
			} else {
				dev_ptr->rtc_status = CH_RTC_STATUS_CAL_ESTIMATED;
			}
		} else if (rtc_source == CH_RTC_SRC_EXTERNAL) {
			dev_ptr->rtc_status = CH_RTC_STATUS_EXTERNAL;
		}
	}

	return ret_val;
}


uint8_t	ch_common_get_cal_result(ch_dev_t *dev_ptr, ch_cal_result_t *cal_ptr) {
	uint8_t ret_val = RET_OK;

	if (dev_ptr->part_number == CH101_PART_NUMBER) {
		if (dev_ptr->sensor_connected && (cal_ptr != NULL)) {		
			ret_val |= chdrv_read_word(dev_ptr, CH101_COMMON_REG_DCO_PERIOD, &(cal_ptr->dco_period));
			ret_val |= chdrv_read_word(dev_ptr, CH101_COMMON_REG_REV_CYCLES, &(cal_ptr->rev_cycles));
		}
		else {
			ret_val = RET_ERR;
		}
	}
	else {
		ret_val = RET_ERR;
	}

	return ret_val;
}

uint8_t ch_common_check_program(ch_dev_t *dev_ptr) {
	uint16_t addr;
	uint16_t num_bytes;					// total bytes to read
	uint8_t  num_transfers;
	uint16_t bytes_left;       			// remaining bytes to read
	uint8_t  rx_buf[CH_PROG_XFER_SIZE];
	uint8_t  err = 0;

	num_bytes = dev_ptr->fw_text_size;
	num_transfers = (num_bytes + (CH_PROG_XFER_SIZE - 1)) / CH_PROG_XFER_SIZE;
	bytes_left = num_bytes;

	for (uint8_t xfer_num = 0; !err && (xfer_num < num_transfers); xfer_num++) {
		uint16_t bytes_to_read;

		if (bytes_left > CH_PROG_XFER_SIZE) {
			 bytes_to_read = CH_PROG_XFER_SIZE;
		} else {
			 bytes_to_read = bytes_left;
		}

		/* Read bytes from program memory */
#ifdef INCLUDE_SHASTA_SUPPORT
		addr = ICU_PROG_MEM_ADDR;
		err = chdrv_burst_read(dev_ptr, (addr + (xfer_num * CH_PROG_XFER_SIZE)), rx_buf, bytes_to_read);

#elif defined(INCLUDE_WHITNEY_SUPPORT)
		uint8_t  message[] = {(0x80 | CH_PROG_REG_CTL), 0x09};  // read burst command

		addr = CHX01_PROG_MEM_ADDR;
		chbsp_program_enable(dev_ptr);							// assert PROG line
		chdrv_prog_write(dev_ptr, CH_PROG_REG_ADDR, (addr + (xfer_num * CH_PROG_XFER_SIZE)));
		chdrv_prog_write(dev_ptr, CH_PROG_REG_CNT, (CH_PROG_XFER_SIZE - 1));

		err = chdrv_prog_i2c_write(dev_ptr, message, sizeof(message));
		err |= chdrv_prog_i2c_read(dev_ptr, rx_buf, bytes_to_read);
		chbsp_program_disable(dev_ptr);							// de-assert PROG line
#endif  // INCLUDE_SHASTA_SUPPORT

		bytes_left -= bytes_to_read;							// adjust remaining byte count

		/* Compare read bytes with original firmware image */
		if (!err) {
			for (uint16_t idx = 0; idx < bytes_to_read; idx++) {
				uint8_t *fw_ptr = ((uint8_t *)(dev_ptr->fw_text) + (xfer_num * CH_PROG_XFER_SIZE));

				if (rx_buf[idx] != fw_ptr[idx]) {
					err = 1;
					break;
				}
			}
		}
	}  /* end for (xfer_num < num_transfers) */

#ifdef INCLUDE_WHITNEY_SUPPORT
	if (dev_ptr->group->num_connected[dev_ptr->bus_index] > 1) {		// if more than one device on this I2C bus
		chbsp_delay_ms(10);										// delay to let PROG interface settle
	}
#endif  // INCLUDE_WHITNEY_SUPPORT

	return err;
}

static uint32_t set_new_dco_code(ch_dev_t *dev_ptr, uint16_t dcocode){
	ch_common_set_mode(dev_ptr, CH_MODE_IDLE);
	chdrv_write_word(dev_ptr, CH101_COMMON_REG_DCO_PERIOD, dcocode);
	chdrv_wait_for_lock(dev_ptr, CHDRV_FREQLOCK_TIMEOUT_MS);
	ch_common_set_mode(dev_ptr, CH_MODE_TRIGGERED_TX_RX);
	dev_ptr->store_op_freq(dev_ptr);

	return dev_ptr->op_frequency;
}

uint8_t ch_common_set_frequency(ch_dev_t *dev_ptr, uint32_t target_freq_Hz) {
	uint8_t ret_val = RET_ERR;
	
	if (dev_ptr->part_number == CH101_PART_NUMBER) {
		uint32_t freq = 0;
		uint32_t dcoper[2];
		
		//initially, find two points on the DCO curve, which should be linear in period
		//increasing DCO code ~= increasing DCO period
		dcoper[0] = 1000000000U / set_new_dco_code(dev_ptr, CH101_DCO_LOW);
		dcoper[1] = 1000000000U / set_new_dco_code(dev_ptr, CH101_DCO_HIGH);
		uint32_t targetper = 1000000000U / target_freq_Hz; ///5617
		//Now interpolate to estimate the DCO code
		uint16_t dcoest = (int32_t)CH101_DCO_LOW + ((int32_t) targetper - (int32_t) dcoper[0]) * (int32_t)(CH101_DCO_HIGH - CH101_DCO_LOW) /
		(int32_t)(dcoper[1] - dcoper[0]);
		freq = set_new_dco_code(dev_ptr, dcoest);

#ifdef CHDRV_DEBUG
		char cbuf[80];
		snprintf(cbuf, sizeof(cbuf), "# Port %u, dco0=%lu, dco1=%lu, dcoest=%u, freq=%lu, targ= %lu\n",
		dev_ptr->io_index, dcoper[0], dcoper[1], dcoest, freq, target_freq_Hz);
		chbsp_print_str(cbuf);
#endif
		int32_t minerr = abs((int32_t)freq - (int32_t)target_freq_Hz);
		uint32_t minoff = 0;
		//if the error is too high, search around the estimate for the best code
		if (minerr > CH101_DCO_SEARCH_THRESHOLD) {
#ifdef CHDRV_DEBUG
			snprintf(cbuf, sizeof(cbuf), "# Frequency error above %dHz, searching for better match to %luHz\n",
			CH101_DCO_SEARCH_THRESHOLD, target_freq_Hz);
			chbsp_print_str(cbuf);
#endif
			int i;
			for (i = -5; i < 6; i++) { //+/-5 DCO codes should be about +/-1500Hz
				freq = set_new_dco_code(dev_ptr, dcoest + i);
				if (abs((int32_t)freq - (int32_t)target_freq_Hz) < minerr) {
					minerr = abs((int32_t)freq - (int32_t)target_freq_Hz);
					minoff = i;
#ifdef CHDRV_DEBUG
					printf("# *");
#endif					
				}
#ifdef CHDRV_DEBUG
				snprintf(cbuf, sizeof(cbuf), "# dcoest=%u, freq=%lu\n", dcoest + i, freq);
				chbsp_print_str(cbuf);
#endif
			}
			dcoest = dcoest + minoff;
			freq = set_new_dco_code(dev_ptr, dcoest);
#ifdef CHDRV_DEBUG
			snprintf(cbuf, sizeof(cbuf), "# Final setting dco=%u, freq=%lu\n", dcoest, freq);
			chbsp_print_str(cbuf);
#endif
		}
		ret_val = RET_OK;
	}
	return ret_val;
}


uint32_t ch_common_samples_to_cycles(uint16_t num_samples, ch_odr_t odr) {
	uint32_t num_cycles = 0;

	num_cycles = num_samples << (CH_ODR_DEFAULT + (7 - odr));

#ifdef INCLUDE_WHITNEY_SUPPORT
	num_cycles /= CH101_COMMON_SM_CYCLES_PER_PMUT;		// Whitney uses PMUT cycles, same for CH101 & CH201
#endif

	return num_cycles;
}

uint16_t ch_common_cycles_to_samples(uint32_t num_cycles, ch_odr_t odr) {
	uint32_t num_samples = 0;

#ifdef INCLUDE_WHITNEY_SUPPORT
	num_cycles *= CH101_COMMON_SM_CYCLES_PER_PMUT;		// Whitney uses PMUT cycles, same for CH101 & CH201
#endif

	num_samples = num_cycles >> (CH_ODR_DEFAULT + (7 - odr));

	return num_samples;
}

uint32_t ch_common_usec_to_cycles(ch_dev_t *dev_ptr, uint32_t num_usec) {
	uint32_t num_cycles = 0;
	uint32_t	op_freq = dev_ptr->op_frequency;
	uint32_t	cycles_per_pmut;							// "cycle" units per PMUT cycle

	if (dev_ptr->asic_gen == CH_ASIC_GEN_2_SHASTA) {
		cycles_per_pmut = SHASTA_SMCLK_CYCLES_PER_PMUT;		// Shasta uses SMCLK cycles
	} else {
		cycles_per_pmut = 1;								// Whitney uses PMUT transducer cycles
	}

	num_cycles = (cycles_per_pmut * num_usec * (op_freq / 64)) / (1000000 / 64);

	return num_cycles;
}


uint32_t ch_common_cycles_to_usec(ch_dev_t *dev_ptr, uint32_t num_cycles) {
	uint32_t num_usec = 0;
	uint32_t	op_freq = dev_ptr->op_frequency;

	num_usec = (num_cycles * (1000000 / 64)) /  (op_freq / 64);	// calculate as if PMUT cycles

	if (dev_ptr->asic_gen == CH_ASIC_GEN_2_SHASTA) {			// Shasta uses SMCLK cycles
		num_usec /= SHASTA_SMCLK_CYCLES_PER_PMUT;
	}

	return num_usec;
}



uint32_t ch_common_get_tof_us(ch_dev_t *dev_ptr) {
	uint32_t tof_us = 0;

	tof_us = ch_common_get_target_tof_us(dev_ptr, CH_DEFAULT_TARGET_NUM);

	return tof_us;
}

uint32_t ch_common_get_target_tof_us(ch_dev_t *dev_ptr, uint8_t target_num) {
	uint32_t num_usec = 0;
  
#if (defined(INCLUDE_SHASTA_SUPPORT) && defined(INCLUDE_ALGO_RANGEFINDER))
	if (dev_ptr->sensor_connected) {
		uint8_t err = 0;

		/* Get current target list from sensor */		// XXX TODO optimize reads vs. get_amplitude etc.
		err = read_target_list(dev_ptr);

		if (!err) {
			/* Report specified target from list */
			target_list_t *tgt_list_ptr = (target_list_t *) &(dev_ptr->algo_out.tL);
			uint32_t tof_cycles = (uint32_t) tgt_list_ptr->targets[target_num].range;
			uint16_t pre_rx_cycles = 0;

			/* Adjust ToF for pre-rx cycles in measurement */
			pre_rx_cycles = dev_ptr->meas_pre_rx_cycles[dev_ptr->last_measurement];
			if (pre_rx_cycles > REF_PRE_RX_CYCLES) {
				tof_cycles -= (pre_rx_cycles - REF_PRE_RX_CYCLES);
			}

			num_usec = ch_cycles_to_usec(dev_ptr, tof_cycles);
		}
	}

#else
	// XXX TODO add whitney
	(void)dev_ptr;
	(void)target_num;

#endif  // INCLUDE_SHASTA_SUPPORT	&& 	INCLUDE_ALGO_RANGEFINDER

	return num_usec;
}



#ifdef INCLUDE_SHASTA_SUPPORT

uint8_t ch_common_meas_init_queue(ch_dev_t *dev_ptr) {
	uint8_t err = 0;

	dev_ptr->meas_queue.meas_start = 0;
	dev_ptr->meas_queue.meas_stop = 0;
	dev_ptr->meas_queue.current_meas = 0;

	dev_ptr->meas_queue.intconfig = 0;
	if (dev_ptr->group->sensor_int_pin == 2) {
		dev_ptr->meas_queue.intconfig |= INTCONFIG_DR_INT2;
	}
	if (dev_ptr->tgt_int_filter == CH_TGT_INT_FILTER_ANY) {
		dev_ptr->meas_queue.intconfig |= INTCONFIG_ON_TARGET;
	}

	dev_ptr->meas_queue.trigsrc = 0;
	if (dev_ptr->trig_type == CH_TRIGGER_TYPE_HW) {
		if (dev_ptr->group->sensor_trig_pin == 1) {
			dev_ptr->meas_queue.trigsrc |= TRIGSRC_HWTRIGGER_INT1;
		} else if (dev_ptr->group->sensor_trig_pin == 2) {
			dev_ptr->meas_queue.trigsrc |= TRIGSRC_HWTRIGGER_INT2;
		}
	}

	for (uint8_t meas_num = 0; meas_num < MEAS_QUEUE_MAX_MEAS; meas_num++) {
		err |= ch_common_meas_reset(dev_ptr, meas_num);
		if (err) {
			break;
		}
	}

	return err;
}

uint8_t ch_common_meas_import(ch_dev_t *dev_ptr, measurement_queue_t *meas_queue_ptr, void *algo_cfg_ptr) {
	uint8_t needs_update = 0;	// set if sensor needs update
	uint8_t err = 0;			// default is error return

	if (meas_queue_ptr != NULL) {
		/* Copy queue definition */
		memcpy((void *)&(dev_ptr->meas_queue), (void *)meas_queue_ptr, sizeof(measurement_queue_t));

		/* Set counts in dev descriptor for each measurement in queue */
		for (uint8_t meas_num = 0; meas_num < MEAS_QUEUE_MAX_MEAS; meas_num++) {
			measurement_t *meas_ptr = (measurement_t *) &(dev_ptr->meas_queue.meas[meas_num]);

			ch_common_meas_update_counts(dev_ptr, meas_num, meas_ptr);
		}

		/* Update interrupt and trigger pin settings */
		dev_ptr->tgt_int_filter             = (meas_queue_ptr->intconfig & INTCONFIG_ON_TARGET ? CH_TGT_INT_FILTER_ANY : CH_TGT_INT_FILTER_OFF);
		dev_ptr->group->sensor_int_pin  = (meas_queue_ptr->intconfig & INTCONFIG_DR_INT2 ? 2 : 1);
		dev_ptr->group->sensor_trig_pin = (meas_queue_ptr->trigsrc == TRIGSRC_HWTRIGGER_INT2 ? 2 : 1);

		/* Update measurement interval if self-timed (freerunning) */
		if (meas_queue_ptr->trigsrc == TRIGSRC_RTC) {
			uint16_t meas_period = meas_queue_ptr->meas[CH_DEFAULT_MEAS_NUM].meas_period;
			dev_ptr->sample_interval = (uint32_t) meas_period * dev_ptr->group->rtc_cal_pulse_ms / dev_ptr->rtc_cal_result;
		}
		needs_update = 1;
	}

	if (algo_cfg_ptr != NULL) {
#ifdef USE_INTERNAL_ALGO
		/* Copy algo configuration to device descriptor, will be written to sensor later */
		memcpy((void *)&(dev_ptr->algo_cfg), (void *)algo_cfg_ptr, sizeof(ICU_ALGO_SHASTA_CONFIG));
		needs_update = 1;
#else
		/* Algo can't be stored in device descriptor, write measurement algorithm configuration directly to sensor */
		uint16_t algo_cfg_addr = (uint16_t)(uintptr_t) dev_ptr->algo_info.algo_cfg_ptr;
																	// destination address on sensor
		uint16_t algo_cfg_nbytes = dev_ptr->algo_info.algo_cfg_len;

		err = chdrv_burst_write(dev_ptr, algo_cfg_addr, (uint8_t *) algo_cfg_ptr, algo_cfg_nbytes);
#endif  // USE_INTERNAL_ALGO
	}

	/* Update sensor */
	if (!err && needs_update) {
		err = ch_common_meas_write_config(dev_ptr);
	}
		
	return err;
}


uint8_t ch_common_meas_reset(ch_dev_t *dev_ptr, uint8_t meas_num) {
	measurement_t *meas_ptr = (measurement_t *)&(dev_ptr->meas_queue.meas[meas_num]);
#ifdef INCLUDE_ALGO_RANGEFINDER
	InvnAlgoRangeFinderMeasConfig *meas_cfg_ptr = (InvnAlgoRangeFinderMeasConfig *) &(dev_ptr->algo_cfg.meas_cfg[meas_num]);
#endif
	uint8_t ret_val = 0;

	meas_ptr->meas_period = 0;
	meas_ptr->odr = CH_ODR_DEFAULT;

	// Init transceiver instructions
	for (int i = 0; i < INST_BUF_LENGTH; i++) {
		meas_ptr->trx_inst[i].cmd_config = 0;
		meas_ptr->trx_inst[i].length = 0;
	}

	meas_ptr->trx_inst[0].cmd_config = PMUT_CMD_EOF;

#ifdef INCLUDE_ALGO_RANGEFINDER
	// Init meas config
	meas_cfg_ptr->num_ranges = 0;
	meas_cfg_ptr->ringdown_cancel_samples = 0;
	meas_cfg_ptr->static_target_samples = 0;
	meas_cfg_ptr->iq_output_format = 0;
	meas_cfg_ptr->filter_update_modulo = 0;

	// Init thresholds
	for (int i = 0; i < LEN_THRESH; i++) {
		meas_cfg_ptr->thresholds.stop_index[i] = 0;
		meas_cfg_ptr->thresholds.threshold[i] = 0;
	}
#endif

	// Init counters
	dev_ptr->meas_num_segments[meas_num] = 0;
	dev_ptr->meas_num_rx_samples[meas_num] = 0;

	return ret_val;
}


uint8_t ch_common_meas_init(ch_dev_t *dev_ptr, uint8_t meas_num, ch_meas_config_t *meas_config_ptr, ch_thresholds_t *lib_thresh_ptr) {

	measurement_t *meas_ptr = (measurement_t *)&(dev_ptr->meas_queue.meas[meas_num]);
#ifdef INCLUDE_ALGO_RANGEFINDER
	InvnAlgoRangeFinderMeasConfig *meas_cfg_ptr = (InvnAlgoRangeFinderMeasConfig *) &(dev_ptr->algo_cfg.meas_cfg[meas_num]);
	thresholds_t * sens_thresh_ptr = (thresholds_t *) &(meas_cfg_ptr->thresholds);        // thresholds converted to sensor format
#else
	(void)lib_thresh_ptr;
#endif
	uint8_t ret_val = 0;

	meas_ptr->meas_period 				= meas_config_ptr->meas_period;		// XXX TODO do some validation
	meas_ptr->odr 						= (uint8_t) meas_config_ptr->odr;

#ifdef INCLUDE_ALGO_RANGEFINDER
	meas_cfg_ptr->ringdown_cancel_samples 	= meas_config_ptr->ringdown_cancel_samples;	// XXX use zero for default ??
	meas_cfg_ptr->static_target_samples 	= meas_config_ptr->static_filter_samples;
	meas_cfg_ptr->num_ranges 				= meas_config_ptr->num_ranges;
	meas_cfg_ptr->iq_output_format		 	= meas_config_ptr->iq_output_format;
	meas_cfg_ptr->filter_update_modulo 		= meas_config_ptr->filter_update_interval;

	if (lib_thresh_ptr == NULL) {
		lib_thresh_ptr = &default_thresholds;
	}

	thresh_lib_to_sensor(dev_ptr, lib_thresh_ptr, sens_thresh_ptr);		// convert format and copy to meas config
#endif

	return ret_val;
}


uint8_t ch_common_meas_add_segment(ch_dev_t *dev_ptr, uint8_t meas_num, ch_meas_segment_t *seg_ptr) {
	measurement_t *meas_ptr = (measurement_t *)&(dev_ptr->meas_queue.meas[meas_num]);
	uint8_t	seg_num = dev_ptr->meas_num_segments[meas_num];
	uint16_t num_samples;
	uint8_t	err = 0;

	if (seg_num >= INST_BUF_LENGTH) {
		err = 1;
	}

	if (!err) {
		// Copy transceiver instruction 
		meas_ptr->trx_inst[seg_num].cmd_config = seg_ptr->inst.cmd_config;
		meas_ptr->trx_inst[seg_num].length = seg_ptr->inst.length;

		// Mark end of instruction sequence, if any remaining slots
		seg_num++;
		if (seg_num < INST_BUF_LENGTH) {
			meas_ptr->trx_inst[seg_num].cmd_config = PMUT_CMD_EOF;
		}

		dev_ptr->meas_num_segments[meas_num]++;		// count one more active segment in this meas

		num_samples = ch_common_cycles_to_samples(seg_ptr->inst.length, meas_ptr->odr);
		dev_ptr->meas_num_rx_samples[meas_num] += num_samples;
	}

	return err;
}


uint8_t ch_common_meas_add_segment_count(ch_dev_t *dev_ptr, uint8_t meas_num, uint16_t num_cycles, uint8_t int_enable) {
	ch_meas_segment_t meas_segment;
	uint8_t	err = 0;

	ch_common_meas_init_segment_count(&meas_segment, num_cycles, int_enable);
	err = ch_meas_add_segment(dev_ptr, meas_num, &meas_segment);

	return err;
}


uint8_t ch_common_meas_add_segment_rx(ch_dev_t *dev_ptr, uint8_t meas_num, uint16_t num_samples, uint8_t gain, uint8_t atten, uint8_t int_enable) {
	ch_meas_segment_t meas_segment;
	ch_odr_t odr = (ch_odr_t) dev_ptr->meas_queue.meas[meas_num].odr;
	uint8_t	err = 0;

	ch_common_meas_init_segment_rx(&meas_segment, num_samples, odr, gain, atten, int_enable);
	err = ch_meas_add_segment(dev_ptr, meas_num, &meas_segment);

	return err;
}


uint8_t ch_common_meas_add_segment_tx(ch_dev_t *dev_ptr, uint8_t meas_num, uint16_t num_cycles, uint8_t pulse_width, uint8_t phase,
										uint8_t int_enable) {
	ch_meas_segment_t meas_segment;
	uint8_t	err = 0;

	ch_common_meas_init_segment_tx(&meas_segment, num_cycles, pulse_width, phase, int_enable);
	err = ch_meas_add_segment(dev_ptr, meas_num, &meas_segment);

	return err;
}


void ch_common_meas_init_segment_count(ch_meas_segment_t *seg_ptr, uint16_t num_cycles, uint8_t int_enable) {
	uint16_t cmd_config = PMUT_CMD_COUNT;

	if (int_enable) {
		cmd_config |= PMUT_DONE_IEN_BITS;									// enable interrupt when done 
	}

	seg_ptr->inst.cmd_config = cmd_config;
	seg_ptr->inst.length = num_cycles;
}


void ch_common_meas_init_segment_rx(ch_meas_segment_t *seg_ptr, uint16_t num_samples, ch_odr_t odr, uint8_t gain, 
									uint8_t atten, uint8_t int_enable) {

	uint16_t cmd_config = (PMUT_CMD_RX | PMUT_RX_BIAS_EN_BITS);
	uint32_t num_cycles = ch_common_samples_to_cycles(num_samples, odr);

	if (num_cycles > UINT16_MAX) {						
		num_cycles = UINT16_MAX;
	}

	if (int_enable) {
		cmd_config |= PMUT_DONE_IEN_BITS;									// enable interrupt when done 
	}

	cmd_config |= ((gain & PMUT_RXGAIN_BM) << PMUT_RXGAIN_RED_BITSHIFT);		// add gain bits
	cmd_config |= ((atten & PMUT_RXATTEN_BM) << PMUT_RXATTEN_BITSHIFT);		// add attenuation bits

	seg_ptr->inst.cmd_config = cmd_config;
	seg_ptr->inst.length = (uint16_t) num_cycles;
}


void ch_common_meas_init_segment_tx(ch_meas_segment_t *seg_ptr, uint16_t num_cycles, uint8_t pulse_width,
									uint8_t phase, uint8_t int_enable) {

	uint16_t cmd_config = (PMUT_CMD_TX | PMUT_RX_BIAS_EN_BITS);

	if (int_enable) {
		cmd_config |= PMUT_DONE_IEN_BITS;									// enable interrupt when done 
	}

	cmd_config |= ((pulse_width & PMUT_TX_PW_BM) << PMUT_TX_PW_BITSHIFT);	// add pulse_width bits
	cmd_config |= ((phase & PMUT_TX_PHASE_BM) << PMUT_TX_PHASE_BITSHIFT);	// add phase bits

	seg_ptr->inst.cmd_config = cmd_config;
	seg_ptr->inst.length = num_cycles;
}


uint8_t ch_common_meas_write_config(ch_dev_t *dev_ptr) {
	uint8_t err = 0;

	/* Write measurement queue values */
	err = chdrv_meas_queue_write(dev_ptr, NULL);

#ifdef USE_INTERNAL_ALGO
	/* Write measurement algorithm configuration */
	if (!err) {
		uint8_t *algo_cfg_data = (uint8_t *) &(dev_ptr->algo_cfg);		// source copy in ch_dev_t
		uint16_t algo_cfg_addr = (uint16_t)(uintptr_t) dev_ptr->sens_algo_cfg_addr;
																		// destination address on sensor
		uint16_t algo_cfg_nbytes = (uint16_t)(uintptr_t) dev_ptr->algo_info.algo_cfg_len;

		err = chdrv_burst_write(dev_ptr, algo_cfg_addr, algo_cfg_data, algo_cfg_nbytes);
	}
#endif	// USE_INTERNAL_ALGO

	/* Send event to apply settings (especially ODR) immediately */
	if (!err) {
		chdrv_event_trigger(dev_ptr, EVENT_PREPARE_MEAS);
	}

	/* Update measurement counts */
	if (!err) {
		for (uint8_t meas_num = 0; meas_num < MEAS_QUEUE_MAX_MEAS; meas_num++) {
			measurement_t *meas_ptr = (measurement_t *) &(dev_ptr->meas_queue.meas[meas_num]);

			err |= ch_common_meas_update_counts(dev_ptr, meas_num, meas_ptr);
		}
	}

	return  err;
}

uint8_t ch_common_meas_optimize(ch_dev_t *dev_ptr, measurement_queue_t *meas_queue_ptr, void *algo_cfg_ptr) {
	measurement_queue_t  tx_opt_queue;			// meas queue after optimization
	ch_group_t *grp_ptr = dev_ptr->group;
	uint8_t ret_val = RET_ERR;

	if (grp_ptr->status == CH_GROUP_STAT_INIT_OK) {

		/* Import new meas queue and/or algo config if specified */
		if ((meas_queue_ptr != NULL) || (meas_queue_ptr != NULL)) {
			ret_val = ch_common_meas_import(dev_ptr, meas_queue_ptr, algo_cfg_ptr);
		} else {
			ret_val = RET_OK;
		}

		/* Load init f/w to perform optimization */
		if (ret_val == RET_OK) {
			grp_ptr->status = CH_GROUP_STAT_INIT_PENDING;		// re-init now pending

			dev_ptr->fw_reinit_func = dev_ptr->fw_init_func;	// save original init routine to call again later

			dev_ptr->fw_init_func = CH_STD_INIT_FUNC;
			ret_val = (dev_ptr->fw_init_func)(dev_ptr, grp_ptr, 0, dev_ptr->io_index, 0);
		}

		if (ret_val == RET_OK) {
			ret_val =  chdrv_restart(dev_ptr);
		}

		/* Write measurement queue values */
		if (ret_val == RET_OK) {
			ret_val = chdrv_meas_queue_write(dev_ptr, NULL);
		}

		/* Trigger Tx optimize event on sensor */
		if (ret_val == RET_OK) {
			ret_val = chdrv_event_trigger_and_wait(dev_ptr, EVENT_TX_OPTIMIZE);
		}

		/* Read modified message queue */
		if (ret_val == RET_OK) {
			ret_val = chdrv_meas_queue_read(dev_ptr, &tx_opt_queue);
		}

		/* Extract optimized tx instructions and combine with existing meas queue segments */
		if (ret_val == RET_OK) {
			for (uint8_t meas_num = 0; meas_num < MEAS_QUEUE_MAX_MEAS; meas_num++) {
				uint16_t num_opt_tx_inst = 0;	// number of tx instructions after optimization
				uint16_t num_added;				// number of tx instructions added by opt

				for (uint8_t i = 0; i < INST_BUF_LENGTH; i++) {
					pmut_transceiver_inst_t pmut_inst = tx_opt_queue.meas[meas_num].trx_inst[i];

					uint16_t pmut_cmd = (uint16_t)((pmut_inst.cmd_config >> PMUT_CMD_BITSHIFT) & PMUT_CMD_BITS);
					if (pmut_cmd == PMUT_CMD_TX) {
						num_opt_tx_inst++;				// count this tx instruction
					}
				}

				num_added = num_opt_tx_inst - dev_ptr->meas_num_tx_segments[meas_num];

				/* Shuffle old non-tx entries in queue to insert new set of tx instructions */
				for (uint8_t idx = INST_BUF_LENGTH-1; (idx - num_added) > 0; idx--) {
					pmut_transceiver_inst_t *inst_ptr = (pmut_transceiver_inst_t *) &(dev_ptr->meas_queue.meas[meas_num].trx_inst);

					inst_ptr[idx] = inst_ptr[idx - num_added];
				}

				/* Copy optimized tx instructions into meas queue */
				for (uint8_t idx = 0; idx < num_opt_tx_inst; idx++) {
					pmut_transceiver_inst_t *inst_ptr = (pmut_transceiver_inst_t *) &(dev_ptr->meas_queue.meas[meas_num].trx_inst);
					pmut_transceiver_inst_t *opt_inst_ptr = (pmut_transceiver_inst_t *) &(tx_opt_queue.meas[meas_num].trx_inst);

					inst_ptr[idx] = opt_inst_ptr[idx];
				}
			}
		} // end for (meas_num < MEAS_QUEUE_MAX_MEAS)

		/* Reload measurement f/w */
		if (ret_val == RET_OK) {
			dev_ptr->fw_init_func = dev_ptr->fw_reinit_func;
			ret_val = (dev_ptr->fw_init_func)(dev_ptr, grp_ptr, 0, dev_ptr->io_index, 0);
		}

		if (ret_val == RET_OK) {
			ret_val =  chdrv_restart(dev_ptr);
		}

		/* Write new meas queue definition */
		if (ret_val == RET_OK) {
			grp_ptr->status = CH_GROUP_STAT_INIT_OK;				// indicate sensor (re-)init'd

			/* Write meas queue to device */
			ret_val = ch_common_meas_write_config(dev_ptr);

			/* Apply trigger pin selection */
			ret_val |= chdrv_event_trigger(dev_ptr, EVENT_CONFIG_TRIGGER);
		}

	}  // end if (grp_ptr->status == CH_GROUP_STAT_INIT_OK)

	return ret_val;
}


void ch_common_meas_get_info(ch_dev_t *dev_ptr, uint8_t meas_num, ch_meas_info_t *info_ptr) {
	measurement_t *meas_ptr = (measurement_t *)&(dev_ptr->meas_queue.meas[meas_num]);
#ifdef INCLUDE_ALGO_RANGEFINDER
	InvnAlgoRangeFinderMeasConfig *meas_cfg_ptr = (InvnAlgoRangeFinderMeasConfig *) &(dev_ptr->algo_cfg.meas_cfg[meas_num]);
#endif

	memset((void *)info_ptr, 0, sizeof(ch_meas_info_t));	// init info struct

	info_ptr->num_segments    = dev_ptr->meas_num_segments[meas_num];		// values maintained by SonicLib
	info_ptr->num_tx_segments 	  = dev_ptr->meas_num_tx_segments[meas_num];
	info_ptr->num_rx_samples  = dev_ptr->meas_num_rx_samples[meas_num];
	info_ptr->num_cycles      = dev_ptr->meas_num_cycles[meas_num];

	info_ptr->meas_period 				= meas_ptr->meas_period;			// copies of values on sensor
	info_ptr->odr 						= (ch_odr_t) meas_ptr->odr;
#ifdef INCLUDE_ALGO_RANGEFINDER
	info_ptr->num_ranges 				= meas_cfg_ptr->num_ranges;
	info_ptr->ringdown_cancel_samples 	= meas_cfg_ptr->ringdown_cancel_samples;
	info_ptr->static_filter_samples 	= meas_cfg_ptr->static_target_samples;
	info_ptr->iq_output_format 			= meas_cfg_ptr->iq_output_format;
	info_ptr->filter_update_interval	= meas_cfg_ptr->filter_update_modulo;
#endif
}

void ch_common_meas_get_seg_info(ch_dev_t *dev_ptr, uint8_t meas_num, uint8_t seg_num, ch_meas_seg_info_t *info_ptr) {
	pmut_transceiver_inst_t  *inst_ptr = (pmut_transceiver_inst_t *)&(dev_ptr->meas_queue.meas[meas_num].trx_inst[seg_num]);
	ch_odr_t odr = (ch_odr_t) dev_ptr->meas_queue.meas[meas_num].odr;
	uint16_t cmd_config = inst_ptr->cmd_config;
	ch_meas_seg_type_t seg_type;

	memset((void *)info_ptr, 0, sizeof(ch_meas_seg_info_t));	// init info struct

	seg_type = (ch_meas_seg_type_t) ((cmd_config >> PMUT_CMD_BITSHIFT) & PMUT_CMD_BITS);	// same values as ch_meas_seg_type_t

	info_ptr->type = seg_type;
	info_ptr->num_cycles = inst_ptr->length;

	info_ptr->rdy_int_en  = ((cmd_config & PMUT_RDY_IEN_BITS) != 0);
	info_ptr->done_int_en = ((cmd_config & PMUT_DONE_IEN_BITS) != 0);

	if (seg_type == CH_MEAS_SEG_TYPE_TX) {
		info_ptr->tx_phase       = ((cmd_config >> PMUT_TX_PHASE_BITSHIFT) & PMUT_TX_PHASE_BM);
		info_ptr->tx_pulse_width = ((cmd_config >> PMUT_TX_PW_BITSHIFT) & PMUT_TX_PW_BM);
	} else if (seg_type == CH_MEAS_SEG_TYPE_RX) {
		info_ptr->num_rx_samples = ch_common_cycles_to_samples(info_ptr->num_cycles, odr);
		info_ptr->rx_gain  = ((cmd_config >> PMUT_RXGAIN_RED_BITSHIFT) & PMUT_RXGAIN_BM);
		info_ptr->rx_atten = ((cmd_config >> PMUT_RXATTEN_BITSHIFT) & PMUT_RXATTEN_BM);
	}

}


void ch_common_meas_get_queue_info(ch_dev_t *dev_ptr, ch_meas_queue_info_t *info_ptr) {
	measurement_queue_t  *meas_q_ptr = &(dev_ptr->meas_queue);

	memset((void *)info_ptr, 0, sizeof(ch_meas_queue_info_t));	// init info struct

	info_ptr->int_config 	= meas_q_ptr->intconfig;
	info_ptr->meas_start 	= meas_q_ptr->meas_start;
	info_ptr->meas_stop 	= meas_q_ptr->meas_stop;
	info_ptr->current_meas 	= meas_q_ptr->current_meas;
	info_ptr->trig_src 		= meas_q_ptr->trigsrc;
}

uint8_t ch_common_meas_update_counts(ch_dev_t *dev_ptr, uint8_t meas_num, measurement_t *meas_ptr) {
	pmut_transceiver_inst_t *trx_inst_ptr = (pmut_transceiver_inst_t *) &(meas_ptr->trx_inst[0]); // first instruction in meas
	uint16_t total_cycles = 0;
	uint16_t total_rx_samples = 0;
	uint8_t  total_segments = 0;
	uint16_t pre_rx_cycles = 0;
	uint8_t num_rx_segments = 0;
	uint8_t err = 0;
	uint16_t total_tx_segments = 0;		// # of tx instructions

	dev_ptr->meas_num_segments[meas_num] = 0;
	dev_ptr->meas_num_rx_samples[meas_num] = 0;

	/* Examine instructions until EOF marker */
	while (trx_inst_ptr->cmd_config != PMUT_CMD_EOF) {
		uint16_t  num_samples = 0;		// samples in this segment
		uint16_t  num_inst = 0;			// # of instructions
		ch_meas_seg_type_t seg_type = (ch_meas_seg_type_t) ((trx_inst_ptr->cmd_config >> PMUT_CMD_BITSHIFT) & PMUT_CMD_BITS);
										// use instruction as segment type
		if (num_inst++ >= INST_BUF_LENGTH) {
			err = 1;		// error - no EOF marker before end of instructions
			break;
		}

		total_cycles += trx_inst_ptr->length;

		total_segments++;							// each instruction = 1 segment

		if (seg_type == CH_MEAS_SEG_TYPE_RX) {
    		num_samples = ch_common_cycles_to_samples(trx_inst_ptr->length, meas_ptr->odr);
    		total_rx_samples += num_samples;
			num_rx_segments++;
        } else {
			if (num_rx_segments == 0) {
				pre_rx_cycles += trx_inst_ptr->length;
			}
			if (seg_type == CH_MEAS_SEG_TYPE_TX) {
				total_tx_segments++;
			}
		}

		trx_inst_ptr++;					// next instruction

	}	// end while (trx_inst_ptr->cmd_config != PMUT_CMD_EOF)

	if (!err) {
		dev_ptr->meas_num_cycles[meas_num]     = total_cycles;
		dev_ptr->meas_num_tx_segments[meas_num]= total_tx_segments;
		dev_ptr->meas_num_rx_samples[meas_num] = total_rx_samples;
		dev_ptr->meas_num_segments[meas_num]   = total_segments;
		dev_ptr->meas_pre_rx_cycles[meas_num]  = pre_rx_cycles;
		
		/* Fill in ch_dev_t fields for compatibility with simple SonicLib v2 API calls (use default meas) */
		if (meas_num == CH_DEFAULT_MEAS_NUM) {
			dev_ptr->num_rx_samples = total_rx_samples;
			dev_ptr->max_range = ch_samples_to_mm(dev_ptr, total_rx_samples);	// store corresponding range in mm
		}
	}
	
	return err;
}



uint8_t ch_common_meas_set_interval(ch_dev_t *dev_ptr, uint8_t meas_num, uint16_t interval_ms) {
	measurement_t  *meas_ptr = (measurement_t *) &(dev_ptr->meas_queue.meas[meas_num]);
	uint32_t	num_rtc_periods;
	uint8_t	ret_val = RET_ERR;

	num_rtc_periods = ((uint32_t)interval_ms * dev_ptr->rtc_cal_result) / dev_ptr->group->rtc_cal_pulse_ms;

	meas_ptr->meas_period = num_rtc_periods;

	ret_val = ch_common_meas_write_config(dev_ptr);

	if ((ret_val == RET_OK) && (meas_num == 0)) {
		dev_ptr->sample_interval = interval_ms;			// measurement 0 gets interval in dev descriptor
	}

	return ret_val;
}
	
uint16_t ch_common_meas_get_interval(ch_dev_t *dev_ptr, uint8_t meas_num) {
	uint32_t num_cycles =   (uint32_t) dev_ptr->meas_queue.meas[meas_num].meas_period;
	uint16_t num_ms;

	num_ms = (uint16_t) ((num_cycles * dev_ptr->group->rtc_cal_pulse_ms) / dev_ptr->rtc_cal_result);

	return  num_ms;
}
	

uint8_t ch_common_meas_set_odr(ch_dev_t *dev_ptr, uint8_t meas_num, ch_odr_t odr) {
	measurement_t  *meas_ptr = (measurement_t *) &(dev_ptr->meas_queue.meas[meas_num]);
#ifdef INCLUDE_ALGO_RANGEFINDER
	InvnAlgoRangeFinderMeasConfig *meas_cfg_ptr = (InvnAlgoRangeFinderMeasConfig *) &(dev_ptr->algo_cfg.meas_cfg[meas_num]);
	ch_odr_t old_odr = (ch_odr_t) meas_ptr->odr;		// save previous ODR setting
#endif
	uint8_t	err = RET_ERR;

	meas_ptr->odr = (uint8_t) odr;						// set new ODR

#ifdef INCLUDE_ALGO_RANGEFINDER
	/* Adjust ringdown and static filter sample counts */
	if (odr < old_odr) {										// if lower ODR
		meas_cfg_ptr->ringdown_cancel_samples >>= (old_odr - odr);	//   fewer samples for same time/distance
		meas_cfg_ptr->static_target_samples >>= (old_odr - odr);
	} else if (odr > old_odr) {									// if higher ODR
		meas_cfg_ptr->ringdown_cancel_samples <<= (odr - old_odr);	//   more samples
		meas_cfg_ptr->static_target_samples <<= (odr - old_odr);
	}

	/* Adjust detection threshold sample counts */
	for (uint8_t thresh_num = 0; thresh_num < dev_ptr->max_num_thresholds; thresh_num++) {
		thresholds_t *thresh_ptr = (thresholds_t *)&(meas_cfg_ptr->thresholds);
		if (odr < old_odr) {										// if lower ODR
			thresh_ptr->stop_index[thresh_num] >>= (old_odr - odr);	//   fewer samples for same time/distance
		} else if (odr > old_odr) {									// if higher ODR
			thresh_ptr->stop_index[thresh_num] <<= (odr - old_odr);	//   more samples
		}
	}
#endif

	err = ch_common_meas_write_config(dev_ptr);								// write to sensor

	return err;
}
	
	
ch_odr_t ch_common_meas_get_odr(ch_dev_t *dev_ptr, uint8_t meas_num) {

	return  dev_ptr->meas_queue.meas[meas_num].odr;
}
	

#ifdef INCLUDE_ALGO_RANGEFINDER

uint8_t ch_common_meas_set_num_ranges(ch_dev_t *dev_ptr, uint8_t meas_num, uint8_t num_ranges) {
	uint8_t	ret_val = RET_ERR;

	dev_ptr->algo_cfg.meas_cfg[meas_num].num_ranges = num_ranges;

	ret_val = ch_common_meas_write_config(dev_ptr);

	return ret_val;
}
	
uint8_t ch_common_meas_get_num_ranges(ch_dev_t *dev_ptr, uint8_t meas_num) {

	return  dev_ptr->algo_cfg.meas_cfg[meas_num].num_ranges;
}
	

uint8_t ch_common_meas_set_ringdown_cancel(ch_dev_t *dev_ptr, uint8_t meas_num, uint16_t num_samples) {
	uint8_t	ret_val = RET_ERR;

	dev_ptr->algo_cfg.meas_cfg[meas_num].ringdown_cancel_samples = num_samples;

	ret_val = ch_common_meas_write_config(dev_ptr);

	return ret_val;
}
	
uint16_t ch_common_meas_get_ringdown_cancel(ch_dev_t *dev_ptr, uint8_t meas_num) {

	return  dev_ptr->algo_cfg.meas_cfg[meas_num].ringdown_cancel_samples;
}
	

uint8_t ch_common_meas_set_static_filter(ch_dev_t *dev_ptr, uint8_t meas_num, uint16_t num_samples) {
	uint8_t	ret_val = RET_ERR;

	dev_ptr->algo_cfg.meas_cfg[meas_num].static_target_samples = num_samples;

	ret_val = ch_common_meas_write_config(dev_ptr);

	return ret_val;
}
	
uint16_t ch_common_meas_get_static_filter(ch_dev_t *dev_ptr, uint8_t meas_num) {

	return  dev_ptr->algo_cfg.meas_cfg[meas_num].static_target_samples;
}
	



uint8_t ch_common_meas_set_iq_output(ch_dev_t *dev_ptr, uint8_t meas_num, uint8_t output_format) {
	uint8_t	ret_val = RET_ERR;

	dev_ptr->algo_cfg.meas_cfg[meas_num].iq_output_format = output_format;

	ret_val = ch_common_meas_write_config(dev_ptr);

	return ret_val;
}
	
uint8_t ch_common_meas_get_iq_output(ch_dev_t *dev_ptr, uint8_t meas_num) {

	return  dev_ptr->algo_cfg.meas_cfg[meas_num].iq_output_format;
}
	

uint8_t ch_common_meas_set_filter_update(ch_dev_t *dev_ptr, uint8_t meas_num, uint8_t update_interval) {
	uint8_t	ret_val = RET_ERR;

	dev_ptr->algo_cfg.meas_cfg[meas_num].filter_update_modulo = update_interval;

	ret_val = ch_common_meas_write_config(dev_ptr);

	return ret_val;
}
	
uint8_t ch_common_meas_get_filter_update(ch_dev_t *dev_ptr, uint8_t meas_num) {

	return  dev_ptr->algo_cfg.meas_cfg[meas_num].filter_update_modulo;
}

static uint8_t read_target_list(ch_dev_t *dev_ptr) {
	uint8_t	err = 0;

	/* Get current sensor algo output (includes target list) */
	err = chdrv_algo_out_read(dev_ptr, (void *) &(dev_ptr->algo_out));		// copy algo output to ch_dev_t

	if (!err) {
		uint8_t  last_measurement = 0;
		uint8_t  odr_out = 0;
		uint8_t  iq_output_format = 0;
		uint16_t num_iq_bytes = 0;
		uint16_t last_measurement_reg = (uint16_t)(uintptr_t)&((dev_ptr->sens_cfg_addr)->raw.last_measurement);
		uint16_t odr_out_reg 		  = (uint16_t)(uintptr_t)&((dev_ptr->sens_cfg_addr)->raw.odr_out);
		uint16_t iq_output_format_reg = (uint16_t)(uintptr_t)&((dev_ptr->sens_cfg_addr)->raw.iq_output_format);
		uint16_t num_iq_bytes_reg 	  = (uint16_t)(uintptr_t)&((dev_ptr->sens_cfg_addr)->raw.num_iq_bytes);

		err  = chdrv_read_byte(dev_ptr, last_measurement_reg, &last_measurement);
		err |= chdrv_read_byte(dev_ptr, odr_out_reg, &odr_out);
		err |= chdrv_read_byte(dev_ptr, iq_output_format_reg, &iq_output_format);
		err |= chdrv_read_word(dev_ptr, num_iq_bytes_reg, &num_iq_bytes);

		if (!err) {
			dev_ptr->last_measurement 	= last_measurement;
			dev_ptr->odr_out 			= odr_out;
			dev_ptr->iq_output_format 	= iq_output_format;
			dev_ptr->num_iq_bytes 		= num_iq_bytes;
		}
	}

	return err;
}
	
#endif	// INCLUDE_ALGO_RANGEFINDER

uint8_t ch_common_set_target_interrupt(ch_dev_t *dev_ptr, ch_tgt_int_filter_t tgt_filter_mode) {
	int	ret_val = RET_OK;

	if (tgt_filter_mode == CH_TGT_INT_FILTER_ANY) {
		dev_ptr->meas_queue.intconfig |= INTCONFIG_ON_TARGET;
	} else if (tgt_filter_mode == CH_TGT_INT_FILTER_OFF) {
		dev_ptr->meas_queue.intconfig &= !INTCONFIG_ON_TARGET;
	} else {
		ret_val = RET_ERR;							// no Shasta target int counter support yet
	}

	if (ret_val == RET_OK) {
		dev_ptr->tgt_int_filter = tgt_filter_mode;			// save tgt_int_mode state
		ret_val = ch_common_meas_write_config(dev_ptr);
	}

	return ret_val;
}


ch_tgt_int_filter_t ch_common_get_target_interrupt(ch_dev_t *dev_ptr) {

	return  dev_ptr->tgt_int_filter;
}


uint8_t ch_common_set_interrupt_mode(ch_dev_t *dev_ptr, ch_interrupt_mode_t mode) {
	int	ret_val = RET_ERR;

	if (dev_ptr->asic_gen == CH_ASIC_GEN_2_SHASTA) {
		if (mode == CH_INTERRUPT_MODE_PULSE) {
			dev_ptr->meas_queue.intconfig |= INTCONFIG_PULSE_INT;
		} else {
			dev_ptr->meas_queue.intconfig &= ~INTCONFIG_PULSE_INT;
		}

		/* Write to sensor */
		ret_val = ch_common_meas_write_config(dev_ptr);
		if (ret_val == RET_OK) {
			chdrv_event_trigger(dev_ptr, EVENT_CONFIG_TRIGGER);
		}
	} else {									// CH_ASIC_GEN_1_WHITNEY (CH101 & CH201)
		if (mode == CH_INTERRUPT_MODE_PULSE) {	// ok because always enabled
			ret_val = RET_OK;
		}
	}

	if (ret_val != RET_ERR) {
		dev_ptr->int_mode = mode;				// save mode state
	}

	return ret_val;
}


ch_interrupt_mode_t ch_common_get_interrupt_mode(ch_dev_t *dev_ptr) {

	return  dev_ptr->int_mode;
}



uint8_t ch_common_set_data_output(ch_dev_t *dev_ptr, ch_output_t *output_ptr) {
	uint8_t	ret_val = RET_OK;
#ifdef INCLUDE_ALGO_RANGEFINDER
	uint8_t meas_num = CH_DEFAULT_MEAS_NUM;

	/* Set data output type */
	if (output_ptr->output_type	== CH_OUTPUT_AMP) {
		dev_ptr->algo_cfg.meas_cfg[meas_num].iq_output_format = IQ_OUTPUT_MAG;
	} else {					// CH_OUTPUT_IQ
		dev_ptr->algo_cfg.meas_cfg[meas_num].iq_output_format = IQ_OUTPUT_NORMAL;
	}

	/* Convert "decimation" to modified output data rate (ODR) */
	if (output_ptr->decimation_factor == CH_DECIMATION_NONE) {
		ch_common_meas_set_odr(dev_ptr, meas_num, CH_ODR_DEFAULT);
	} else if (output_ptr->decimation_factor == CH_DECIMATION_2) {
		ch_common_meas_set_odr(dev_ptr, meas_num, (CH_ODR_DEFAULT - 1));
	} else if (output_ptr->decimation_factor == CH_DECIMATION_4) {
		ch_common_meas_set_odr(dev_ptr, meas_num, (CH_ODR_DEFAULT - 2));
	} else {												// note: CH_DECIMATION_3 is not supported
		ret_val = RET_ERR;
	}

#else
	(void)dev_ptr;
	(void)output_ptr;
	ret_val = RET_ERR;
#endif	// INCLUDE_ALGO_RANGEFINDER

	return ret_val;
}

uint8_t ch_common_init_algo(ch_dev_t *dev_ptr) {
	uint8_t ret_val = RET_ERR;

	ret_val = chdrv_algo_init(dev_ptr);

	return ret_val;
}

uint8_t ch_common_get_algo_info(ch_dev_t *dev_ptr, ICU_ALGO_SHASTA_INFO *algo_info_ptr) {
	uint8_t ret_val = RET_ERR;

	if (algo_info_ptr != NULL) {
		ret_val = chdrv_algo_info_read(dev_ptr, algo_info_ptr);
	}
	return ret_val;
}


uint8_t ch_common_get_algo_config(ch_dev_t *dev_ptr, void *algo_cfg_ptr) {
	uint8_t ret_val = RET_ERR;

	if (algo_cfg_ptr != NULL) {
		ret_val = chdrv_algo_cfg_read(dev_ptr, algo_cfg_ptr);
	}
	return ret_val;
}


uint8_t ch_common_get_algo_output(ch_dev_t *dev_ptr, void *algo_out_ptr) {
	uint8_t ret_val = RET_ERR;

	if (algo_out_ptr != NULL) {
		ret_val = chdrv_algo_out_read(dev_ptr, algo_out_ptr);
	}
	return ret_val;
}


uint8_t ch_common_get_algo_state(ch_dev_t *dev_ptr, void *algo_state_ptr) {
	uint8_t ret_val = RET_ERR;

	if (algo_state_ptr != NULL) {
		ret_val = chdrv_algo_state_read(dev_ptr, algo_state_ptr);
	}
	return ret_val;
}


uint8_t ch_common_get_mfg_info(ch_dev_t *dev_ptr, ch_mfg_info_t *info_ptr) {
	otp_copy_t otp;
	uint8_t ret_val = RET_OK;

	ret_val = chdrv_otpmem_copy(dev_ptr, &otp);

	if (ret_val == RET_OK) {
		info_ptr->mfg_year = otp.serial.year_since_2000;
		if (info_ptr->mfg_year != 0) {
			info_ptr->mfg_year += 2000;
		}
		info_ptr->mfg_week = otp.serial.work_week;
		info_ptr->mfg_site = otp.serial.production_site;

		info_ptr->product_code = otp.product_variant;
		info_ptr->package_code = otp.package_variant;
		info_ptr->mems_code    = otp.mems_variant;
		info_ptr->module_code  = otp.module_variant;
	}

	return ret_val;
}

#endif


