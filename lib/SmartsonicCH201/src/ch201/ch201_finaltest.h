/*!
 * \file ch201_finaltest.h
 *
 * \brief Internal definitions for the Chirp CH201 Finaltest sensor firmware.
 *
 * This file contains function definitions, register offsets and other interfaces
 * for use with the CH201 Finaltest sensor firmware.  Many of these are designed for
 * compatibility with the Chirp Finaltest production test system.
 * These values are subject to change without notice.
 *
 * You should not need to edit this file or call the driver functions directly.  Doing so
 * will reduce your ability to benefit from future enhancements and releases from Chirp.
 *
 */

/*
 * Copyright © 2016-2022, Chirp Microsystems.  All rights reserved.
 *
 * Chirp Microsystems CONFIDENTIAL
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CHIRP MICROSYSTEMS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * You can contact the authors of this program by email at support@chirpmicro.com
 * or by mail at 2560 Ninth Street, Suite 220A, Berkeley, CA 94710.
 */

#ifndef CH201_FINALTEST_H_
#define CH201_FINALTEST_H_

#include "ch201.h"
#include "soniclib.h"
#include <stddef.h>
#include <stdint.h>


#define CH201_FINALTEST_REG_REGMAPFMT			0x00
#define CH201_FINALTEST_REG_OPMODE          	0x01
#define CH201_FINALTEST_REG_TICK_INTERVAL   	0x02
#define CH201_FINALTEST_REG_WBCFG				0x04
#define CH201_FINALTEST_REG_PERIOD          	0x05
#define CH201_FINALTEST_REG_CAL_TRIG        	0x06
#define CH201_FINALTEST_REG_MAX_RANGE       	0x07
#define CH201_FINALTEST_REG_DCOSTART			0x08
#define CH201_FINALTEST_REG_CAL_RESULT      	0x0A
#define CH201_FINALTEST_REG_DCOSTOP				0x0C
#define CH201_FINALTEST_REG_TXLENGTH			0x0E
#define CH201_FINALTEST_REG_READY           	0x0F
#define CH201_FINALTEST_REG_PULSE_WIDTH     	0x10
#define CH201_FINALTEST_REG_HOLDOFF         	0x11
#define CH201_FINALTEST_REG_THRESHOLD			0x12
#define CH201_FINALTEST_REG_RXQUEUE				0x14
#define CH201_FINALTEST_REG_TOF_SF          	0x22
#define CH201_FINALTEST_REG_TOF             	0x24
#define CH201_FINALTEST_REG_AMPLITUDE       	0x26
#define CH201_FINALTEST_REG_DCOCODE				0x28
#define CH201_FINALTEST_REG_DATA				0x2A

// Maximum number of samples that can be stored; max value of MAX_RANGE
#define CH201_FINALTEST_MAX_SAMPLES				450

// Number of RXQUEUE 16-bit entries
#define CH201_FINALTEST_RXQUEUE_ITEMS			7

// Bit width of each field in RXQUEUE items
#define CH201_FINALTEST_RXQUEUE_BITS_SAMPLES	7
#define CH201_FINALTEST_RXQUEUE_BITS_ATTEN		2
#define CH201_FINALTEST_RXQUEUE_BITS_GAIN		3

// Position of lowest bit in each field of RXQUEUE items
#define CH201_FINALTEST_RXQUEUE_BITPOS_SAMPLES	3
#define CH201_FINALTEST_RXQUEUE_BITPOS_ATTEN	(CH201_FINALTEST_RXQUEUE_BITPOS_SAMPLES + CH201_FINALTEST_RXQUEUE_BITS_SAMPLES)
#define CH201_FINALTEST_RXQUEUE_BITPOS_GAIN		(CH201_FINALTEST_RXQUEUE_BITPOS_ATTEN + CH201_FINALTEST_RXQUEUE_BITS_ATTEN )

// Values for Rx attenuation IN RXQUEUE items
#define CH201_FINALTEST_RXQUEUE_ATTEN120		0
#define CH201_FINALTEST_RXQUEUE_ATTEN60			1
#define CH201_FINALTEST_RXQUEUE_ATTEN30			2
#define CH201_FINALTEST_RXQUEUE_ATTEN1			3

// Values for Rx gain IN RXQUEUE items
#define CH201_FINALTEST_RXQUEUE_GAIN1P6			0
#define CH201_FINALTEST_RXQUEUE_GAIN3P8			1
#define CH201_FINALTEST_RXQUEUE_GAIN5P7			2
#define CH201_FINALTEST_RXQUEUE_GAIN12P3		3
#define CH201_FINALTEST_RXQUEUE_GAIN25P6		4

// Enumerated values for various registers
#define CH201_FINALTEST_OPMODE_IDLE             0x00
#define CH201_FINALTEST_OPMODE_FREERUN          0x02
#define CH201_FINALTEST_OPMODE_TRIGGERED        0x10
#define CH201_FINALTEST_OPMODE_RX_ONLY          0x20

#define CH201_FINALTEST_PERIOD_IDLE             0
#define CH201_FINALTEST_TICK_INTERVAL_IDLE      2048

#define CH201_FINALTEST_READY_NOTLOCKED         0x00
#define CH201_FINALTEST_READY_NOTREADY          0x01
#define CH201_FINALTEST_READY_FREQ_LOCKED_BM    0x04


// ASIC firmware linkage
extern const char *ch201_finaltest_version;		// version string in fw .c file

extern const uint8_t ch201_finaltest_fw_text[];
extern const uint8_t ch201_finaltest_fw_vec[];
extern const uint16_t ch201_finaltest_text_size;
extern const uint16_t ch201_finaltest_vec_size;

const unsigned char * get_ram_ch201_finaltest_init_ptr(void);
uint16_t get_ch201_finaltest_fw_ram_init_addr(void);
uint16_t get_ch201_finaltest_fw_ram_init_size(void);

uint8_t ch201_finaltest_init(ch_dev_t *dev_ptr, ch_group_t *grp_ptr, uint8_t i2c_addr, uint8_t io_index, uint8_t bus_index);


typedef enum {		// XXX redundant definition from soniclib.h to resolve circular #include's
	CH201_FINALTEST_RANGE_ECHO_ONE_WAY 		= 0,		/*!< One way - gets full pulse/echo distance & divides by 2. */
	CH201_FINALTEST_RANGE_ECHO_ROUND_TRIP	= 1,		/*!< Round trip - full pulse/echo distance. */
	CH201_FINALTEST_RANGE_DIRECT			= 2,		/*!< Direct - for receiving node in pitch-catch mode. */
} ch201_finaltest_range_t;


/*!
 * \brief Write an entry into the receive settings queue
 *
 * \param dev_ptr a pointer to the ch_dev_t device descriptor
 * \param queue_index which position in the queue to write this item
 * \param samples sample count for which these settings will be in effect, Range of values 1-127
 * \param attenuation Range of values 0-3, see Final Test interface description
 * \param gain Range of values 0-4, see Final Test interface description
 */
uint8_t ch201_finaltest_set_rxqueue_item( ch_dev_t* dev_ptr, uint8_t queue_index,
		uint8_t samples, uint8_t attenuation, uint8_t gain );

/*!
 * \brief Set the MAX_RANGE register directly.
 *
 * \param dev_ptr a pointer to the ch_dev_t device descriptor
 * \param samples Value to write to MAX_RANGE register, determining length of data used in target detection
 *
 * \return 0 if successful
 */
uint8_t ch201_finaltest_set_samples( ch_dev_t* dev_ptr, uint8_t samples );

uint8_t ch201_finaltest_set_max_range(ch_dev_t *dev_ptr, uint16_t max_range_mm);

/*!
 * \brief Configure threshold of detection.
 *
 * \param dev_ptr a pointer to the ch_dev_t device descriptor
 * \param threshold minimum received echo intensity for detecting a target
 *
 * \return 0 if successful.
 */
uint8_t ch201_finaltest_set_threshold(ch_dev_t *dev_ptr, uint16_t threshold);

/*!
 * \brief Configure target detection holdoff.
 *
 * \param dev_ptr a pointer to the ch_dev_t device descriptor
 * \param holdoff number of samples to discard before starting target detection
 *
 * \return 0 if successful.
 */
uint8_t ch201_finaltest_set_holdoff(ch_dev_t *dev_ptr, uint8_t holdoff);

uint32_t ch201_finaltest_get_range(ch_dev_t *dev_ptr, ch201_finaltest_range_t range_type);


/*!
 * \brief Gets measured intensity from the sensor.
 *
 * \param dev_ptr a pointer to the ch_dev_t device descriptor
 *
 * This function reads back the amplitude from the sensor. The intensity is representative of the incoming sound pressure.
 *
 * \return Amplitude (arbitrary units).
 */
uint16_t ch201_finaltest_get_amplitude(ch_dev_t *dev_ptr);

/*!
 * \brief Reads IQ data from sensor and places it into specified buffer.
 * \param dev_ptr Pointer to the ch_dev_t device descriptor
 * \param buf_ptr Buffer to which to store IQ data
 * \param start_sample  starting sample of requested I/Q data
 * \param num_samples  number of samples to return I/Q for
 * \param mode  I/O mode - must be CH_IO_MODE_BLOCK
 * \return 0 on success, nonzero on failure
 */
uint8_t  ch201_finaltest_get_iq_data(ch_dev_t *dev_ptr, uint8_t /*ch_iq_sample_t*/ *buf_ptr, uint16_t start_sample, uint16_t num_samples,
uint8_t /*ch_io_mode_t*/ mode);


/*!
 * \brief Check if the sensor has completed its start-up calibration (locking) procedure.
 *
 * \param dev_ptr a pointer to the ch_dev_t device descriptor
 *
 * \return 1 if the sensor is locked, 0 otherwise.
 */
uint8_t ch201_finaltest_get_locked_state(ch_dev_t *dev_ptr);

void ch201_finaltest_prepare_pulse_timer(ch_dev_t *dev_ptr);

void ch201_finaltest_store_pt_result(ch_dev_t *dev_ptr);

void ch201_finaltest_store_op_freq(ch_dev_t *dev_ptr);

void ch201_finaltest_store_bandwidth(ch_dev_t *dev_ptr);

void ch201_finaltest_store_scale_factor(ch_dev_t *dev_ptr);

int ch201_finaltest_set_pulse_width(ch_dev_t *dev_ptr,uint8_t pulse_width);

int ch201_finaltest_set_tx_length(ch_dev_t *dev_ptr, uint8_t tx_length);

uint32_t ch201_finaltest_get_op_freq(ch_dev_t *dev_ptr);


#endif	/* CH201_FINALTEST_H_ */
