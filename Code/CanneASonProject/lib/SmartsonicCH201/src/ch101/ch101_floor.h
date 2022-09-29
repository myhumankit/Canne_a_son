/*! \file ch101_floor.h
 *
 * \brief Internal definitions for the Chirp CH101 Floor Detection enabled firmware
 *
 * This file contains register offsets and other values for use with the CH101 Floor
 * sensor firmware.  These values are subject to change without notice.
 *
 * You should not need to edit this file or call the driver functions directly.  Doing so
 * will reduce your ability to benefit from future enhancements and releases from Chirp.
 *
 */

/*
 * Copyright � 2016-2022, Chirp Microsystems.  All rights reserved.
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
 * or by mail at 2560 Ninth Street, Suite 220, Berkeley, CA 94710.
 */

#ifndef CH101_FLOOR_H_
#define CH101_FLOOR_H_

#include "ch101.h"
#include "soniclib.h"
#include <stdint.h>

/* GPR firmware registers */
#define CH101_FLOOR_REG_OPMODE 				0x01
#define CH101_FLOOR_REG_TICK_INTERVAL 		0x02
#define CH101_FLOOR_REG_RX_WIN_END			0x04
#define CH101_FLOOR_REG_PERIOD 				0x05
#define CH101_FLOOR_REG_CAL_TRIG 			0x06
#define CH101_FLOOR_REG_MAX_RANGE 			0x07
#define CH101_FLOOR_REG_TX_LENGTH			0x08
#define CH101_FLOOR_REG_REV_CYCLES			0x0C
#define CH101_FLOOR_REG_DCO_PERIOD			0x0E
#define CH101_FLOOR_REG_RX_GAIN_ATTEN_1		0x10
#define CH101_FLOOR_REG_RX_HOLDOFF			0x11
#define CH101_FLOOR_REG_RX_GAIN_ATTEN_2		0x12
#define CH101_FLOOR_REG_DECIMATION			0x13
#define CH101_FLOOR_REG_READY 				0x14
#define CH101_FLOOR_REG_TOF_SF 				0x16
#define CH101_FLOOR_REG_AMPLITUDE_LOW		0x18
#define CH101_FLOOR_REG_AMPLITUDE_HIGH		0x1A
#define CH101_FLOOR_REG_CAL_RESULT 			0x0A
#define CH101_FLOOR_REG_DATA 				0x1C

#define CH101_FLOOR_MAX_SAMPLES				(150)

// Enumerated values for various registers
#define CH101_FLOOR_DECIMATION_MASK (0x03)
#define CH101_FLOOR_DECIMATION_NONE (0x00)
#define CH101_FLOOR_DECIMATION_2    (0x01)
#define CH101_FLOOR_DECIMATION_3    (0x02)
#define CH101_FLOOR_DECIMATION_4    (0x03)

#define CH101_FLOOR_CORDIC_MASK     (0x80)
#define CH101_FLOOR_CORDIC_EN       (0x80)
#define CH101_FLOOR_CORDIC_DIS      (0x00)

#define CH101_FLOOR_MODE_DONE		(0x00)		// note - same as CH_MODE_IDLE
#define CH101_FLOOR_MODE_SOFT_TRIG	(0x04)
#define CH101_FLOOR_MODE_PENDING	(0x08)
#define CH101_FLOOR_MODE_TUNING		(0x40)

extern const char *ch101_floor_version;		// version string in fw .c file
extern const uint8_t ch101_floor_fw_text[];
extern const uint8_t ch101_floor_fw_vec[];
extern const uint16_t ch101_floor_text_size;
extern const uint16_t ch101_floor_vec_size;

uint16_t get_ch101_floor_fw_ram_init_addr(void);
uint16_t get_ch101_floor_fw_ram_init_size(void);

const unsigned char * get_ram_ch101_floor_init_ptr(void);

uint8_t ch101_floor_init(ch_dev_t *dev_ptr, ch_group_t *grp_ptr, uint8_t i2c_addr, uint8_t dev_num, uint8_t bus_index);


#endif
