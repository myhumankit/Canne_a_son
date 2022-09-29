/*! \file icu_woa.h
 *
 * \brief Internal definitions for the Chirp ICU Wake-On-Approach sensor firmware.
 *
 * This file contains various definitions and values for use with the icu_woa
 * sensor firmware.
 *
 * You should not need to edit this file or call the driver functions directly.  Doing so
 * will reduce your ability to benefit from future enhancements and releases from Chirp.
 *
 */

/*
 * Copyright (c) 2016-2022, Chirp Microsystems.  All rights reserved.
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

#ifndef ICU_WOA_H_
#define ICU_WOA_H_

#include "icu.h"
#include "soniclib.h"
#include <stdint.h>

#define ICU_WOA_MAX_SAMPLES		(IQ_SAMPLES_MAX)	// from shasta_external_regs.h

extern const char *icu_woa_version;		// version string in fw .c file
extern const uint8_t icu_woa_fw_text[];
extern const uint8_t icu_woa_fw_vec[];
extern const uint16_t icu_woa_text_size;
extern const uint16_t icu_woa_vec_size;

uint16_t get_icu_woa_fw_ram_init_addr(void);
uint16_t get_icu_woa_fw_ram_init_size(void);

const unsigned char * get_ram_icu_woa_init_ptr(void);

uint8_t icu_woa_init(ch_dev_t *dev_ptr, ch_group_t *grp_ptr, uint8_t i2c_addr, uint8_t dev_num, uint8_t bus_index);

#endif	/* ICU_WOA_H_ */
