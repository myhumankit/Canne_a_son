/*! \file icu.h
 *
 * \brief Internal definitions for TDK/Chirp ICU ultrasonic sensors.
 *
 * This file contains various hardware-defined values for ICU series sensors,
 * including ICU-20201.
 
 * You should not need to edit this file or call the driver functions directly.  Doing so
 * will reduce your ability to benefit from future enhancements and releases from Chirp.
 *
 */

/*
 * Copyright � 2016-2021, Chirp Microsystems.  All rights reserved.
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

#ifndef ICU_H_
#define ICU_H_

#include "ch_asic_shasta.h"

#define ICU_DATA_MEM_SIZE			SHASTA_DATA_MEM_SIZE
#define ICU_DATA_MEM_ADDR			SHASTA_DATA_MEM_ADDR
#define ICU_PROG_MEM_SIZE			SHASTA_PROG_MEM_SIZE
#define ICU_PROG_MEM_ADDR			SHASTA_PROG_MEM_ADDR
#define ICU_FW_SIZE					ICU_PROG_MEM_SIZE

#define ICU_CPU_ID					SHASTA_CPU_ID_HI_VALUE			/*!< Value in sensor ID reg */

#define ICU_BANDWIDTH_INDEX_1		6 								 /*!< Index of first sample for bandwidth calc */
#define ICU_BANDWIDTH_INDEX_2    	(ICU_BANDWIDTH_INDEX_1 + 1) /*!< Index of second sample for bandwidth calc */
#define ICU_SCALEFACTOR_INDEX		4 								 /*!< Index for calculating scale factor. */

#define ICU_MAX_TICK_INTERVAL		256

#define ICU_COMMON_READY_FREQ_LOCKED	(SHASTA_READY_FREQ_LOCKED)		// XXX TODO confirm (if applicable)
#define ICU_COMMON_FREQCOUNTERCYCLES (SHASTA_FREQCOUNTERCYCLES)		// XXX here to make compiler happy

#endif	// ICU_H_
