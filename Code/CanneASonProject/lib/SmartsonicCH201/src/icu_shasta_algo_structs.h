/*! \file icu_shasta_algo_structs.h
 *
 * \brief Local SonicLib version of sensor algorithm header file
 * 
 * This is simply a wrapper header file that uses build symbols to select the
 * appropriate version of the icu_shasta_algo_structs.h file located in a
 * specific subdirectory under sensor_fw (which must be in the include path).
 */

/*
 Copyright (c) 2021-2022, Chirp Microsystems.  All rights reserved.

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

#ifndef SONICLIB_ICU_SHASTA_ALGO_STRUCTS_H
#define SONICLIB_ICU_SHASTA_ALGO_STRUCTS_H

#include "soniclib.h"

#if defined(INCLUDE_ALGO_RANGEFINDER)
#include "icu_gpt/icu_shasta_algo_structs.h"

/* add new sensor f/w interfaces here */

#elif defined(INCLUDE_ALGO_NONE)
#include "icu_init/icu_shasta_algo_structs.h"		// no algorithm - initialization only
#endif

#endif //SONICLIB_ICU_SHASTA_ALGO_STRUCTS_H
