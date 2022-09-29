/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2020-2022 InvenSense - All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively “Software”) is subject
 * to InvenSense and its licensors' intellectual property rights under U.S. and international copyright
 * and other intellectual property rights laws.
 *
 * InvenSense and its licensors retain all intellectual property and proprietary rights in and to the Software
 * and any use, reproduction, disclosure or distribution of the Software without an express license agreement
 * from InvenSense is strictly prohibited.
 *
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, THE SOFTWARE IS
 * PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, IN NO EVENT SHALL
 * INVENSENSE BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, OR ANY
 * DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 * ________________________________________________________________________________________________________
 */

/*! \file app_config.h */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include "soniclib.h"
#include "../../../include/boardConfig.h"

/*========================= Sensor Firmware Selection ===========================*/

/* Select sensor firmware to use 
 *   The sensor firmware is specified during the call to ch_init(), by
 *   giving the name (address) of the firmware initialization function 
 *   that will be called.
 *
 *   To use a different sensor firmware type (e.g. a new distribution from
 *   Chirp), simply define CHIRP_SENSOR_FW_INIT_FUNC to equal the name of
 *   the init routine for the new firmware.
 */
#define	 CHIRP_SENSOR_FW_INIT_FUNC	CH201_FIRMWARE			/* standard STR firmware */
// #define	 CHIRP_SENSOR_FW_INIT_FUNC	ch201_gprstr_wd_init	/* watchdog-enabled STR firmware */



/*============================ Sensor Configuration =============================*/

/* Operating mode for the sensor
 *	 This value defines the mode in which the sensor will operate.  For this
 *	 example application, the choices are CH_MODE_FREERUN or CH_MODE_TRIGGERED_TX_RX.
 *
 *   This application uses CH_MODE_FREERUN by default, in which the sensor's internal
 *   clock will initiate the measurement cycles.  Triggered mode can be enabled instead, 
 *   to demonstrate controlling the sensor's measurement cycles from an external source
 *   (a periodic timer set up by the board support package). 
 */
#define CHIRP_SENSOR_MODE		CH201_MODE

/* Target detection interrupt filter mode
 *   This value controls under what conditions the sensor will interrupt following
 *   a measurement.  If no filtering is used (CH_TGT_INT_FILTER_OFF), the sensor 
 *   will interrupt after each measurement, whether or not a target object is detected.  
 *   If the mode is CH_TGT_INT_FILTER_ANY, the sensor will interrupt if any target
 *   is detected, but will not interrupt if no target is detected.
 *   If the mode is CH_TGT_INT_FILTER_COUNTER, the sensor will only interrupt if 
 *   certain number of recent measurements detected a target.
 *
 *   See the description of ch_set_target_interrupt() for more information.
 */
#define CHIRP_SENSOR_TARGET_INT		CH_TGT_INT_FILTER_OFF

/* Target counter filter mode settings
 *   These values control the target interrupt filter when using counter mode.
 *   The number of measurements kept in history is set by CHIRP_SENSOR_TARGET_INT_HIST.
 *
 *   The required number of target detections within the most recent
 *   (CHIRP_SENSOR_TARGET_INT_HIST + 1) measurements is set by CHIRP_SENSOR_TARGET_INT_THRESH.
 *
 *   See the description of ch_set_target_int_counter() for more information.
 */

#define CHIRP_SENSOR_TARGET_INT_HIST	5		// num of previous results kept in history
#define CHIRP_SENSOR_TARGET_INT_THRESH  3		// num of target detections req'd to interrupt

/* Maximum detection range for the sensor
 * This value will determine how long the sensor "listens" for an ultrasound 
 * signal (echo).  If the value specified here is greater than the maximum possible 
 * range for the sensor, the maximum possible range will be used.
 */
#define	CHIRP_SENSOR_MAX_RANGE_MM		CH201_MAX_RANGE	/* maximum range, in mm */

/* Static target rejection sample range, in samples (0=disabled)
 */
#define	CHIRP_SENSOR_STATIC_RANGE		0

/* Internal sample interval
 * NOT USED IF TRIGGERED
 */
#define CHIRP_SENSOR_SAMPLE_INTERVAL	0

/* Detection threshold settings
 * These values set the required amplitude for an ultrasound signal to be
 * reported as a detected object.  In the CH201 STR sensor firmware, two
 * thresholds may be set.  Threshold 0 is used for samples very close to the sensor.
 * Threshold 1 is used for samples at more typical operating distances.
 *
 * If either value is set to zero, the sensor's default value for that threshold
 * will not be changed.
 */
#define	CHIRP_SENSOR_THRESHOLD_0		0	/* close range threshold (0 = use default) */
#define	CHIRP_SENSOR_THRESHOLD_1		0	/* standard threshold (0 = use default) */

/* Receive holdoff range
 * This value specifies the number of samples at the start (closest end) of the measurement 
 * that will be ignored when detecting a target, regardless of the signal amplitude.  This
 * setting can be combined with the above threshold values to tune the responsiveness of 
 * the sensor over the desired measurement range.  The default is zero.
 */
#define	CHIRP_SENSOR_RX_HOLDOFF			0	/* # of samples to ignore at start of meas */

/* Receive low-gain range
 * This value specifies the number of samples within the measurement that will use
 * a lower gain setting than is used for farther samples.
 * 
 * If this value is set to zero, the sensor's default number of samples for the low-gain 
 * range will not be changed.  Otherwise, it will be set to the specified number of samples.
 * In CH201 GPRSTR firmware, the default is 60 samples.
 */
#define	CHIRP_SENSOR_RX_LOW_GAIN		0	/* # of samples (0 = use default) */

/* Transmit length
 * This value controls the duration of the ultrasound pulse that is generated by the sensor
 * at the start of a measurement.  The units are the number of internal PMUT (transducer) 
 * cycles per pulse.  In general, a pulse using a longer transmit length will have a higher 
 * amplitude when the echo is received.
 *
 * If this value is set to zero, the sensor's default transmit length will not be changed.  
 * For CH201 GPRSTR firmware, the default is 30 cycles.  The default value used by the sensor
 * is appropriate for most situations.
 */
#define	CHIRP_SENSOR_TX_LENGTH			0	/* Tx pulse length, in cycles (0 = use default) */


/*============================ STR Configuration =============================*/

/* Range for static target rejection (STR)
 * This value specifies how many samples within the measurement will be filtered
 * for static target rejection.  Stationary objects that are beyond this limit will
 * be reported.
 *
 * If zero (0) is specified, STR will be enabled for the entire measurement range 
 * specified in CHIRP_SENSOR_MAX_RANGE_MM.
 */
#define	CHIRP_SENSOR_STR_RANGE		0		/* STR range, in samples (0 = entire meas range) */

/*============================ Application Timing ============================*/

/* Define how often the application will get a new sample from the sensor(s) 
 *   This macro defines the sensor measurement interval, in milliseconds.  
 *
 *   For sensors in free-running mode (CH_MODE_FREERUN), the application will
 *   set this period as the sensor's internal sample interval.
 *
 *   For sensors in triggered mode (CH_MODE_TRIGGERED_TX_RX or 
 *   CH_MODE_TRIGGERED_RX_ONLY), the application will use a periodic timer to 
 *   trigger a sensor measurement each time this period elapses.
 */
#define	MEASUREMENT_INTERVAL_MS		CH201_MEASUREMENT_INTERVAL


/*===================  Application Storage for Sensor Data ======================*/

/* Define how many I/Q samples are expected by this application
 *   The following macro is used to allocate space for I/Q data in the 
 *   "chirp_data_t" structure, defined below.  It reserves enough space to
 *   hold I/Q samples for the maximum range of the sensor.
 */
#define IQ_DATA_MAX_NUM_SAMPLES  CH201_GPRSTR_MAX_SAMPLES	// use max for ch201_gprstr


/*========================  Application Display Settings ==========================*/

/* The following definitions set parameters affecting the application's display of STR output. They do not
 * modify the sensor operation. 
 */
//#define SHOW_PRESENCE_INDICATORS			/* COMMENT OUT THIS LINE TO DISABLE "Present" INDICATORS */
#define PRESENCE_HOLD_SECONDS      	1		/* time until end of presence is reported, in seconds */
#define NUM_RANGE_HISTORY_VALUES    7 		/* number of range history values used in filtering */ 


/*===================  Build Options for I/Q Data Handling ======================*/

/* The following build options control if and how the raw I/Q data is read 
 * from the device after each measurement cycle, in addition to the standard 
 * range and amplitude.  
 *
 * Note that reading the I/Q data is not required for most basic sensing 
 * applications - the reported range value is typically all that is required. 
 * However, the full data set may be read and analyzed for more advanced 
 * sensing or data capture needs.  You may customize the format of the
 * data output to meet your needs, based on these examples.
 *
 * Comment or un-comment the various definitions, as appropriate.
 *
 * If OUTPUT_AMP_DATA_CSV is defined, the application will read the I/Q data from
 * the sensor, then calculate the amplitude value for each I/Q pair and output
 * those values in CSV format.  The amplitude data is displayed as a continuation
 * of the same output line that contains the basic range information, with comma
 * separators.  This allows easy import of the amplitude values into a spreadsheet
 * or other tool.
 *
 * If OUTPUT_IQ_DATA_CSV is defined, the application will read the I/Q data from
 * the sensor and output the raw I/Q data values (in ascii) through the serial port.
 * The I/Q sample values for each sample is output on its own line as a comma-separated 
 * numeric value pair (Q,I).  The data can then be captured and imported into a 
 * spreadsheet or other analysis tool, with the Q and I values in separate columns.  
 *
 * The normal "Present" indicators are not displayed when amplitude or I/Q output
 * is enabled.
 */

// #define OUTPUT_AMP_DATA_CSV	/* uncomment to output calculated amplitudes in CSV */
// #define OUTPUT_IQ_DATA_CSV	/* uncomment to output I/Q data in CSV format */
											
/* By default, this application will read the I/Q data in blocking mode 
 * (i.e. READ_IQ_BLOCKING is defined by default).  The data will be read from 
 * the device and placed in the I/Q data array field in the application's 
 * chirp_data structure.  Because the I/Q data is read in blocking mode, the 
 * call to ch_get_iq_data() will not return until the data has actually 
 * been copied from the device.  
 *
 * However, if READ_IQ_NONBLOCKING is defined instead, the I/Q data will be 
 * read in non-blocking mode. The ch_get_iq_data() call will return immediately,
 * and a separate callback function will be called to notify the application 
 * when the read operation is complete.
 */

#define READ_IQ_BLOCKING 		/* use blocking mode when reading I/Q */
// #define READ_IQ_NONBLOCKING 	/* use non-blocking mode when reading I/Q */

#endif /* APP_CONFIG_H */
