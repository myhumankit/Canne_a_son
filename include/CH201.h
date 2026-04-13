/*
 *  Fonctions d'utilisation des capteurs ultrasons CH201
 */

#ifndef PCB_CANNEASON_CH201_H
#define PCB_CANNEASON_CH201_H

#include <stdio.h>
#include "soniclib.h"			// Chirp SonicLib sensor API definitions
#include "chirp_board_config.h"	// required header with basic device counts etc.
#include "chirp_bsp.h"			// board support package function definitions
#include "app_config.h"
#include "app_version.h"
#include "MedianFilter.h"       // definitions specific to this application
#include "ultrasound_display_config_info.h"

/* Bit flags used in main loop to check for completion of I/O or timer operations.  */
#define DATA_READY_FLAG		(1 << 0)		// data ready from sensor
#define IQ_READY_FLAG		(1 << 1)		// non-blocking I/Q read has completed
#define TIMER_FLAG			(1 << 2)		// period timer has interrupted

/* Number of measurement cycles to hold presence count, equals (hold seconds * sampling rate) */
#define PRESENCE_HOLD_CYCLES	(PRESENCE_HOLD_SECONDS * 1000 / MEASUREMENT_INTERVAL_MS)

/* Enable I/Q readout if needed */
#if (defined(OUTPUT_AMP_DATA_CSV) || defined(OUTPUT_IQ_DATA_CSV))
#define READ_IQ_DATA						// enable I/Q read
#undef  SHOW_PRESENCE_INDICATORS			// disable "Present" indicators if amp or I/Q output
#endif


/* chirp_data_t - Structure to hold measurement data for one sensor
 *   This structure is used to hold the data from one measurement cycle from
 *   a sensor.  The data values include the measured range, the ultrasonic
 *   signal amplitude, the number of valid samples (I/Q data pairs) in the
 *   measurement, and (optionally) the raw I/Q data from the measurement.
 *
 *  The format of this data structure is specific to this application, so
 *  you may change it as desired.
 *
 *  A "chirp_data[]" array of these structures, one for each possible sensor,
 *  is declared in the main.c file.  The sensor's device number is
 *  used to index the array.
 */
typedef struct {
    uint32_t		range;							// from ch_get_range()
    uint16_t		amplitude;						// from ch_get_amplitude()
    uint16_t		num_samples;					// from ch_get_num_samples()
#ifdef READ_IQ_DATA
    ch_iq_sample_t	iq_data[IQ_DATA_MAX_NUM_SAMPLES];
												// from ch_get_iq_data()
#endif
} chirp_data_t;

/* presence_output_t - Structure to hold the presence data from STR firmware
 *   This structure is used to hold the data from one measurement cycle from
 *   a sensor.  The data values include the presence detection result (0/1),
 *   and the the range where presence is detected
 *
 *  The format of this data structure is specific to this application, so
 *  you may change it as desired.
 */
typedef struct {
    uint8_t		presence_detection;
    uint32_t	presence_range;
} presence_output_t;

/* presence_utils_t - Structure to hold the presence utility data from STR firmware
 *   This structure is used to hold the data from NUM_RANGE_HISTORY_VALUES measurement
 *   cycles from the sensor.
 *
 *  The format of this data structure is specific to this application, so
 *  you may change it as desired.
 */
typedef struct {
    sMedianFilter_t medianFilter;
    sMedianNode_t medianBuffer[NUM_RANGE_HISTORY_VALUES];
} presence_utils_t;


bool initCH201();       /* Détection des CH201, chargement du firmware et configuration  */
int sendReceiveCH201(unsigned int *distance, unsigned int *amplitude);      /*   Demande au capteur CH201 d'effectuer une nouvelle mesure ou une lecture de la mesure selon le flag "taskflags" */
void    sensor_int_callback();      /*  Routine d'interruption de la sortie INT du capteur  */
static void    io_complete_callback(ch_group_t *grp_ptr);       /*  Non-blocking I/O complete callback routine  */
static void     periodic_timer_callback(void);      /*  Routine d'interruption du timer  */
static uint8_t  handle_data_ready(ch_group_t *grp_ptr, unsigned int *distance, unsigned int *amplitude);        /*  get data from all sensors  */
static void     presence_utils_init(presence_utils_t *util);        /*  initialize presence utility functions  */
static uint32_t update_range(presence_utils_t *util, uint32_t range);       /*  update the range using the presence filter  */
static uint8_t display_config_info(ch_dev_t *dev_ptr);      /*  Display the configuration values for a sensor  */

#endif //PCB_CANNEASON_CH201_H
