/*
 *  Fonctions d'utilisation des capteurs ultrasons CH201
 */

#include "CH201.h"

/* Array of structs to hold measurement data, one for each possible device */
chirp_data_t	chirp_data[CHIRP_MAX_NUM_SENSORS];

/* Array of ch_dev_t device descriptors, one for each possible device */
ch_dev_t	chirp_devices[CHIRP_MAX_NUM_SENSORS];

/* Configuration structure for group of sensors */
ch_group_t 	chirp_group;

/* Detection level settings - for CH201 sensors only
 *   Each threshold entry includes the starting sample number & threshold level.
 */
ch_thresholds_t chirp_ch201_thresholds = {0, 	5000,		/* threshold 0 */
                                          26,	2000,		/* threshold 1 */
                                          39,	800,		/* threshold 2 */
                                          56,	400,		/* threshold 3 */
                                          79,	250,		/* threshold 4 */
                                          89,	175};		/* threshold 5 */

/* Task flag word
        *   This variable contains the DATA_READY_FLAG and IQ_READY_FLAG bit flags
*   that are set in I/O processing routines.  The flags are checked in the
*   main() loop and, if set, will cause an appropriate handler function to
        *   be called to process sensor data.
*/
volatile uint32_t taskflags = 0;

/* Device tracking variables
 *   These are bit-field variables which contain a separate bit assigned to
 *   each (possible) sensor, indexed by the device number.  The active_devices
 *   variable contains the bit pattern describing which ports have active
 *   sensors connected.  The data_ready_devices variable is set bit-by-bit
 *   as sensors interrupt, indicating they have completed a measurement
 *   cycle.  The two variables are compared to determine when all active
 *   devices have interrupted.
 */
static uint32_t active_devices;
static uint32_t data_ready_devices;

/* Number of connected sensors */
static uint8_t	num_connected_sensors = 0;

/* STR variables
*    These are data structures which hold the presence detection outputs and variables
*/
presence_output_t	presence_output;
presence_utils_t	presence_utils;

/* Number of seconds that "Present" indicator will continue */
static int presence_hold_count = 0;


/* Number of sensors that use h/w triggering to start measurement */
static uint8_t	num_triggered_devices = 0;

hw_timer_t *My_timer = NULL;

#if (defined(READ_IQ_DATA) && defined(READ_IQ_NONBLOCKING))
/* Count of non-blocking I/Q reads queued */
static uint8_t	num_io_queued = 0;
#endif


#ifdef READ_IQ_DATA
static uint8_t display_iq_data(ch_dev_t *dev_ptr);
#ifdef READ_IQ_NONBLOCKING
static uint8_t handle_iq_data_done(ch_group_t *grp_ptr);
#endif
#endif

/*  Variables globales  */
ch_group_t	*grp_ptr = &chirp_group;
uint8_t  	dev_num;
uint8_t  	chirp_error = 0;
uint8_t  	num_ports;
ch_i2c_info_t *info_ptr;


/*
 *  Détection des CH201, chargement du firmware et configuration.
 *
 *  Sortie : Renvoi 1 si tout s'est bien passé, 0 en cas d'erreur
 */
bool initCH201(){
    /* Initialize presence detection utility */
    presence_utils_init(&presence_utils);

    /* Initialize board hardware functions
     *   This call to the board support package (BSP) performs all necessary
     *   hardware initialization for the application to run on this board.
     *   This includes setting up memory regions, initializing clocks and
     *   peripherals (including I2C and //Serial port), and any processor-specific
     *   startup sequences.
     *
     *   The chbsp_board_init() function also initializes fields within the
     *   sensor group descriptor, including number of supported sensors and
     *   the RTC clock calibration pulse length.
     */
    chbsp_board_init(grp_ptr);


    printf("\nSonicLib version: %u.%u.%u\n", SONICLIB_VER_MAJOR,SONICLIB_VER_MINOR, SONICLIB_VER_REV);


    /* Get the number of (possible) sensor devices on the board
     *   Set by the BSP during chbsp_board_init()
     */
    num_ports = ch_get_num_ports(grp_ptr);

    /* Initialize sensor descriptors.
     *   This loop initializes each (possible) sensor's ch_dev_t descriptor,
     *   although we don't yet know if a sensor is actually connected.
     *
     *   The call to ch_init() specifies the sensor descriptor, the sensor group
     *   it will be added to, the device number within the group, and the sensor
     *   firmware initialization routine that will be used.  (The sensor
     *   firmware selection effectively specifies whether it is a CH101 or
     *   CH201 sensor, as well as the exact feature set.)
     */
    Serial.print("Initializing sensor(s)... ");
    for (dev_num = 0; dev_num < num_ports; dev_num++) {
        ch_dev_t *dev_ptr = &(chirp_devices[dev_num]);	// init struct in array

        /* Init device descriptor
         *   Note that this assumes all sensors will use the same sensor
         *   firmware.  The CHIRP_SENSOR_FW_INIT_FUNC symbol is defined in
         *   str.h and is used for all devices.
         *
         *   However, it is possible for different sensors to use different firmware
         *   images, by specifying different firmware init routines when ch_init() is
         *   called for each.
         */
        chirp_error |= ch_init(dev_ptr, grp_ptr, dev_num, CHIRP_SENSOR_FW_INIT_FUNC);
    }

    /* Start all sensors.
     *   The ch_group_start() function will search each port (that was
     *   initialized above) for a sensor. If it finds one, it programs it (with
     *   the firmware specified above during ch_init()) and waits for it to
     *   perform a self-calibration step.  Then, once it has found all the
     *   sensors, ch_group_start() completes a timing reference calibration by
     *   applying a pulse of known length to the sensor's INT line.
     */
    if (chirp_error == 0) {
        Serial.print("starting group... ");
        chirp_error = ch_group_start(grp_ptr);
    }

    if (chirp_error == 0) {
        Serial.println("OK");
    } else {
        printf("FAILED: %d\n", chirp_error);
        return 0;
    }
    printf("\n");

    /* Get and display the initialization results for each connected sensor.
     *   This loop checks each device number in the sensor group to determine
     *   if a sensor is actually connected.  If so, it makes a series of
     *   function calls to get different operating values, including the
     *   operating frequency, clock calibration values, and firmware version.
     */
    printf("Sensor\tType\tFreq\t\tB/W\t\tRTC Cal\t\tFirmware\n");

    for (dev_num = 0; dev_num < num_ports; dev_num++) {
        ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);

        if (ch_sensor_is_connected(dev_ptr)) {

            printf("%d\tCH%d\t%u Hz\t%u Hz \t%u@%ums\t%s\n", dev_num,
                   ch_get_part_number(dev_ptr),
                   (unsigned int) ch_get_frequency(dev_ptr),
                   (unsigned int) ch_get_bandwidth(dev_ptr),
                   ch_get_rtc_cal_result(dev_ptr),
                   ch_get_rtc_cal_pulselength(dev_ptr),
                   ch_get_fw_version_string(dev_ptr));
        }
    }
    printf("\n");

    /* Register callback function to be called when Chirp sensor interrupts */
    ch_io_int_callback_set(grp_ptr, sensor_int_callback);

    /* Register callback function called when non-blocking I/Q readout completes
      *   Note, this callback will only be used if READ_IQ_DATA_NONBLOCK is
     *   defined to enable non-blocking I/Q readout in this application.
     */
    ch_io_complete_callback_set(grp_ptr, io_complete_callback);

    /* Configure each sensor with its operating parameters
     *   Initialize a ch_config_t structure with values defined in the
     *   app_config.h header file, then write the configuration to the
     *   sensor using ch_set_config().
     */
    printf ("Configuring sensor(s)...\n");

    for (dev_num = 0; dev_num < num_ports; dev_num++) {
        ch_config_t dev_config;
        ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);

        if (ch_sensor_is_connected(dev_ptr)) {

            /* Select sensor mode
             *   All connected sensors are placed in hardware triggered mode.
               *   The first connected (lowest numbered) sensor will transmit and
             *   receive, all others will only receive.
               */

            num_connected_sensors++;					// count one more connected
            active_devices |= (1 << dev_num);	// add to active device bit mask

            if (dev_config.mode != CH_MODE_FREERUN) {	// unless free-running
                num_triggered_devices++;				// will be triggered
            }

            if (num_connected_sensors == 1) {			// if this is the first sensor
                dev_config.mode = CH_MODE_TRIGGERED_TX_RX;
            }

            /* Init config structure with values from hello_chirp.h */
            dev_config.max_range       = CHIRP_SENSOR_MAX_RANGE_MM;
            dev_config.static_range    = CHIRP_SENSOR_STATIC_RANGE;
            dev_config.sample_interval = CHIRP_SENSOR_SAMPLE_INTERVAL;

            /* Set detection thresholds (CH201 only) */
            if (ch_get_part_number(dev_ptr) == CH201_PART_NUMBER) {
                /* Set pointer to struct containing detection thresholds */
                dev_config.thresh_ptr = &chirp_ch201_thresholds;
            } else {
                dev_config.thresh_ptr = 0;
            }

            /* Apply sensor configuration */
            chirp_error = ch_set_config(dev_ptr, &dev_config);

            /* Enable sensor interrupt if using free-running mode
             *   Note that interrupt is automatically enabled if using
             *   triggered modes.
             */
            if ((!chirp_error) && (dev_config.mode == CH_MODE_FREERUN || dev_config.mode == CH_MODE_TRIGGERED_TX_RX)) {
                chbsp_int1_interrupt_enable(dev_ptr);
            }

            /* Read back and display config settings */
            if (!chirp_error) {
                display_config_info(dev_ptr);
            } else {
                printf("Device %d: Error during ch_set_config()\n", dev_num);
                return 0;
            }

            /* Turn on an LED to indicate device connected */
            if (!chirp_error) {
                chbsp_led_on(dev_num);
            }

        }
    }

    printf("\n");


    /* Initialize the periodic timer.
     *   For sensors in triggered mode, the timer will be used to trigger the
     *   measurements.  This timer is also used to control the persistent output
     *   of a presence indication for a fixed "hold" time.
     *
     *   This function initializes a timer that will interrupt every time it
     *   expires, after the specified measurement interval.  The function also
     *   registers a callback function that will be called from the timer
     *   handler when the interrupt occurs.
     */
    printf("Initializing sample timer for %dms interval... ", MEASUREMENT_INTERVAL_MS);

    My_timer = timerBegin(1, 80, true);
    timerAttachInterrupt(My_timer, periodic_timer_callback, true);
    timerAlarmWrite(My_timer, MEASUREMENT_INTERVAL_MS*1000, true);
    timerAlarmEnable(My_timer);
    printf("OK\n");
    return 1;
}


/*
 *  Demande au capteur CH201 d'effectuer une nouvelle mesure ou une lecture de la mesure selon le flag "taskflags".
 *
 *  Sortie : Renvoi 1 si la fonction vient de demander une mesure au capteur, 2 si une mesure vient d'être lue, 0 en cas d'erreur
 */
int sendReceiveCH201(unsigned int *distance, unsigned int *amplitude){
    if (taskflags & DATA_READY_FLAG) {
        //Serial.print(" DATA_READY_FLAG ->");
        chbsp_group_set_int1_dir_out(grp_ptr);
        digitalWrite(INT1,LOW);
        /* All sensors have interrupted - handle sensor data */
        taskflags &= ~DATA_READY_FLAG;		// clear flag
        handle_data_ready(grp_ptr, distance, amplitude);			// read and display measurement
        return 2;
    }

    /* Check for non-blocking I/Q readout complete */
    if (taskflags & TIMER_FLAG) {
        //Serial.print("TIMER_FLAG ->");
        if (num_triggered_devices > 0) {
            ledcWrite(0, 0); //OFF buzzer
            ledcWrite(1, 0); //OFF vibreur
            ch_group_trigger(&chirp_group);
        }
        /* Periodic timer interrupt occurred */
        /* All non-blocking I/Q readouts have completed */
        taskflags &= ~TIMER_FLAG;		// clear flag
        //handle_iq_data(grp_ptr);			// display I/Q data
        return 1;
    }
    return 0;
}


/*  Routine d'interruption du timer  */
void periodic_timer_callback(void) {
    taskflags |= TIMER_FLAG;    // Set TIMER_FLAG
}


/*  Routine d'interruption de la sortie INT du capteur  */
void sensor_int_callback() {
    taskflags |= DATA_READY_FLAG;   // Set DATA_READY_FLAG
}


/*
 * io_complete_callback() - non-blocking I/O complete callback routine
 *
 * This function is called by SonicLib's I2C DMA handling function when all
 * outstanding non-blocking I/Q readouts have completed.  It simply sets a flag
 * that will be detected and handled in the main() loop.
 *
 * This callback function is registered by the call to
 * ch_io_complete_callback_set() in main().
 *
 *  Note: This callback is only used if READ_IQ_NONBLOCKING is defined to
 *  select non-blocking I/Q readout in this application.
 */
static void io_complete_callback(ch_group_t __attribute__((unused)) *grp_ptr) {

    taskflags |= IQ_READY_FLAG;
}

/*
 * presence_utils_init() - initialize presence utility functions
 *
 * This function initializes the presence utility median filter.
 */
static void presence_utils_init(presence_utils_t *util) {
    util->medianFilter.numNodes = NUM_RANGE_HISTORY_VALUES;
    util->medianFilter.medianBuffer = util->medianBuffer;

    MEDIANFILTER_Init(&util->medianFilter);
}

/*
 * display_config_info() - display the configuration values for a sensor
 *
 * This function displays the current configuration settings for an individual
 * sensor.  The operating mode, maximum range, and static target rejection
 * range (if used) are displayed.
 *
 * For CH201 sensors only, the multiple detection threshold values are also
 * displayed.
 */
static uint8_t display_config_info(ch_dev_t *dev_ptr) {
    ch_config_t 	read_config;
    uint8_t 		chirp_error;
    uint8_t 		dev_num = ch_get_dev_num(dev_ptr);

    /* Read configuration values for the device into ch_config_t structure */
    chirp_error = ch_get_config(dev_ptr, &read_config);

    if (!chirp_error) {
        char *mode_string;

        switch (read_config.mode) {
            case CH_MODE_IDLE:
                mode_string = "IDLE";
                break;
            case CH_MODE_FREERUN:
                mode_string = "FREERUN";
                break;
            case CH_MODE_TRIGGERED_TX_RX:
                mode_string = "TRIGGERED_TX_RX";
                break;
            case CH_MODE_TRIGGERED_RX_ONLY:
                mode_string = "TRIGGERED_RX_ONLY";
                break;
            default:
                mode_string = "UNKNOWN";
        }

        /* Display sensor number, mode and max range */
        printf("Sensor %d:\tmax_range=%dmm \tmode=%s  ", dev_num, read_config.max_range, mode_string);

        /* Display static target rejection range, if used */
        if (read_config.static_range != 0) {
            //printf("static_range=%d samples", read_config.static_range);
        }

        /* Display detection thresholds (only supported on CH201) */
        if (ch_get_part_number(dev_ptr) == CH201_PART_NUMBER) {
            ch_thresholds_t read_thresholds;

            /* Get threshold values in structure */
            chirp_error = ch_get_thresholds(dev_ptr, &read_thresholds);

            if (!chirp_error) {
                //printf("\n  Detection thresholds:\n");
                for (int i = 0; i < CH_NUM_THRESHOLDS; i++) {
                    //printf("     %d\tstart: %2d\tlevel: %d\n", i,read_thresholds.threshold[i].start_sample,read_thresholds.threshold[i].level);
                }
            } else {
                //printf(" Device %d: Error during ch_get_thresholds()", dev_num);
            }
        }
        //printf("\n");

    } else {
        //printf(" Device %d: Error during ch_get_config()\n", dev_num);
    }

    return chirp_error;
}

/*
 * update_range() - update the range using the presence filter
 *
 * This function obtains an updated range value using the presence utility
 * median filter, based on a new measured range value.
 */
static uint32_t update_range(presence_utils_t *util, uint32_t range_in) {
    uint32_t range_out = (uint32_t) MEDIANFILTER_Insert(&(util->medianFilter), range_in);

    if (range_out == 0) {
        range_out = range_in;		// use input value if zero
    }

    return range_out;
}

/*
 * handle_data_ready() - get data from all sensors
 *
 * This routine is called from the main() loop after all sensors have
 * interrupted. It shows how to read the sensor data once a measurement is
 * complete.  This routine always reads out the range and amplitude, and
 * optionally will read out the raw I/Q for all samples in the measurement.
 *
 * See the comments in app_config.h for information about the I/Q readout
 * build options.
 *
 */
static uint8_t handle_data_ready(ch_group_t *grp_ptr, unsigned int *distance, unsigned int *amplitude) {
    uint8_t 	dev_num;
    uint8_t 	ret_val = 0;

    /* Read and display data from the connected sensor
     *   This loop will write the sensor data to this application's "chirp_data"
     *   array.  Each possible sensor has a separate chirp_data_t structure in that
     *   array, so the device number is used as an index.
     */

    for (dev_num = 0; dev_num < ch_get_num_ports(grp_ptr); dev_num++) {
        ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);

        if (ch_sensor_is_connected(dev_ptr)) {

            /* Get measurement results from the connected sensor */

            chirp_data[dev_num].range = ch_get_range(dev_ptr, CH_RANGE_ECHO_ONE_WAY);

            if (chirp_data[dev_num].range != CH_NO_TARGET) {
                /* Because sensor is in target interrupt mode, it should only interrupt
                 * if a target was successfully detected, and this should always be true */

                /* Get the new amplitude value - it's only updated if range
                 * was successfully measured.  */
                chirp_data[dev_num].amplitude = ch_get_amplitude(dev_ptr);
                uint32_t range = update_range(&presence_utils, chirp_data[dev_num].range);
                presence_output.presence_range = range;
                distance[dev_num+1] = (unsigned int) range/32;
                amplitude[dev_num+1] =  chirp_data[dev_num].amplitude;

                presence_output.presence_detection = 1;
            }

#ifdef READ_IQ_DATA
            /* Optionally read raw I/Q values for all samples */
			display_iq_data(dev_ptr);
#endif
        }
    }

    return ret_val;
}

#ifdef READ_IQ_DATA
/* display_iq_data() - Read full I/Q data from device into buffer or queue read
 * This function is used to obtain the full I/Q data from the sensor following a
 * measurement.  Depending on the build options specified in the app_config.h
 * header file, this routine will either read the I/Q data here, before returning,
 * or it will queue a non-blocking read of the data.
 *
 * If READ_IQ_BLOCKING is defined, this function will read the data from
 * the sensor into the application's "chirp_data" structure for this device
 * before returning.
 *
 * If READ_IQ_NONBLOCKING is defined, the I/Q read operation will be queued
 * and this routine will return immediately.  A callback routine will be called
 * when the read operation is complete.  The callback routine must have been
 * registered using the ch_io_complete_callback_set function.
 *
 * Two options are provided to output the data in ascii CSV (comma separated value)
 * format, suitable for import into a spreadsheet or other program on the host system.
 * These are controlled by definitions in the app_config.h header file.
 *
 * If OUTPUT_AMP_DATA_CSV is defined, I/Q data read from the device will be converted
 * to amplitude values for each sample, then output in CSV format.  The amplitude values
 * for each sample in the measurement will be output on the same line that contains the
 * regular target range information, separated by commas.  This amplitude data may be
 * easily used to construct a simple "A-scan" type chart.
 *
 * The raw I/Q data values may also be output in comma-separated-value (CSV) format.
 * If OUTPUT_IQ_DATA_CSV is defined, full I/Q data will be output as a series of
 * comma-separated value pairs (Q, I), each on a separate line.
 */
uint8_t display_iq_data(ch_dev_t *dev_ptr) {
	uint16_t 	start_sample = 0;
	uint8_t 	error = 0;
	uint16_t 	num_samples = ch_get_num_samples(dev_ptr);
	uint8_t		dev_num = ch_get_dev_num(dev_ptr);

#if defined(READ_IQ_BLOCKING)
	ch_iq_sample_t *iq_ptr;			// pointer to individual I/Q pair

	/* Read I/Q data in normal, blocking mode */
	error = ch_get_iq_data(dev_ptr, chirp_data[dev_num].iq_data,
								start_sample, num_samples, CH_IO_MODE_BLOCK);

	if (!error) {
		//printf("    %d I/Q samples copied", num_samples);
		iq_ptr =  &(chirp_data[dev_num].iq_data[0]);

#ifdef OUTPUT_AMP_DATA_CSV
		/* Calculate and output amplitude values in CSV format, on same line */
		for (uint16_t i = 0; i < num_samples; i++) {
			//printf(", %d", ch_iq_to_amplitude(iq_ptr++));	// print comma separator and amp value
		}
#endif

#ifdef OUTPUT_IQ_DATA_CSV
		/* Output individual I/Q values in CSV format, one pair (sample) per line */
		//printf("\n");
		for (uint16_t count = 0; count < num_samples; count++) {
			//printf("%d,%d\n", iq_ptr->q, iq_ptr->i);	// output Q before I
			iq_ptr++;
		}
#endif

	} else {
		//printf("    Error reading %d I/Q samples", num_samples);
	}

#elif defined(READ_IQ_NONBLOCKING)
	/* Reading I/Q data in non-blocking mode - queue a read operation */

	//printf("     queuing %d I/Q samples... ", num_samples);

	error = ch_get_iq_data(dev_ptr, chirp_data[dev_num].iq_data,
								start_sample, num_samples, CH_IO_MODE_NONBLOCK);

	if (!error) {
		num_io_queued++;		// record a pending non-blocking read
		//printf("OK");
	} else {
		//printf("**ERROR**");
	}
#endif  // defined(READ_IQ_BLOCKING)

	return error;
}
#endif  // READ_IQ_DATA


#if (defined(READ_IQ_DATA) && defined(READ_IQ_NONBLOCKING))
/*
 * handle_iq_data_done() - handle raw I/Q data from a non-blocking read
 *
 * This function is called from the main() loop when a queued non-blocking
 * readout of the raw I/Q data has completed for all sensors.  The data will
 * have been placed in this application's "chirp_data" array, in the
 * chirp_data_t structure for each sensor, indexed by the device number.
 *
 * The output options specified in app_config.h for this function are the
 * same as for the preceding display_iq_data().  Refer to the comments there
 * for more information.
 */
static uint8_t handle_iq_data_done(ch_group_t *grp_ptr) {
	int				dev_num;
	uint16_t 		num_samples;
#if (defined(OUTPUT_IQ_DATA_CSV) || defined(OUTPUT_AMP_DATA_CSV))
	ch_iq_sample_t 	*iq_ptr;			// pointer to an I/Q sample pair
#endif

	for (dev_num = 0; dev_num < ch_get_num_ports(grp_ptr); dev_num++) {
		ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);

		if (ch_sensor_is_connected(dev_ptr)) {

			num_samples = ch_get_num_samples(dev_ptr);

			//printf ("Read %d samples from device %d:", num_samples, dev_num);

#ifdef OUTPUT_AMP_DATA_CSV
			iq_ptr = chirp_data[dev_num].iq_data;	// start of I/Q in data buf

			/* Calculate and output amplitude values in CSV format, on same line */
			for (uint16_t i = 0; i < num_samples; i++) {
				//printf(", %d", ch_iq_to_amplitude(iq_ptr++));	// output comma separator and amp value
			}
			//printf("\n");
#endif

#ifdef OUTPUT_IQ_DATA_CSV
			iq_ptr = chirp_data[dev_num].iq_data;	// start of I/Q in data buf

			/* Output IQ values in CSV format, one pair per line */
			//printf("\n");
			for (uint16_t count = 0; count < num_samples; count++) {
				//printf("%d,%d\n", iq_ptr->q, iq_ptr->i);	// output Q before I
				iq_ptr++;									// next sample
			}
			//printf("\n");
#endif
		}
	}

	return 0;
}

#endif  // (defined(READ_IQ_DATA) && defined(READ_IQ_NONBLOCKING))
