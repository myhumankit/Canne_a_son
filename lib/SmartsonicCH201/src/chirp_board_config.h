/*
 * chirp_board_config.h
 *
 * This file defines required symbols used to build an application with the Chirp SonicLib
 * API and driver.  These symbols are used for static array allocations and counters in SonicLib
 * (and often applications), and are based on the number of specific resources on the target board.
 *
 * Two symbols must be defined:
 *  CHIRP_MAX_NUM_SENSORS - the number of possible sensor devices (i.e. the number of sensor ports)
 *  CHIRP_NUM_BUSES - the number of I2C/SPI buses on the board that are used for those sensor ports
 *
 * This file must be in the C pre-processor include path when the application is built with SonicLib
 * and this board support package.
 */

#ifndef CHIRP_BOARD_CONFIG_H_
#define CHIRP_BOARD_CONFIG_H_

#include "../../../include/boardConfig.h"

#define INCLUDE_WHITNEY_SUPPORT

 /* Settings for the Chirp driver test setup */
#define CHIRP_MAX_NUM_SENSORS 		NB_CH201_SENSOR		// maximum possible number of sensor devices
#define CHIRP_NUM_BUSES      		1		// number of I2C/SPI buses used by sensors

//#define CHIRP_PIN_PROG   {CHIRP_PROG_0, CHIRP_PROG_1, CHIRP_PROG_2, CHIRP_PROG_3}
//#define CHIRP_PIN_INT     {CHIRP_INT_0, CHIRP_INT_1, CHIRP_INT_2, CHIRP_INT_3}
//#define CHIRP_PIN_IO_IRQ {PIN_EXT_ChirpINT0_MASK, PIN_EXT_ChirpINT1_MASK, PIN_EXT_ChirpINT2_MASK, PIN_EXT_ChirpINT3_MASK}
//#define CHIRP_PIN_LED    {CHIRP_OK_0, CHIRP_OK_3, CHIRP_OK_2, CHIRP_OK_1}

#endif /* CHIRP_BOARD_CONFIG_H_ */