# Chirp SonicLib

## Summary

Chirp SonicLib is a set of API functions and sensor driver routines designed 
to easily control Chirp ultrasonic sensors from an embedded C application.  It allows an
application developer to obtain ultrasonic range data from one or more devices, without
needing to develop special low-level code to interact with the sensors directly.

The SonicLib API functions provide a consistent interface for an application to use
Chirp sensors in various situations.  This is especially important, because
all Chirp sensors are completely software-defined, including the register map.  The
SonicLib interfaces allow an application to use different Chirp sensor firmware images,
without requiring code changes.  Only a single initialization parameter must be modified,
and one line added to a header file, to use a new sensor firmware version.

\note All operation of the sensor is controlled through the set of functions, data structures,
and symbolic values defined in the soniclib.h header file.  You should not need to modify this file 
or the SonicLib functions, or use lower-level internal functions such as described in 
the ch_driver.h file.  Using any of these non-public methods will reduce your ability to 
benefit from future enhancements and releases from Chirp.

## SonicLib API
### The main documentation for the SonicLib API is in the soniclib.h header file.
That file contains definitions of all public interfaces that applications can
use to interact with the sensor.

There are over 100 different functions available in the SonicLib API, although a typical 
application will only use a small subset of them.  There are many \a ch_set_XXX() and
\a ch_get_XXX() functions that set and return specific values for the device, so it should 
never be necessary to directly access fields in the device descriptor or other internal
structures.

## Required Board Support Package (BSP)
### The main documentation for the BSP interface is in the chirp_bsp.h header file.
SonicLib also defines interfaces for a set of board support package (BSP) 
functions that are specific to the hardware platform being used.  These are functions
that are not part of SonicLib, but that SonicLib will call when necessary
to interact with the peripheral devices and other resources on the board.  
These include GPIO pins, timers, enabling/disabling interrupts, etc.

The chirp_bsp.h file contains information on implementing these functions as well as
the interface definitions and which functions are required, recommended, or optional 
for your situation.

The BSP implementation in a final product is ultimately the responsibility of
the development team. However, example BSP's are available for certain 
evaluation boards for use during development and as a template for custom
designs.  Typically, BSPs that use the same MCU family will share common calls
to the MCU vendor's I/O library, varying mostly in pin assignments, etc.

Contact Chirp for information on BSP availability.

## Chirp Driver (internal)
SonicLib includes internal driver functions to interact directly with the sensor.
These modules are distributed as source code, to simplify integration with your embedded
application and for reference only.  These functions are described in the ch_driver.h 
header file.  You should not modify these functions or call them directly from your application.

## Support
support@chirpmicro.com

## Copyright
&copy; Copyright 2016-2022, TDK / Chirp Microsystems.  All rights reserved.
