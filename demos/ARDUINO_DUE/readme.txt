*****************************************************************************
** ChibiOS/RT demo for Arduino Due.                                        **
*****************************************************************************

** TARGET **

The demo runs on Arduino Due.

** Build Procedure **

The demo has been tested by using the free CodeSourcery GCC-based toolchain
that comes with Arduino.

Windows user should add
 
	${ARDUINO_PATH}\hardware\tools\g++_arm_none_eabi\bin  
	    (for the Codesourcery toolchain)
	${ARDUINO_PATH}\hardware\tools\avr\utils\bin  
	    (for the binutils like Unix style mkdir)
	    
to the path before start, where ${ARDUINO_PATH} is the installation 
directory of Arduino installation (>= 1.5.*).

Use `cs-make all' to build.  

Modify the TRGT line in the makefile in order to use different GCC toolchains.

** Purpose **

It blinks the 3 LEDs on Arduino Due with 3 different threads.  

** Flash Procedure **

For Windows users:

Flash the Arduino Due with (for Native Port)

   ${ArduinoDir}\hardware\tools\bossac.exe --port=${PORT} \ 
     -U true -e -w -v -b build/ch.bin -R

or with (for Programming Port)
     
   ${ArduinoDir}\hardware\tools\bossac.exe --port=${PORT} \ 
     -U false -e -w -v -b build/ch.bin -R
     
For flashing over the native port, one must manually press erase and reset.
Because the code for 1200bps auto reset is not in-place.