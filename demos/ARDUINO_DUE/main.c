#include "ch.h"
#include "halconf.h"
#include "pal.h"

/*
 * This is a periodic thread that does absolutely nothing except increasing
 * the seconds counter.
 */
static WORKING_AREA(waThread1, 128);
static WORKING_AREA(waThread2, 128);
static WORKING_AREA(waThread3, 128);

static msg_t Thread1(void *arg) {
	(void)arg;

	palSetPadMode(IOPORT2, 27, PAL_MODE_OUTPUT_PUSHPULL);
	while (TRUE) {
		palTogglePad(IOPORT2, 27);
		chThdSleepMilliseconds(1000);
	}
	return 0;
}

static msg_t Thread2(void *arg) {
	(void)arg;

	palSetPadMode(IOPORT3, 30, PAL_MODE_OUTPUT_PUSHPULL);

	palSetPad(IOPORT3, 21);
	while (TRUE) {
		palTogglePad(IOPORT3, 30);
		chThdSleepMilliseconds(1000);
	}
	return 0;
}

static msg_t Thread3(void *arg) {
	(void)arg;

	palSetPadMode(IOPORT1, 21, PAL_MODE_OUTPUT_PUSHPULL);
	while (TRUE) {
		palTogglePad(IOPORT1, 21);
		chThdSleepMilliseconds(500);
	}
	return 0;
}

/*
 * Application entry point.
 */
int main(void) {

	halInit();
  /*
   * System initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */

  chSysInit();

  /*
   * Creates the example thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);
  chThdCreateStatic(waThread2, sizeof(waThread2), NORMALPRIO, Thread2, NULL);
  chThdCreateStatic(waThread3, sizeof(waThread3), NORMALPRIO, Thread3, NULL);

  while (1);
}
