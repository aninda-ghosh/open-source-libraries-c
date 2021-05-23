/*
 * ISR_timer.h
 *
 *  Created on: Apr 30, 2021
 *      Author: aninda
 */

#ifndef INC_ISR_TIMER_H_
#define INC_ISR_TIMER_H_

#include "main.h"

#define STM32_ISR_Timer STM32_ISRTimer

typedef void (*timerCallback)();
typedef void (*timerCallback_p)(void*);


class STM32_ISR_Timer {

public:
	// maximum number of timers
#define MAX_NUMBER_TIMERS       16
#define TIMER_RUN_FOREVER       0
#define TIMER_RUN_ONCE          1

	// constructor
	STM32_ISR_Timer ();

	void init();

	// this function must be called inside loop()
	void run();

	// Timer will call function 'f' every 'd' milliseconds forever
	// returns the timer number (numTimer) on success or
	// -1 on failure (f == NULL) or no free timers
	int setInterval(unsigned long d, timerCallback f);

	// Timer will call function 'f' with parameter 'p' every 'd' milliseconds forever
	// returns the timer number (numTimer) on success or
	// -1 on failure (f == NULL) or no free timers
	int setInterval(unsigned long d, timerCallback_p f, void *p);

	// Timer will call function 'f' after 'd' milliseconds one time
	// returns the timer number (numTimer) on success or
	// -1 on failure (f == NULL) or no free timers
	int setTimeout(unsigned long d, timerCallback f);

	// Timer will call function 'f' with parameter 'p' after 'd' milliseconds one time
	// returns the timer number (numTimer) on success or
	// -1 on failure (f == NULL) or no free timers
	int setTimeout(unsigned long d, timerCallback_p f, void *p);

	// Timer will call function 'f' every 'd' milliseconds 'n' times
	// returns the timer number (numTimer) on success or
	// -1 on failure (f == NULL) or no free timers
	int setTimer(unsigned long d, timerCallback f, unsigned n);

	// Timer will call function 'f' with parameter 'p' every 'd' milliseconds 'n' times
	// returns the timer number (numTimer) on success or
	// -1 on failure (f == NULL) or no free timers
	int setTimer(unsigned long d, timerCallback_p f, void *p, unsigned n);

	// updates interval of the specified timer
	bool changeInterval(unsigned numTimer, unsigned long d);

	// destroy the specified timer
	void deleteTimer(unsigned numTimer);

	// restart the specified timer
	void restartTimer(unsigned numTimer);

	// returns true if the specified timer is enabled
	bool isEnabled(unsigned numTimer);

	// enables the specified timer
	void enable(unsigned numTimer);

	// disables the specified timer
	void disable(unsigned numTimer);

	// enables all timers
	void enableAll();

	// disables all timers
	void disableAll();

	// enables the specified timer if it's currently disabled, and vice-versa
	void toggle(unsigned numTimer);

	// returns the number of used timers
	unsigned getNumTimers();

	// returns the number of available timers
	unsigned getNumAvailableTimers() {
		return MAX_NUMBER_TIMERS - numTimers;
	}
	;

private:
	// deferred call constants
#define TIMER_DEFCALL_DONTRUN   0       // don't call the callback function
#define TIMER_DEFCALL_RUNONLY   1       // call the callback function but don't delete the timer
#define TIMER_DEFCALL_RUNANDDEL 2       // call the callback function and delete the timer

	// low level function to initialize and enable a new timer
	// returns the timer number (numTimer) on success or
	// -1 on failure (f == NULL) or no free timers
	int setupTimer(unsigned long d, void *f, void *p, bool h, unsigned n);

	// find the first available slot
	int findFirstFreeSlot();

	typedef struct {
		unsigned long prev_millis; // value returned by the millis() function in the previous run() call
		void *callback;           // pointer to the callback function
		void *param;              // function parameter
		bool hasParam;           // true if callback takes a parameter
		unsigned long delay;              // delay value
		unsigned maxNumRuns;         // number of runs to be executed
		unsigned numRuns;            // number of executed runs
		bool enabled;            // true if enabled
		unsigned toBeCalled; // deferred function call (sort of) - N.B.: only used in run()
	} timer_t;

	volatile timer_t timer[MAX_NUMBER_TIMERS];

	// actual number of timers in use (-1 means uninitialized)
	volatile int numTimers;
};

#endif /* INC_ISR_TIMER_H_ */
