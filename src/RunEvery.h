#ifndef  RUN_EVERY_H
#define  RUN_EVERY_H
/**
 * Data storage for scheduling events.
 */

typedef struct __attribute__((__packed__)) _RUN_EVERY{
	//The start time for the schedule
	float MsTime;
	//The time from the start time to loop over
	float setPoint;
} RunEveryData;

/**
 * RunEvery
 * This function returns not 0 if it has been at least as long as the "setPoint" field says since the last time it returned not 0.
 * All timeing is handeled internally
 * @param data Pointer to a data storage table
 * @return float of MS after the assigned time that this function is running. A value of 0 means it has not been long enough
 */
float RunEvery(RunEveryData * data);

#endif
