#ifndef  RUN_EVERY_H
#define  RUN_EVERY_H

class RunEveryObject {
public:
	//The start time for the schedule
	float MsTime;
	//The time from the start time to loop over
	float setPoint;
	float RunEvery(float currentTime){
		float diff;
		if(currentTime< MsTime)
			MsTime=currentTime;//Check and fix overflow
		diff =(currentTime-MsTime);
		if (diff > setPoint){
			if(MsTime+setPoint<currentTime)
				MsTime = currentTime;
			else
				MsTime += setPoint;
			return diff-setPoint;
		}
		return 0;
	}
};

/**
 * RunEvery
 * This function returns not 0 if it has been at least as long as the "setPoint" field says since the last time it returned not 0.
 * All timeing is handeled internally
 * @param data Pointer to a data storage table
 * @return float of MS after the assigned time that this function is running. A value of 0 means it has not been long enough
 */


#endif
