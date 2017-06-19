#ifndef INTERPOLATE_H
#define INTERPOLATE_H
  typedef struct  __attribute__((__packed__)) _INTERPOLATE_DATA
  {
  	//Target value for the interpolator to get to
  	float set;
  	//Initial starting point value of target
  	float start;
  	//How many ms the interpolation should take
  	float setTime;
  	//The timestamp of when the interpolation began.
  	float startTime;

  } INTERPOLATE_DATA;
#endif
