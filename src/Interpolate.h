#ifndef INTERPOLATE_H
#define INTERPOLATE_H
# define isnan(a) (a!=a)
  class Interpolate {
    public:
    //Target value for the interpolator to get to
  	float set;
  	//Initial starting point value of target
  	float start;
  	//How many ms the interpolation should take
  	float setTime;
  	//The timestamp of when the interpolation began.
  	float startTime;
    Interpolate();
    float go( float currentTime) ;
    bool between(float targetupper, float actual, float targetLower) ;

  };
#endif
