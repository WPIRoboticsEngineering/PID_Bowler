#ifndef PID_BOWLER_H
#define PID_BOWLER_H
 #include <stdint.h>
typedef enum _PidLimitType {
	//NO_LIMIT(0x00),
	/** The lowerlimit. */
	//LOWERLIMIT(0x01),

	/** The indexevent. */
	//INDEXEVENT(0x02),

	/** The upperlimit. */
	//UPPERLIMIT(0x04),

	/** The overcurrent. */
	//OVERCURRENT(0x08),
	//CONTROLLER_ERROR(0x10),
	//HOME_EVENT(0x20)
    NO_LIMIT = (0x00),

    LOWERLIMIT = (0x01),

    INDEXEVENT = (0x02),

    UPPERLIMIT = (0x04),

    OVERCURRENT = (0x08),

    CONTROLLER_ERROR = (0x10),
	HOME_EVENT = (0x20)
} PidLimitType;

typedef enum _PidCalibrationType {
    CALIBRARTION_Uncalibrated = (0),
    CALIBRARTION_DONE = (1),
    CALIBRARTION_hysteresis = (2),
    CALIBRARTION_home_up = (3),
    CALIBRARTION_home_down = (4),
    CALIBRARTION_home_velocity = (5)

} PidCalibrationType;

typedef struct __attribute__((__packed__)) _PidLimitEvent {
    uint8_t group;
    PidLimitType type;
    float time;
    int32_t value;
    int32_t latchTickError;
    //	boolean stopOnIndex;
}
PidLimitEvent;

/**
 * These are your Control Constants
 */
typedef enum _CAL_STATE {
    forward = 0,
    backward = 1,
    done = 2
} CAL_STATE;

class PIDBowler {
public:
  // Implement in the subclass
  virtual float getPosition()=0;
  virtual void setOutputLocal( float)=0;
  virtual int resetPosition(int)=0;
  virtual void onPidConfigureLocal()=0;
  virtual void MathCalculationPosition( float)=0;
  virtual void MathCalculationVelocity( float)=0;
  virtual PidLimitEvent* checkPIDLimitEvents()=0;
protected:
  PidLimitEvent currentEvent;
  void MathCalculationPositionDefault( float currentTime);
  void MathCalculationVelocityDefault( float currentTime);

};
#endif
