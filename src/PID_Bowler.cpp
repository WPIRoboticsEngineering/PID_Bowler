#include "PID_Bowler.h"

void PIDBowler::updatePosition(){
  state.CurrentState = getPosition() - state.config.offset;
}
void PIDBowler::updateControl(){
  if (state.config.Enabled == true) {
      state.SetPoint = state.interpolate.run(getMs());
      MathCalculationPosition(getMs());
      if (GetPIDCalibrateionState() <= CALIBRARTION_DONE) {
          setOutput(state.Output);
      } else if (GetPIDCalibrateionState() == CALIBRARTION_hysteresis) {
          pidHysterisis();
      } else if ((GetPIDCalibrateionState() == CALIBRARTION_home_down) ||
              (GetPIDCalibrateionState() == CALIBRARTION_home_up) ||
              (GetPIDCalibrateionState() == CALIBRARTION_home_velocity)) {
          checkLinkHomingStatus();
      }
  }else{
			RunPDVel();
  }
}


float PIDBowler::runPdVelocityFromPointer(float currentState,float KP, float KD){

    float currentTime = getMs();
		float timeMsDiff =  (currentTime -state.vel.lastTime);
		float timeDiff =  timeMsDiff/1000;
		float posDiff=currentState -state.vel.lastPosition;
		float currentVelocity = posDiff/timeDiff;
		//float velocityDiff = currentVelocity-state.vel.lastVelocity;
		float velocityDiff=0;
		float proportional =  currentVelocity-state.vel.unitsPerSeCond;
		float set = (proportional*KP)+(velocityDiff*KD)*timeMsDiff;
		state.vel.currentOutputVel-=(set);

		if (state.vel.currentOutputVel>200){
			state.vel.currentOutputVel=200;
                }else if(state.vel.currentOutputVel<-200){
			state.vel.currentOutputVel=-200;
                }

		// println_I("\t Velocity: set=   ");p_fl_I(state.vel.unitsPerSeCond );print_I(" ticks/seCond" );
    //             println_I("\t current state=   ");p_fl_I(currentState );print_I(" ticks" );
    //             println_I("\t last state=      ");p_fl_I(state.vel.lastPosition );print_I(" ticks" );
		// println_I("\t position diff=   ");p_fl_I(posDiff );print_I(" ticks" );
		// println_I("\t MS diff=         ");p_fl_I(timeMsDiff );
		// println_I("\t current=         ");p_fl_I(currentVelocity );print_I(" ticks/seCond" );
		// println_I("\t Velocity offset= ");p_fl_I(set );
		// println_I("\t Velocity set=    ");p_fl_I(state.vel.currentOutputVel );

		//cleanup
		state.vel.lastPosition=currentState;
		state.vel.lastVelocity=currentVelocity;
		state.vel.lastTime=currentTime;
    return state.vel.currentOutputVel;
}

void PIDBowler::RunPDVel(){
	//println_I("Running PID vel");
	if(state.vel.enabled==true) {


		state.Output=runPdVelocityFromPointer(
                        state.CurrentState,
                        state.config.V.P,
                        state.config.V.D
                        );

    if(state.calibration.state<=CALIBRARTION_DONE)
        setOutput(state.Output);
	}
}

void PIDBowler::StartPDVel(float unitsPerSeCond,float ms){

        if(ms<.1){
            //println_I("Starting Velocity");
            state.vel.enabled=true;
            state.config.Enabled=false;
            state.vel.lastPosition=GetPIDPosition();
            state.vel.lastTime=getMs();
            state.vel.unitsPerSeCond=unitsPerSeCond;
            state.vel.currentOutputVel =0;
        }else{
            //println_I("Starting Velocity Timed");
            float seConds = ms/1000;
            int32_t dist = (int32_t) unitsPerSeCond*(int32_t) seConds;
            int32_t delt = ((int32_t) (GetPIDPosition())-dist);
            SetPIDTimed( delt, ms);
        }


}

void PIDBowler::MathCalculationVelocityDefault( float currentTime){

}
void PIDBowler::OnPidConfigure() {
    onPidConfigureLocal();
}

PD_VEL * PIDBowler::getPidVelocityDataTable() {
    return &state.vel;
}

AbsPID * PIDBowler::getPidGroupDataTable() {

    // Internal reference stores the address of the base of the array
    // Add to that the size of the struct times the index. THis should create
    // a pointer to the address of this specific array address
    return &state;
}

bool PIDBowler::isPidEnabled() {
    return state.config.Enabled;
}

void PIDBowler::SetPIDEnabled( bool enabled) {
    state.config.Enabled = enabled;
}

void  PIDBowler::InitilizePidController() {
      int enabled = state.config.Enabled;
      pidReset( 0);
      SetPIDEnabled( enabled);

}

void PIDBowler::SetPIDCalibrateionState(PidCalibrationType incoming) {
    state.config.calibrationState = incoming;
    OnPidConfigure();
}

PidCalibrationType PIDBowler::GetPIDCalibrateionState() {

    return state.config.calibrationState;
}

uint8_t PIDBowler::ZeroPID() {
    //b_println("Resetting PID channel from zeroPID:",INFO_PRINT);
    pidReset( 0);
    return true;
}

uint8_t PIDBowler::ClearPID() {
    state.config.Enabled = false;
    return true;
}

uint8_t PIDBowler::SetPIDTimedPointer( float val, float current, float ms) {
    if (ms < .01)
        ms = 0;
    //local_groups[chan].config.Enabled=true;
    state.interpolate.set = val;
    state.interpolate.setTime = ms;
    state.interpolate.start = current;
    state.interpolate.startTime = getMs();
    state.SetPoint = val;
    //conf->config.Enabled=true;
    InitAbsPIDWithPosition( state.config.K.P,
                            state.config.K.I,
                            state.config.K.D,
                            getMs(),
                            current);
    return true;
}

uint8_t PIDBowler::SetPIDTimed(float val, float ms) {
    state.vel.enabled = false;
    return SetPIDTimedPointer( val, GetPIDPosition(), ms);
}

uint8_t PIDBowler::SetPID( float val) {
    SetPIDTimed( val, 0);
    return true;
}

int PIDBowler::GetPIDPosition() {
    //state.CurrentState=(int)getPosition(chan);
    return state.CurrentState;
}

float PIDBowler::pidResetNoStop( int32_t val) {
    //float value = (float)resetPosition(chan,val);
    float current = state.CurrentState;
    float raw = current + state.config.offset;
    float value = (float) val;
    state.config.offset = (raw - value);
    state.CurrentState = raw - state.config.offset;
//    //println_E("From pidReset Current State: ");
//    p_fl_E(current);
//    print_E(" Target value: ");
//    p_fl_E(value);
//    print_E(" Offset: ");
//    p_int_E(state.config.offset);
//    print_E(" Raw: ");
//    p_int_E(raw);
    float time = getMs();
    state.lastPushedValue = val;
    InitAbsPIDWithPosition( state.config.K.P,
       state.config.K.I, state.config.K.D, time, val);
    state.vel.lastPosition = val;
    state.vel.lastTime = time;
    return val;
}

void PIDBowler::pidReset( int32_t val) {
    float value = pidResetNoStop( val);

    state.interpolate.set = value;
    state.interpolate.setTime = 0;
    state.interpolate.start = value;
    state.interpolate.startTime = getMs();
    state.SetPoint = value;
    uint8_t enabled = state.config.Enabled;
    state.config.Enabled = true; //Ensures output enabled to stop motors
    state.Output = 0.0;
    setOutput( state.Output);
    state.config.Enabled = enabled;

}

void PIDBowler::InitAbsPID( float KP, float KI, float KD, float time) {
    InitAbsPIDWithPosition( KP, KI, KD, time, 0);
}

void PIDBowler::setPIDConstants( float p, float i, float d) {
    state.config.K.P = p;
    state.config.K.I = i;
    state.config.K.D = d;
}

/**
 * RunAbstractPIDCalc
 * @param state A pointer to the AbsPID struct to run the calculations on
 * @param CurrentTime a float of the time it is called in MS for use by the PID calculation
 */
void PIDBowler::InitAbsPIDWithPosition( float KP, float KI, float KD, float time, float currentPosition) {
    state.config.K.P = KP;
    state.config.K.I = KI;
    state.config.K.D = KD;
    //state.integralCircularBufferIndex = 0;
    state.integralTotal = 0.0;
    state.integralSize = 20.0;
    state.SetPoint = currentPosition;
    state.PreviousError = 0;
    state.Output = 0.0;
    state.PreviousTime = time;
}

bool PIDBowler::isPIDInterpolating() {
    return state.interpolate.setTime != 0;
}

bool PIDBowler::isPIDArrivedAtSetpoint( float plusOrMinus) {
    if (state.config.Enabled)
        return bound(state.SetPoint,
            state.CurrentState,
            plusOrMinus,
            plusOrMinus);
    return true;
}

void PIDBowler::RunPIDControl() {
   updatePosition();
   updateControl();
}

// void RunPIDComs(BowlerPacket *Packet, bool(*pidAsyncCallbackPtr)(BowlerPacket *Packet)) {
//     int i;
//     for (i = 0; i < getNumberOfPidChannels(); i++) {
//         pushPIDLimitEvent(Packet, pidAsyncCallbackPtr, checkPIDLimitEvents(i));
//     }
//     updatePidAsync(Packet, pidAsyncCallbackPtr);
// }
//
// void RunPID(BowlerPacket *Packet, bool(*pidAsyncCallbackPtr)(BowlerPacket *Packet)) {
//     RunPIDControl();
//     RunPIDComs(Packet, pidAsyncCallbackPtr);
// }

/**
 * InitAbsPID
 * @param state A pointer to the AbsPID the initialize
 * @param KP the Proportional Constant
 * @param KI the Integral Constant
 * @param KD the Derivative Constant
 * @param time the starting time
 */

void PIDBowler::RunAbstractPIDCalc( float CurrentTime) {
    float error;
    float derivative;



    //calculate set error
    error = state.SetPoint - state.CurrentState;

    //remove the value that is INTEGRALSIZE cycles old from the integral calculation to avoid overflow
    //state.integralTotal -= state.IntegralCircularBuffer[state.integralCircularBufferIndex];
    //add the latest value to the integral
    state.integralTotal = (error * (1.0 / state.integralSize)) +
            (state.integralTotal * ((state.integralSize - 1.0) / state.integralSize));

    //This section clears the integral buffer when the zero is crossed
    if ((state.PreviousError >= 0 && error < 0) ||
            (state.PreviousError < 0 && error >= 0)) {
        state.integralTotal = 0;
    }


    //calculate the derivative
    derivative = (error - state.PreviousError); // / ((CurrentTime-state.PreviousTime));
    state.PreviousError = error;

    //do the PID calculation
    state.Output = ((state.config.K.P * error) +
            (state.config.K.D * derivative) +
            (state.config.K.I * state.integralTotal)
            );

    if (state.config.Polarity == false)
        state.Output *= -1.0;
    //Store the current time for next iterations previous time
    state.PreviousTime = CurrentTime;

}

void PIDBowler::setOutput( float val) {
    if(bound(0,state.config.tipsScale, .001, .001)){
      //  println_W("PID TPS Sclale close to zero");p_fl_W(state.config.tipsScale);
    }

    val *= state.config.tipsScale;
    val += getPidStop();
    if (val > getPidStop() && val < getUpperPidHistoresis() ){
        val = getUpperPidHistoresis();
        ////println_E("Upper histerisys");
    }
    if (val < getPidStop() && val > getLowerPidHistoresis()){
        val = getLowerPidHistoresis();
      //  //println_E("Lower histerisys");
    }
    state.OutputSet = val;
    //
    setOutputLocal( val);
}


void PIDBowler::incrementHistoresis() {
    state.config.upperHistoresis += 1;
    //calcCenter( group);
}

void PIDBowler::decrementHistoresis() {
    state.config.lowerHistoresis -= 1;
}

void PIDBowler::calcCenter() {
    int diff = (state.config.upperHistoresis + state.config.lowerHistoresis) / 2;
    state.config.stop = diff;
}

void PIDBowler::checkCalibration() {
    if (state.calibration.calibrated != true) {
        state.config.upperHistoresis = 0;
        state.config.lowerHistoresis = 0;
        state.config.stop = 0;
        state.calibration.calibrated = true;
    }
}

int PIDBowler::getUpperPidHistoresis() {
    checkCalibration();
    return state.config.upperHistoresis;
}

int PIDBowler::getLowerPidHistoresis() {
    checkCalibration();
    return state.config.lowerHistoresis;
}

int PIDBowler::getPidStop() {
    checkCalibration();
    return state.config.stop;
}

// boolean processRunAutoCal(BowlerPacket * Packet) {
//      = Packet->use.data[0];
//
//     runPidHysterisisCalibration();
//
//     READY(Packet, 0, 0);
//     return true;
// }

void PIDBowler::runPidHysterisisCalibration() {

    if (!state.config.Enabled) {
        ////println_E("Axis disabled for calibration #");
        //p_int_E();
        state.config.Enabled = true;
    }
    state.config.lowerHistoresis = 0;
    state.config.upperHistoresis = 0;
    state.config.stop = 0;
    //    println_I("\tReset PID");
    pidReset( 0); // Zero encoder reading
    //   println_I("\tDisable PID Output");
    SetPIDEnabled( true);
    SetPIDCalibrateionState( CALIBRARTION_hysteresis);

    state.calibration.state = forward;
    //  println_I("\tSetting slow move");
    setOutput( -1.0f);
    state.timer.setPoint = 2000;
    state.timer.MsTime = getMs();

}

CAL_STATE PIDBowler::pidHysterisis() {

    if (state.timer.RunEvery(getMs()) > 0) {
      //  //Print_Level l = getPrintLevel();
        //setPrintLevelInfoPrint();
        float boundVal = 150.0;
        float extr = GetPIDPosition();
        if (bound(0, extr, boundVal, boundVal)) {// check to see if the encoder has moved
            //we have not moved
            //          println_I("NOT moved ");p_fl_I(extr);
            if (state.calibration.state == forward) {
                incrementHistoresis();
            } else if (state.calibration.state == backward) {
                decrementHistoresis();
            }
            int historesisBound = 25;
            if (state.config.lowerHistoresis < (-historesisBound) &&
                    state.calibration.state == backward) {
                ////println_E("Backward Motor seems damaged, more then counts of historesis #");
              //  p_int_I();
                state.calibration.state = forward;
            }
            if (state.config.upperHistoresis > (historesisBound) &&
                    state.calibration.state == forward) {
                ////println_E("Forward Motor seems damaged, more then counts of historesis #");
                //p_int_I();
                state.calibration.state = done;
            }
        } else {
            pidReset( 0);
            setOutput( 0);
            ////println_E("Moved ");
          //  p_fl_E(extr);
            if (state.calibration.state == forward) {
                //println_I("Backward Calibrated for link# ");
                //p_int_I();
                state.calibration.state = backward;
            } else {
                //println_I("Calibration done for link# ");
                //p_int_I();
                state.calibration.state = done;

                float offset = .9;
                state.config.lowerHistoresis *= offset;
                state.config.upperHistoresis *= offset;
                calcCenter();
            }

        }
        if (state.calibration.state == forward) {
            setOutput( 1.0f);
        } else if (state.calibration.state == backward) {
            setOutput( -1.0f);
        }
        ////setPrintLevel(l);
    }
    if (state.calibration.state == done)
        SetPIDCalibrateionState( CALIBRARTION_DONE);
    return state.calibration.state;
}

void PIDBowler::startHomingLink( PidCalibrationType type, float homedValue) {
    float speed = 20.0;
    if (type == CALIBRARTION_home_up)
        speed *= 1.0;
    else if (type == CALIBRARTION_home_down)
        speed *= -1.0;
    else {
        //println_E("Invalid homing type");
        return;
    }
    state.config.tipsScale = 1;
    SetPIDCalibrateionState( type);
    setOutput( speed);
    state.timer.MsTime = getMs();
    state.timer.setPoint = 1000;
    state.homing.homingStallBound = 20;
    state.homing.previousValue = GetPIDPosition();
    state.homing.lastTime = getMs();
    state.homing.homedValue = homedValue;
    SetPIDEnabled(true);
}

void PIDBowler::checkLinkHomingStatus() {
    if (!(GetPIDCalibrateionState() == CALIBRARTION_home_down ||
            GetPIDCalibrateionState() == CALIBRARTION_home_up ||
            GetPIDCalibrateionState() == CALIBRARTION_home_velocity
            )
            ) {
        return; //Calibration is not running
    }
    float current = GetPIDPosition();
    float currentTime = getMs();
    if (state.timer.RunEvery(getMs()) > 0) {
        //println_W("Check Homing ");
        if (GetPIDCalibrateionState() != CALIBRARTION_home_velocity) {
            float boundVal = state.homing.homingStallBound;

            if (bound(state.homing.previousValue,
                    current,
                    boundVal,
                    boundVal
                    )
                    ) {
                pidReset( state.homing.homedValue);
                //after reset the current value will have changed
                current = GetPIDPosition();
                state.config.tipsScale = 1;
                //println_W("Homing Velocity for group ");
                //p_int_W();
                //print_W(", Resetting position to: ");
                //p_fl_W(state.homing.homedValue);
              //  print_W(" current ");
                //p_fl_W(current);

                float speed = -20.0;
                if (GetPIDCalibrateionState() == CALIBRARTION_home_up)
                    speed *= 1.0;
                else if (GetPIDCalibrateionState() == CALIBRARTION_home_down)
                    speed *= -1.0;
                else {
                    ////println_E("Invalid homing type");
                    return;
                }
                //Print_Level l = getPrintLevel();
                //setPrintLevelInfoPrint();
                setOutput( speed);
                //setPrintLevel(l);
                state.timer.MsTime = getMs();
                state.timer.setPoint = 2000;
                SetPIDCalibrateionState( CALIBRARTION_home_velocity);
                state.homing.lastTime = currentTime;
            }
        } else {
            current = GetPIDPosition();
            float posDiff = current - state.homing.previousValue; //ticks
            float timeDiff = (currentTime - state.homing.lastTime) / 1000.0; //
            float tps = (posDiff / timeDiff);
            state.config.tipsScale = 20 / tps;

            // //println_E("New scale factor: ");
            // p_fl_E(state.config.tipsScale);
            // print_E(" speed ");
            // p_fl_E(tps);
            // print_E(" on ");
            // p_int_E();
            // print_E(" Position difference ");
            // p_fl_E(posDiff);
            // print_E(" time difference ");
            // p_fl_E(timeDiff);


            OnPidConfigure();
            SetPIDCalibrateionState( CALIBRARTION_DONE);
        }
        state.homing.previousValue = current;
    }
}
