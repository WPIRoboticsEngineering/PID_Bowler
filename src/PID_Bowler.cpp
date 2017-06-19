#include "PID_Bowler.h"


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
//    println_E("From pidReset Current State: ");
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

    state.CurrentState = getPosition() - state.config.offset;
    if (state.config.Enabled == true) {
        state.SetPoint = state.interpolate.run( getMs());
        MathCalculationPosition( getMs());
        if (GetPIDCalibrateionState() <= CALIBRARTION_DONE) {
            setOutput( state.Output);
        } else if (GetPIDCalibrateionState() == CALIBRARTION_hysteresis) {
            pidHysterisis();
        } else if ((GetPIDCalibrateionState() == CALIBRARTION_home_down) ||
                (GetPIDCalibrateionState() == CALIBRARTION_home_up) ||
                (GetPIDCalibrateionState() == CALIBRARTION_home_velocity)) {
            checkLinkHomingStatus();
        }
    }


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
        //println_E("Upper histerisys");
    }
    if (val < getPidStop() && val > getLowerPidHistoresis()){
        val = getLowerPidHistoresis();
      //  println_E("Lower histerisys");
    }
    state.OutputSet = val;
    //
    setOutputLocal( val);
}
