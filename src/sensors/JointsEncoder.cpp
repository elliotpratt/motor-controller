#include "JointsEncoder.h"

JointsEncoder::JointsEncoder(int pin, int resolution) {
    // define all the necessary sensor variables
    // or leave empty if not necessary
}

JointsEncoder::init(){
    // setup all the needed sensor hardware 
    // for example
    sensor.hardwareInit();
}

JointsEncoder::getSensorAngle(){
    // Get current shaft angle from the sensor hardware, and 
    // return it as a float in radians, in the range 0 to 2PI.
    return sensor.read() ;
}