#include "EndEffectorEncoder.h"

EndEffectorEncoder::EndEffectorEncoder() {
    // define all the necessary sensor variables
    // or leave empty if not necessary
}

EndEffectorEncoder::init(){
    // setup all the needed sensor hardware 
    // for example
    sensor.hardwareInit();
}

EndEffectorEncoder::getSensorAngle(){
    // Get current shaft angle from the sensor hardware, and 
    // return it as a float in radians, in the range 0 to 2PI.
    return sensor.read() ;
}