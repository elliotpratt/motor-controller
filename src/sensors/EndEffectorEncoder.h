#include "SimpleFOC.h"

class EndEffectorEncoder: public Sensor {
    public:
        EndEffectorEncoder();

        // initialize the sensor hardware
        void init();

        // Get current shaft angle from the sensor hardware, and 
        // return it as a float in radians, in the range 0 to 2PI.
        //  - This method is pure virtual and must be implemented in subclasses.
        //    Calling this method directly does not update the base-class internal fields.
        //    Use update() when calling from outside code.
        float getSensorAngle();
}