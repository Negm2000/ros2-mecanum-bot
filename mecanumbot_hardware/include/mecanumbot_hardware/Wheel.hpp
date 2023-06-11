#ifndef __MEC_WHEEL_HPP_
#define __MEC_WHEEL_HPP_

#include <string>
#include <cmath>

class Wheel {

public:

    std::string name = "";
    int enc = 0; // encoder value
    double vel = 0; // velocity (state: angular velocity)
    double pos = 0; // position (state: angle)
    double cmd = 0; // command value (command: velocity)
    double rads_per_count = 0; // conversion factor from encoder ticks to radians
    
    Wheel() = default;
    void setup(const std::string& name, int counts_per_rev) {this->name = name; rads_per_count = 2*M_PI/counts_per_rev;} // setup function to set name and conversion factor
    double getEncoderAngle() const {return enc*rads_per_count;}
};

#endif // __MEC_WHEEL_HPP_