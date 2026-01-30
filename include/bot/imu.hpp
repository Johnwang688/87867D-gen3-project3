#pragma once

#include "vex.h"
#include <cstdint>

namespace bot {
/**
 * @brief Inertial sensor class
 * \addtogroup Inertial
 * 
 * @class Inertial
 * @brief Inertial sensor class
 * @param port port number of inertial sensor
 * @return inertial sensor object
*/

enum RotationDirection : std::int8_t {
    CW = 0,
    CCW = 1,
    Y = 2,
    X = 3
};

class Inertial : public vex::inertial {
    public: 
        Inertial(const int port) : vex::inertial(port) {}
        using vex::inertial::inertial;
        inline void calibrate() {
            vex::inertial::calibrate();
            while (this->isCalibrating()) {
                vex::task::sleep(10);
            }
            this->setHeading(0, vex::degrees);
        }
        inline void reset() {
            this->setHeading(0, vex::degrees);
        }
        inline void set_heading(double heading){
            if (heading < 0) heading += 360;
            if (heading == 360) heading = 0;
            this->setHeading(heading, vex::degrees);
        }
        inline double get_heading() {
            return this->heading(vex::degrees);
        }
        inline double get_heading(RotationDirection direction) {
            return this->get_heading(direction, bot::RotationDirection::Y);
        }
        inline double get_heading(RotationDirection direction, RotationDirection zero) {
            switch (direction) {
                case CW:
                    switch (zero) {
                        case Y:
                            return this->heading(vex::degrees);
                        case X: {
                            double heading = this->heading(vex::degrees) - 90;
                            if (heading > 180) heading -= 360;
                            return heading;
                        }
                        default: 
                            return this->heading(vex::degrees); 
                    }
                case CCW:
                    switch (zero) {
                        case Y: {
                            double heading = 360 - this->heading(vex::degrees);
                            if (heading == 360) heading = 0;
                            return heading;
                        }
                        case X: {
                            double heading = 90 - this->heading(vex::degrees);
                            if (heading < 0) heading += 360;
                            return heading;
                        }
                        default: 
                            return 360 - this->heading(vex::degrees);
                    }
                default:
                    return this->heading(vex::degrees);
            }
        }
        inline double get_roll() {
            return this->orientation(vex::roll, vex::degrees);
        }
        inline double get_pitch() {
            return this->orientation(vex::pitch, vex::degrees);
        }
};
}