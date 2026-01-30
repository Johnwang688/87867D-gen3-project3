#pragma once

#include "bot/bot.hpp"
#include "bot/imu.hpp"
#include "bot/pid.hpp"
#include "location.hpp"

namespace bot {

class Drivetrain {
    public:
        Drivetrain(
            vex::motor_group& left_dt,
            vex::motor_group& right_dt,
            bot::Inertial& imu
        );
        void tank_drive(double left_speed, double right_speed);
        void arcade_drive(double fwd, double turn);
        void stop();
        void brake();
        void coast();
        void hold();

        void drive_for(double distance, double timeout, double speed_limit, double target_heading);
        void turn_to_heading(double heading, double timeout, double speed_limit);


    private:
        vex::motor_group& _left_dt;
        vex::motor_group& _right_dt;
        bot::Inertial& _imu;
        double _wheel_diameter;
        double _track_width;
        double _gear_ratio;
        double _max_voltage;
        double _max_accel;
        PID _drive_pid;
        PID _heading_pid;
        PID _turn_pid;
        PID _left_arc_pid;
        PID _right_arc_pid;
};

}