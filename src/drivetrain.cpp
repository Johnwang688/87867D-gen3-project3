#include "drivetrain.hpp"

namespace bot {

Drivetrain::Drivetrain(vex::motor_group& left_dt, 
    vex::motor_group& right_dt, bot::Inertial& imu):
    _left_dt(left_dt),
    _right_dt(right_dt),
    _imu(imu),
    _wheel_diameter(WHEEL_DIAMETER),
    _track_width(TRACK_WIDTH),
    _gear_ratio(GEAR_RATIO),
    _max_voltage(MAX_VOLTAGE),
    _max_accel(MAX_ACCEL),
    _drive_pid(DRIVE_KP, DRIVE_KI, DRIVE_KD),
    _heading_pid(HEADING_KP, HEADING_KI, HEADING_KD),
    _turn_pid(TURN_KP, TURN_KI, TURN_KD),
    _left_arc_pid(ARC_KP, ARC_KI, ARC_KD),
    _right_arc_pid(ARC_KP, ARC_KI, ARC_KD)
     {}

void Drivetrain::tank_drive(double left_speed, double right_speed) {
    left_speed = math::clamp(left_speed, -100, 100);
    right_speed = math::clamp(right_speed, -100, 100);
    _left_dt.spin(vex::forward, left_speed * 0.11, vex::volt);
    _right_dt.spin(vex::forward, right_speed * 0.11, vex::volt);
}

void Drivetrain::arcade_drive(double fwd, double turn) {
    fwd = math::clamp(fwd, -100, 100);
    turn = math::clamp(turn, -100, 100);
    double left_speed = fwd + turn;
    double right_speed = fwd - turn;
    tank_drive(left_speed, right_speed);
}

void Drivetrain::stop() {
    _left_dt.stop();
    _right_dt.stop();
}

void Drivetrain::brake() {
    _left_dt.setStopping(vex::brakeType::brake);
    _right_dt.setStopping(vex::brakeType::brake);
}

void Drivetrain::coast() {
    _left_dt.setStopping(vex::brakeType::coast);
    _right_dt.setStopping(vex::brakeType::coast);
}

void Drivetrain::hold() {
    _left_dt.setStopping(vex::brakeType::hold);
    _right_dt.setStopping(vex::brakeType::hold);
}

void Drivetrain::drive_for(double distance, double timeout, double speed_limit, double target_heading) {
    double start_time = bot::Brain.Timer.time(vex::msec);
    int settle = 0;
    double heading_error, heading_correction, speed, current_pos, left_speed, right_speed;
    _left_dt.setPosition(0, vex::degrees);
    _right_dt.setPosition(0, vex::degrees);
    _drive_pid.reset();
    _heading_pid.reset();
    double dist = helpers::mmToDegrees(distance);
    while (timeout > 0 && bot::Brain.Timer.time(vex::msec) - start_time < timeout) {
        heading_error = helpers::angular_difference(_imu.get_heading(bot::CCW, bot::X), target_heading);
        heading_correction = _heading_pid.compute(heading_error, 0.0, 0.02);
        current_pos = (_left_dt.position(vex::degrees) + _right_dt.position(vex::degrees)) / 2.0;
        speed = _drive_pid.compute(dist, current_pos, 0.02);
        left_speed = speed + heading_correction;
        right_speed = speed - heading_correction;
        left_speed = math::clamp(left_speed, -speed_limit, speed_limit);
        right_speed = math::clamp(right_speed, -speed_limit, speed_limit);
        left_speed *= (_max_voltage / 100.0);
        right_speed *= (_max_voltage / 100.0);
        _left_dt.spin(forward, left_speed, vex::voltageUnits::volt);
        _right_dt.spin(forward, right_speed, vex::voltageUnits::volt);
        if (std::abs(dist - current_pos) < 25
        && std::abs(heading_error) < 1.0) {
            settle++;
        } else {
            settle = 0;
        }
        vex::task::sleep(20);
        if (settle >= 3) break;
    }
    _left_dt.spin(vex::forward, 0, vex::voltageUnits::volt);
    _right_dt.spin(vex::forward, 0, vex::voltageUnits::volt);
}

void Drivetrain::turn_to_heading(double heading, double timeout, double speed_limit) {
    double start_time = bot::Brain.Timer.time(vex::msec);
    int settle_count = 0;
    _left_dt.setPosition(0, vex::degrees);
    _right_dt.setPosition(0, vex::degrees);
    _turn_pid.reset();
    double current_heading, heading_error, output, left_speed, right_speed;
    while (bot::Brain.Timer.time(vex::msec) - start_time < timeout) {
        current_heading = _imu.get_heading(bot::CCW, bot::X);
        heading_error = helpers::angular_difference(current_heading, heading);
        output = _turn_pid.compute(heading_error, 0.0, 0.02);
        output = math::clamp(output, -speed_limit, speed_limit);
        left_speed = output * 0.4;
        right_speed = -output * 0.4;
        _left_dt.spin(vex::forward, left_speed, vex::voltageUnits::volt);
        _right_dt.spin(vex::forward, right_speed, vex::voltageUnits::volt);
        if (std::abs(heading_error) < 1.0) {
            settle_count++;
        } else {
            settle_count = 0;
        }
        vex::task::sleep(20);
        if (settle_count >= 3) break;
    }
    brake();
    _left_dt.stop();
    _right_dt.stop();
    coast();
}

}