# Vex v5 program for 2025-2026 PUSHBACK. 
This is a competition control system featuring tank drive and PID-based motion control algorithms for a ruiguan bot. 

## features 

- ## tank drive code found in main.cpp -> usercontrol(). motor movement commands in drivetrain.cpp -> tank_drive(double, double)
- ## drive curve found in bot/utils.hpp -> math::curve(double). drive curve uses A * B^x - C. 
Default values are A = 2.0, B = 1.05, C = -A. To edit values, go to bot/constants.hpp and find constexpr double A. Clean the project build before reuploading to ensure that header files are recompiled with the new drive curve constants. 
- ## PID controllers use constants found in bot/constants.hpp. find constexpr double DRIVE_KP. clean project build and rebuild and upload to ensure header files are recompiled with new PID gains. 
- ## driving and turning controllers are found in drivetrain.cpp -> drive_for(double, double, double, double), drivetrain.cpp -> turn_to_heading(double, double, double). 

