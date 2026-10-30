#include "vex.h"

using namespace vex;

brain       Brain;
competition Competition;

// pin configuration
controller  Controller1 = controller(primary);
motor       LeftMotor   = motor(PORT1, ratio18_1, false); // Port 1, 18:1 gear ratio, not reversed
motor       RightMotor  = motor(PORT2, ratio18_1, true);  // Port 2, 18:1 gear ratio, reversed
motor       ArmMotor    = motor(PORT3, ratio36_1, false); // Port 3, 36:1 high-torque gear ratio

// Setup!!! VERY IMPORTATANT!
// Used for sensor calibration, clearing encoder values, and selecting autonomous routines.
void pre_auton(void) {
    vexcodeInit(); // Initializes the VEXcode framework backend
    
    // Reset motor encoders before the match starts
    LeftMotor.resetPosition();
    RightMotor.resetPosition();
    ArmMotor.resetPosition();
}

// Code runs for 15 seconds during a standard VEX tournament match.
void autonomous(void) {
    // Example: Move forward using absolute rotation tracking (highly consistent)
    // startRotateTo non-blocking allows left and right motors to begin spinning together
    LeftMotor.startRotateTo(720, rotationUnits::deg, 50, velocityUnits::pct); 
    RightMotor.rotateTo(720, rotationUnits::deg, 50, velocityUnits::pct); // Blocking command
    
    // Example: Lower arm using exact RPM instead of percentages for thermal reliability
    ArmMotor.rotateTo(90, rotationUnits::deg, 100, velocityUnits::rpm);
}

// Code for User Control
// Code loops continuously for 1 minute and 45 seconds during driver control.
void usercontrol(void) {
    while (true) {
        // Tank Drive Configuration: Map joystick axes directly to motor speeds
        int leftSpeed = Controller1.Axis3.position();  // Left stick vertical
        int rightSpeed = Controller1.Axis2.position(); // Right stick vertical
        
        LeftMotor.spin(forward, leftSpeed, percentUnits::pct);
        RightMotor.spin(forward, rightSpeed, percentUnits::pct);
        
        // Button Controls for Auxillary Subsystems (Arm Control)
        if (Controller1.ButtonR1.pressing()) {
            ArmMotor.spin(forward, 75, percentUnits::pct);  // Raise arm
        } 
        else if (Controller1.ButtonR2.pressing()) {
            ArmMotor.spin(reverse, 75, percentUnits::pct); // Lower arm
        } 
        else {
            ArmMotor.stop(brakeType::hold); // Lock the arm in place when no button is pressed
        }
        
        // Prevent CPU hogging by allowing other background processes to run
        wait(20, msec);
    }
}

// Maps the competition callbacks to lock sections during a formal match.
int main() {
    Competition.autonomous(autonomous);
    Competition.drivercontrol(usercontrol);
    
    pre_auton();
    
    while (true) {
        wait(100, msec);
    }
}