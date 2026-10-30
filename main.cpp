
#include "vex.h"

using namespace vex;

brain Brain;
competition Competition;

// Motors and controller
controller Controller1 = controller(primary);

motor LeftMotor = motor(PORT1, ratio18_1, false);
motor RightMotor = motor(PORT2, ratio18_1, true);

// Motors for the cascade lift
motor LiftMotor1 = motor(PORT3, ratio36_1, false);
motor LiftMotor2 = motor(PORT4, ratio36_1, true);

motor_group LiftMotors = motor_group(LiftMotor1, LiftMotor2);

// Switch at the bottom of the lift
limit LiftBottom = limit(Brain.ThreeWirePort.A);

// Lift preset positions (adjust these after testing)
const double LOW_POSITION = 150;
const double MID_POSITION = 300;
const double HIGH_POSITION = 450;

const int LIFT_SPEED = 30;

bool liftHomed = false;

// Used to tell when a button is pressed
bool prevHome = false;
bool prevLow = false;
bool prevMid = false;
bool prevHigh = false;

void pre_auton(void) {
    vexcodeInit();

    LeftMotor.resetPosition();
    RightMotor.resetPosition();

    // Keep the lift from dropping when the motors stop
    LiftMotor1.setStopping(brakeType::hold);
    LiftMotor2.setStopping(brakeType::hold);

    LeftMotor.setStopping(brakeType::coast);
    RightMotor.setStopping(brakeType::coast);

    LiftMotors.stop(brakeType::hold);

    // We need to find the bottom before using presets
    liftHomed = false;
}

void autonomous(void) {
    // Add the autonomous routine here later

    LeftMotor.stop(brakeType::brake);
    RightMotor.stop(brakeType::brake);
    LiftMotors.stop(brakeType::hold);
}

void usercontrol(void) {
    while (true) {

        // Drive with the two joysticks
        int leftSpeed = Controller1.Axis3.position();
        int rightSpeed = Controller1.Axis2.position();

        if (abs(leftSpeed) < 5) {
            LeftMotor.stop(brakeType::coast);
        } else {
            LeftMotor.spin(forward, leftSpeed, percentUnits::pct);
        }

        if (abs(rightSpeed) < 5) {
            RightMotor.stop(brakeType::coast);
        } else {
            RightMotor.spin(forward, rightSpeed, percentUnits::pct);
        }

        // Hold Down to bring the lift to the bottom
        bool homePressed = Controller1.ButtonDown.pressing();

        if (homePressed && !prevHome) {
            liftHomed = false;

            while (Controller1.ButtonDown.pressing() &&
                   !LiftBottom.pressing()) {
                LiftMotors.spin(reverse, 20, percentUnits::pct);
                wait(20, msec);
            }

            LiftMotors.stop(brakeType::hold);

            // Reset the motor positions once the switch is hit
            if (LiftBottom.pressing()) {
                LiftMotor1.resetPosition();
                LiftMotor2.resetPosition();
                liftHomed = true;
            }
        }

        prevHome = homePressed;

        // Buttons A, B, and Y move the lift to different heights
        bool lowPressed = Controller1.ButtonA.pressing();
        bool midPressed = Controller1.ButtonB.pressing();
        bool highPressed = Controller1.ButtonY.pressing();

        // Only move to presets after the lift has been homed
        if (liftHomed) {

            if (lowPressed && !prevLow) {
                LiftMotor1.startRotateTo(
                    LOW_POSITION, rotationUnits::deg,
                    LIFT_SPEED, velocityUnits::pct
                );
                LiftMotor2.startRotateTo(
                    LOW_POSITION, rotationUnits::deg,
                    LIFT_SPEED, velocityUnits::pct
                );
            }

            if (midPressed && !prevMid) {
                LiftMotor1.startRotateTo(
                    MID_POSITION, rotationUnits::deg,
                    LIFT_SPEED, velocityUnits::pct
                );
                LiftMotor2.startRotateTo(
                    MID_POSITION, rotationUnits::deg,
                    LIFT_SPEED, velocityUnits::pct
                );
            }

            if (highPressed && !prevHigh) {
                LiftMotor1.startRotateTo(
                    HIGH_POSITION, rotationUnits::deg,
                    LIFT_SPEED, velocityUnits::pct
                );
                LiftMotor2.startRotateTo(
                    HIGH_POSITION, rotationUnits::deg,
                    LIFT_SPEED, velocityUnits::pct
                );
            }
        }

        // Save the button states for the next loop
        prevLow = lowPressed;
        prevMid = midPressed;
        prevHigh = highPressed;

        wait(20, msec);
    }
}

int main() {
    Competition.autonomous(autonomous);
    Competition.drivercontrol(usercontrol);

    pre_auton();

    while (true) {
        wait(100, msec);
    }
}