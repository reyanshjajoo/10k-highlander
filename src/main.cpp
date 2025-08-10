#include "main.h"
#include "lemlib/api.hpp"

pros::Controller controller(pros::E_CONTROLLER_MASTER);


pros::MotorGroup leftMotors({-11, -12, -13}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({18, 19, 20}, pros::MotorGearset::blue);

pros::Motor firstStageIntake(-17, pros::MotorGearset::blue);
pros::Motor basketRoller(2, pros::MotorGearset::green);
pros::Motor hood(10, pros::MotorGearset::blue);

pros::Imu imu(14);


// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors,
                              &rightMotors,
                              11.2, // track width
                              lemlib::Omniwheel::NEW_275,
                              450,
                              2 // horizontal drift
);

lemlib::ControllerSettings linearController(10, // kP
                                            0, // kI
                                            3, // kD
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

lemlib::ControllerSettings angularController(2, // kP
                                             0, // kI
                                             10, // kD
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel
                            nullptr,
                            nullptr, // horizontal tracking wheel
                            nullptr, 
                            &imu // inertial sensor
);


lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors);

// intake bools
bool runningIntake = false;
bool runningBasket = false;
bool outtake = false;
bool midGoal = false;

void toggle(){
    while (true) {
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            runningIntake = !runningIntake;
            if (runningBasket){
                runningIntake = true;
            }
            runningBasket = false;
            pros::delay(400);
        }
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            if (midGoal != true){
                runningBasket = !runningBasket;
                runningIntake = runningBasket;
            }
            midGoal = false;
            pros::delay(400);
        }
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            if (midGoal != false){
                runningBasket = !runningBasket;
                runningIntake = runningBasket;
            }
            midGoal = true;
            pros::delay(400);
        }
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
            outtake = true;
        } else {
            outtake = false;
        }
        pros::delay(20);
    }
}

void intake_control() {
    int hoodVel = 600;
    while (true) {
        if (runningIntake && !outtake && !runningBasket) {
            firstStageIntake.move_velocity(600);
            hood.move_velocity(600);
            basketRoller.brake();
        } else if (outtake && !runningBasket) {
            firstStageIntake.move_velocity(-600);
            hood.move_velocity(-600);
            basketRoller.brake();
        } else if (!runningIntake && !outtake && !runningBasket) {
            firstStageIntake.move_velocity(0);
            hood.move_velocity(0);
            basketRoller.brake();
        }

        if (runningBasket && runningIntake && !outtake && !midGoal) {
            firstStageIntake.move_velocity(600);
            basketRoller.move_velocity(200);
            hood.move_velocity(600);
        } else if (runningBasket && runningIntake && !outtake && midGoal) {
            basketRoller.move_velocity(200);
            firstStageIntake.move_velocity(600);
            hood.move_velocity(-600);
        } else if (runningBasket && outtake) {
            basketRoller.move_velocity(-200);
            firstStageIntake.move_velocity(-600);
            hood.move_velocity(-600);
        }

        
        pros::delay(20);
    }
}

void initialize() {
    pros::lcd::initialize();
    chassis.calibrate();
    basketRoller.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    pros::Task screenTask([&]() {
        while (true) {
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading

			lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            pros::delay(50);
        }
    });
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
    pros::Task intake_task(intake_control);
    pros::Task toggle_task(toggle);
    while (true) {
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        chassis.tank(leftY, rightY);
        pros::delay(10);
    }
}