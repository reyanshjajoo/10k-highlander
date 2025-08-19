#include "main.h"
#include "lemlib/api.hpp"

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup leftMotors({-11, -12, -13}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({18, 19, 20}, pros::MotorGearset::blue);

pros::Motor firstStageIntake(-17, pros::MotorGearset::blue);
pros::Motor basketRoller(2, pros::MotorGearset::blue);
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

lemlib::ControllerSettings linearController(10,  // kP
                                            0,   // kI
                                            3,   // kD
                                            3,   // anti windup
                                            1,   // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3,   // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20   // maximum acceleration (slew)
);

lemlib::ControllerSettings angularController(2,   // kP
                                             0,   // kI
                                             10,  // kD
                                             3,   // anti windup
                                             1,   // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3,   // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0    // maximum acceleration (slew)
);

lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel
                            nullptr,
                            nullptr, // horizontal tracking wheel
                            nullptr,
                            &imu // inertial sensor
);

lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors);

enum class BallColor
{
    Red,
    Blue,
    Unknown
};

bool colorSort = true;

// pros::Optical optical(8);   // <--- DISABLED SENSOR
pros::Optical optical2(9);              // second optical sensor
BallColor targetColor = BallColor::Red; // default target color

struct HueRange
{
    double min;
    double max;
    bool contains(double hue) const
    {
        if (min < max)
        {
            return hue >= min && hue <= max;
        }
        else
        {
            return hue >= min || hue <= max;
        }
    }
};

const HueRange RED_RANGE{0.0, 26.0};
const HueRange BLUE_RANGE{200.0, 250.0};
BallColor ballColor = BallColor::Unknown; // initially

BallColor identifyColor() {
    const int PROX_THRESHOLD = 80;

    // int prox1 = optical.get_proximity();
    int prox2 = optical2.get_proximity();

    // if (prox1 < PROX_THRESHOLD && prox2 < PROX_THRESHOLD) {
    if (prox2 < PROX_THRESHOLD || !colorSort) {
        return BallColor::Unknown;
    }

    // double hue1 = optical.get_hue();
    double hue2 = optical2.get_hue();

    // if (RED_RANGE.contains(hue1) || RED_RANGE.contains(hue2)) {
    if (RED_RANGE.contains(hue2)) {
        return BallColor::Red;
    }
    // if (BLUE_RANGE.contains(hue1) || BLUE_RANGE.contains(hue2)) {
    if (BLUE_RANGE.contains(hue2)) {
        return BallColor::Blue;
    }
    return BallColor::Unknown;
}



void colorSortTask()
{
    while (true)
    {
        ballColor = identifyColor();
        pros::delay(20);
    }
}

pros::ADIDigitalOut basket('A');
bool basketExtended = false;

pros::ADIDigitalOut matchload('B');
bool matchloadOn = false;

enum class Mode
{
    Idle,
    IntakeToBasket,
    ScoreTop,
    ScoreMid,
    ScoreLow,
    Unjam,
    BottomLoad,
    ejectBall
};

Mode currentMode = Mode::Idle;

void handleL1Press()
{
    if (ballColor == targetColor)
    {
        currentMode = Mode::ejectBall;
    }
    else
    {
        if (currentMode == Mode::IntakeToBasket)
        {
            currentMode = Mode::Idle;
        }
        else
        {
            currentMode = Mode::IntakeToBasket;
        }
    }
}

void handleR1Press()
{ currentMode = (currentMode == Mode::ScoreTop) ? Mode::Idle : Mode::ScoreTop; }

void handleR2Press()
{ currentMode = (currentMode == Mode::ScoreMid) ? Mode::Idle : Mode::ScoreMid; }

void handleBPress()
{ currentMode = (currentMode == Mode::ScoreLow) ? Mode::Idle : Mode::ScoreLow; }

void handleRightPress()
{ currentMode = (currentMode == Mode::BottomLoad) ? Mode::Idle : Mode::BottomLoad; }

void handleLeftPress(){ 
    colorSort = !colorSort;
    controller.rumble(".");
}

void handleL2Held(bool pressed)
{
    if (pressed) currentMode = Mode::Unjam;
    else if (currentMode == Mode::Unjam) currentMode = Mode::Idle;
}

void checkButtons()
{
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1))
        handleL1Press();
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1))
        handleR1Press();
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2))
        handleR2Press();
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
        handleBPress();
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
        handleRightPress();
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT))
        handleLeftPress();
    handleL2Held(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2));
}

void intakeControl()
{
    while (true)
    {
        switch (currentMode)
        {
        case Mode::Idle:
            firstStageIntake.move_velocity(0);
            hood.move_velocity(0);
            basketRoller.brake();
            chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
            break;

        case Mode::IntakeToBasket:
            if (ballColor == targetColor) {
                currentMode = Mode::ejectBall;
                break;
            }
            firstStageIntake.move_velocity(600);
            hood.move_velocity(600);
            basketRoller.brake();
            basketExtended = false;
            basket.set_value(basketExtended);
            chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
            break;

        case Mode::ScoreTop:
            firstStageIntake.move_velocity(600);
            basketRoller.move_velocity(600);
            hood.move_velocity(600);
            pros::delay(250);
            basketExtended = true;
            basket.set_value(basketExtended);
            chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
            break;

        case Mode::ScoreMid:
            basketRoller.move_velocity(600);
            firstStageIntake.move_velocity(0);
            hood.move_velocity(-600);
            basketExtended = true;
            basket.set_value(basketExtended);
            chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
            break;

        case Mode::ScoreLow:
            basketRoller.move_velocity(600);
            firstStageIntake.move_velocity(-400);
            hood.move_velocity(0);
            basketExtended = true;
            basket.set_value(basketExtended);
            chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
            break;

        case Mode::Unjam:
            basketRoller.move_velocity(-600);
            firstStageIntake.move_velocity(-600);
            hood.move_velocity(-600);
            basketExtended = false;
            basket.set_value(basketExtended);
            chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
            break;

        case Mode::BottomLoad:
            firstStageIntake.move_velocity(600);
            hood.move_velocity(0);
            basketRoller.move_velocity(-600);
            basketExtended = false;
            basket.set_value(basketExtended);
            chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
            break;

        case Mode::ejectBall:
            firstStageIntake.move_velocity(600);
            hood.move_velocity(-600);
            basketRoller.move_velocity(0);
            chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
            pros::delay(100);
            currentMode = Mode::IntakeToBasket;
            break;
        }
        pros::delay(20);
    }
}

void toggleTask()
{
    while (true)
    {
        checkButtons();
        pros::delay(20);
    }
}

void pneumaticControl()
{
    while (true)
    {
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN))
        {
            matchloadOn = !matchloadOn;
            matchload.set_value(matchloadOn);
        }
        pros::delay(20);
    }
}

void displayStatusTask() {
    while (true) {
        controller.set_text(0, 0, colorSort ? "Color Sort: ON " : "Color Sort: OFF");
        pros::delay(20);
    }
}


void initialize()
{
    pros::lcd::initialize();
    controller.clear();
    // optical.set_led_pwm(100);
    optical2.set_led_pwm(100);
    // optical.set_integration_time(50);
    optical2.set_integration_time(50);
    chassis.calibrate();
    basketRoller.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    pros::Task screenTask([&]()
                          {
        while (true) {
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // pros::lcd::print(3, "Hue Color 1: %f", optical.get_hue());
            pros::lcd::print(4, "Hue Color 2: %f", optical2.get_hue());
            // pros::lcd::print(5, "Proximity 1: %d", optical.get_proximity());
            pros::lcd::print(6, "Proximity 2: %d", optical2.get_proximity());
            pros::lcd::print(7, "Ball Color: %s", ballColor == BallColor::Red ? "Red" : (ballColor == BallColor::Blue ? "Blue" : "Unknown"));
			lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            pros::delay(50);
        } });
}

void disabled() {}
void competition_initialize() {}
void autonomous() {}

void opcontrol() {
    pros::Task intake_task(intakeControl);
    pros::Task toggle_task(toggleTask);
    pros::Task pneumatic_task(pneumaticControl);
    pros::Task color_task(colorSortTask);
    pros::Task displayTask(displayStatusTask);

    while (true) {
        int leftY  = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        
        chassis.tank(leftY, rightY);

        pros::delay(10);
    }
}
