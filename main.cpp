#include "main.h"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "api.h"
#include "okapi/api.hpp"
#include "pros/api_legacy.h"
#include <memory>
using namespace okapi::literals;
using namespace okapi;
//pros::Motor catapult(20, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_COUNTS);
//pros::Motor intake(17, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);

std::shared_ptr<okapi::ChassisController> drive =
okapi::ChassisControllerBuilder()
	.withMotors(
		{4,8,-1}, 
		{-5,-6,2}
	)
	.withDimensions(okapi::AbstractMotor::gearset::blue, {{3.25_in, 14_in}, okapi::imev5BlueTPR})
	.build();
	//.withOdometry()
	//.buildOdometry()
	/*.withGains(
		{0.001, 0, 0.001}, //distance gains
		{0.001, 0, 0.001}, //turn gains 
		{0,001, 0, 0.001} //angle gains 
	)
	.withDerivativeFilters(
		std::make_unique<AverageFilter<3>>(), //distance filter
		std::make_unique<AverageFilter<3>>(), //turn filter
		std::make_unique<AverageFilter<3>>() //angle filter
	)
	.withMaxVelocity(127)


	const double catakP = 0.001; 
	const double catakI= 0.0001; 
	const double catakD = 0.0001; 
	const double catakP = 0.001; 

	std::shared_ptr<Controller> driveController = 
  		okapi::ChassisControllerBuilder()
    	.withMotors(
		{4,8,-1},
		{-5,-6,2}
		)
    	.withDimensions(okapi::AbstractMotor::gearset::blue, {{3.25, 11.5}, okapi::imev5BlueTPR})
    	.build();

	std::shared_ptr<okapi::AsyncPositionController<double, double>> cataController = 
		okapi::AsyncPosControllerBuilder()
			.withMotor(20)
			.withGains ({catakP, catakD, catakI, })
			.build(); */






pros::Motor driveLeftfront(4, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
 
pros::Motor driveeLftmiddle(8, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
//middle=top motor 
pros::Motor driveLeftback(1, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor driveRightfront(5, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor driveRightmiddle(6, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
//middle=top motor 
pros::Motor driveRightback(2, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);

//Controller
pros::Controller controller (pros::E_CONTROLLER_MASTER);

pros::Motor catapult(20, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor intake(17, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_COUNTS);
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

//Test 
	//std::shared_ptr<okapi::ChassisController> driveController = 
	//		okapi::ChassisControllerBuilder ()
				//.withMotors({1,2}, {4,5}, {6,8});
				//.withDimensions(okapi::AbstractMotor::gearset::blue, {{4_in, 11.5_in}}, {imev5GreenTPR})
				//.build();

	//const double liftkP = 0.001;
	//const double liftkI = 0.0001;
	//const double liftkD = 0.0001
	/*
void chas_move(float lspeed, float rspeed){
	driveLeftfront.move(lspeed);
	driveLeftback.move(lspeed);
	driveeLftmiddle.move(lspeed);
	driveRightback.move(rspeed);
	driveRightfront.move(rspeed);
	driveRightmiddle.move(rspeed);
}

void driveFor(int target, int voltage){
	float encoader_av;
	int direction;
	void resetDriveEncoders ();
	chas_move(voltage, voltage);
	direction = target/abs(target);
	while(true){
		encoader_av = (driveLeftback.get_position() + driveRightback.get_position())/2;
		if(abs(encoader_av) > abs(target)){
			chas_move(0,0);
			break;
		}
	}
	while(fabs(target - encoader_av > 10)){
		encoader_av = (driveLeftback.get_position() + driveRightback.get_position())/2;
		chas_move(10 * direction * -1, 10 * direction * -1);
	}
	chas_move(0,0);
}


 
//INERTIAL SENSOR
	void turnFor(int target, int speed){
		int imu_start = imu.get_rotation();
		float position;
		chas_move(-speed, speed);
		while(true){
			position = imu.get_rotation() - imu_start;
			if (abs(target - position) <= 1.5){
				chas_move(0,0);
				break;
			}
		}
	}

*/ 
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 
 */   


void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "8838B Century");
	pros::lcd::register_btn1_cb(on_center_button);


}
	/*
	driveLeftback.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	driveLeftfront.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	driveeLftmiddle.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	driveRightback.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	driveRightfront.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	driveRightmiddle.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

	catapult.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	intake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	//pros::ADIDigitalOut piston (pros::E_ADI_DIGITAL_OUT); 
	
}*/

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() { 


}
/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
 
void autonomous() {
	

	std::shared_ptr<ChassisController> driveController =
  	ChassisControllerBuilder()
    .withMotors({4,1}, {5,2}, {8,6})
    // Green gearset, 3.25 in wheel diam, 14 in wheel track
    .withDimensions(AbstractMotor::gearset::blue, {{3.25_in, 14_in}, imev5BlueTPR})
    .build();

	const double catakP = 0.01;
	const double catakI = 0.001; 
	const double catakD = 0.001; 

	std::shared_ptr<AsyncPositionController<double, double>> cataController = 
  	AsyncPosControllerBuilder()
    .withMotor(20) // cata motor port 20
    .withGains({catakP, catakI, catakD})
    .build();

	
		
	driveController->moveDistanceAsync(10_in);
	intake.move(127); 
	cataController->setTarget(200);
	driveController->turnAngle(90_deg); 
	intake.move(127);
	cataController->setTarget(200); 
	driveController->moveDistanceAsync(10_in);
	



	//driveFor(1000,100);
	//intake.move(127);
	//driveFor(1000, 100);
	//turnFor(90, 70);
	//driveFor(1500,100);
	//intake.move(127);
	//turnFor(90, 100);
	//driveFor(1000,100);
	//turnFor(90,70);
	//driveFor(1000,100);
	//intake.move(127);
	//driveFor(-1000, 100);

	//driveController->moveDistanceAsync(10_in);
	//cataController->setTarget(200);

	//driveController->waitUntilSettled();


	//pros::Motor left_wheels (left_wheels);
	//pros::Motor right_wheels (right_wheels);
}

//expansion 
	//pros::ADIDigitalOut piston (pros::E_ADI_DIGITAL_OUT);
		//piston.set_value(true);
		//pros::(1000);
		//piston.set_value(false);


/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever t  he robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	Controller controller;
	while (true) {
    // Tank drive, left and right. 
    drive->getModel()->tank(controller.getAnalog(ControllerAnalog::leftY),
                            controller.getAnalog(ControllerAnalog::rightY));
    pros::delay(10);
}
	ADIButton cataLimitSwitch('H');
	Motor cataMotor (-20);
	ControllerButton cataUpButton(ControllerDigital::A);
	ControllerButton cataDownButton(ControllerDigital::B);

	//auto run button 
	 ControllerButton runAutoButton(ControllerDigital::X);

	if (cataLimitSwitch.isPressed()) {
    	cataMotor.moveVelocity(0);
	} else {
		 if (cataUpButton.isPressed()) {
       		cataMotor.moveVoltage(12000);
    } else if (cataDownButton.isPressed()) {
        	cataMotor.moveVoltage(-12000);
    } else {
    		cataMotor.moveVoltage(0);
    }
    // Normal arm control
	}
	if (runAutoButton.changedToPressed()) {
            // Drive the robot in a square pattern using closed-loop control
            for (int i = 0; i < 4; i++) {
                drive->moveDistance(12_in); // Drive forward 12 inches
                drive->turnAngle(90_deg);   // Turn in place 90 degrees
				drive->moveDistanceAsync(10_in);
				
            }
        }

	pros::delay(10);
	}

	//auton selector
	/*void (*autonSelector())() 
	{
		int numAutons = autons.size()-1;
		int selectedAut = 0;
		bool autSelected = false;
		int color = 1;
		int driver = 0;

		while(1)
		{
			if(glb::controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
		{
			if (selectedAut != numAutons)
		{	
			selectedAut++;
		}
		else 
		{
			selectedAut = 0;
		}
	}}}

			if(glb::controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT))
			{
				if(selectedAut != 0)
			}*/
		


    	
	/*while (true) {
        //intake code
        if(controller.get_digital(DIGITAL_R1)){
            intake.move(127);
        }
        else if(controller.get_digital(DIGITAL_R2)){
            intake.move(-127);
        }
        else{ 
            intake.move(0);
        }
	*/
      
		/*controller.clear();
		driveLeftback.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		driveLeftfront.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		driveeLftmiddle.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		driveRightback.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		driveRightfront.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		driveRightmiddle.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

		catapult.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		intake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); 

		int count = 0;
		int detectedTime = 0;
		bool intake_power = false;
		int intake_direction = 1;

		driveLeftfront.move(127);
		driveLeftback.move(127);
		driveeLftmiddle.move(127);
		driveRightfront.move(127);
		driveRightback.move(127);
		driveRightmiddle.move(127);
		pros::delay(20);*/

		/*int lAxis = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int rAxis = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

		driveLeftfront.move(lAxis);
		driveLeftback.move(lAxis);
		driveeLftmiddle.move(lAxis);
		driveRightfront.move(rAxis);
		driveRightback.move(rAxis);
		driveRightmiddle.move(rAxis);

	
		pros::delay(20); 
	}
	}*/

	//{
	//while(true){
	//	pros::delay(10); 
	//}
	//pros::Controller master(pros::E_CONTROLLER_MASTER);
	//pros::Motor left_mtr(1);
	//pros::Motor right_mtr(2);

	/*while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);
		int left = master.get_analog(ANALOG_LEFT_Y);
		int right = master.get_analog(ANALOG_RIGHT_Y);

		left_mtr = left;
		right_mtr = right;
		pros::delay(20);
	}}*/