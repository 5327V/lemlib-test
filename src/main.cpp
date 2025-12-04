#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "robot.h"
#include "intake.hpp"



/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy

/**/
void oldSkills(){
    pros::Task antijack([&]() {
        while(1){
            antiJamTask();
            pros::delay(10);
        }
    });
   wing.set_value(true);
   storeIntake();
   matchloader.set_value(true);
   pros::delay(20);
   chassis.moveToPoint(0, 29, 1500, {.maxSpeed = 70});
 
   chassis.turnToHeading(-88, 900, {.maxSpeed = 70});
   chassis.waitUntilDone();
   chassis.tank(90,90);
   pros::delay(700);
   chassis.tank(40,40);
   pros::delay(2000);
   chassis.tank(-80, -80);
   pros::delay(150);
   chassis.moveToPoint(20, 18.5, 1200, {.forwards = false, .maxSpeed = 70, .minSpeed = 50, .earlyExitRange = 0.3});
   chassis.turnToHeading(-88, 900, {.maxSpeed = 70});
   chassis.waitUntilDone();
   chassis.moveToPose(70, 18.5, -88,800, {.forwards = false, .lead = 0.1, .maxSpeed = 90, .minSpeed = 70, .earlyExitRange = 2});
   chassis.moveToPose(85, 18.5, -88, 700, {.forwards = false, .lead = 0.1, .maxSpeed = 70, .minSpeed = 50, .earlyExitRange = 2});
   chassis.turnToHeading(1, 600, {.maxSpeed = 70});
   chassis.waitUntilDone();
   while(fabs(frontSens.get())>= 558){
    chassis.tank(90, 90);
   }
   chassis.tank(0,0);
   chassis.turnToHeading(88, 600, {.maxSpeed = 70});
   chassis.waitUntilDone();
   chassis.tank(-90, -90);
   pros::delay(600);
   chassis.tank(-30,-30);
   scoreLongGoal();
   pros::delay(2000);
   chassis.setPose(74, 31, chassis.getPose().theta);
   pros::delay(100);
   chassis.moveToPoint(92.5, 31, 1200, {.forwards = true, .maxSpeed = 70, .minSpeed = 50, .earlyExitRange = 0.3});
   matchloader.set_value(true);
   chassis.turnToHeading(90, 600, {.maxSpeed = 70});
   chassis.waitUntilDone();
   storeIntake();
   chassis.tank(90,90);
   pros::delay(700);
   chassis.tank(40,40);
   pros::delay(700);
   chassis.tank(-70,-70);
   pros::delay(200);
   chassis.tank(90,90);
   pros::delay(700);
   chassis.tank(40,40);
   pros::delay(1000);

   chassis.tank(-80, -80);
   pros::delay(150);
   chassis.moveToPoint(80, 32, 800, {.forwards = false, .maxSpeed = 70});
   chassis.turnToHeading(88, 600, {.maxSpeed = 70});
   chassis.moveToPoint(72, 32, 800, {.forwards = false, .maxSpeed = 70});
   chassis.waitUntilDone();
   chassis.tank(-30,-30);
   scoreLongGoal();
   pros::delay(2000);
   chassis.setPose(74, 32, chassis.getPose().theta);
   pros::delay(100);
   chassis.moveToPoint(88, 32, 1200, {.forwards = true, .maxSpeed = 70});
   chassis.turnToHeading(180, 800, {.maxSpeed = 70});
   chassis.moveToPoint(97, -62.5, 1200, {.forwards = true, .maxSpeed = 70});
   chassis.waitUntilDone();
   storeIntake();
   while(fabs(frontSens.get())>= 600){
    chassis.tank(90, 90);
   }
   chassis.tank(0,0);
    
   chassis.turnToHeading(90, 600, {.maxSpeed = 70});
   matchloader.set_value(true);
   chassis.waitUntilDone();
   chassis.tank(90,90);
   pros::delay(1000);
   chassis.tank(40,40);
   pros::delay(700);
   chassis.tank(-70,-70);
   pros::delay(200);
   chassis.tank(90,90);
   pros::delay(300);
   chassis.tank(40,40);
   pros::delay(200);
   chassis.setPose(112, -60, chassis.getPose().theta);
   pros::delay(100);
   chassis.tank(-80, -80);
   pros::delay(150);
   stopIntake();
   chassis.moveToPoint(86, -46, 3000, {.forwards = false, .maxSpeed = 70, .minSpeed = 50, .earlyExitRange = 2});
   chassis.moveToPoint(87, -46, 3000, {.forwards = false, .maxSpeed = 70});
   chassis.turnToHeading(85, 600, {.maxSpeed = 70});
   chassis.moveToPoint(50, -49, 3000, {.forwards = false, .maxSpeed = 70, .minSpeed = 50, .earlyExitRange = 2});
   chassis.moveToPoint(13, -49, 1000, {.forwards = false, .maxSpeed = 70});
   chassis.turnToHeading(180, 700, {.maxSpeed = 70});
   chassis.waitUntilDone();
   //chassis.setPose(144 - (rightSens.get()/25.4), chassis.getPose().y, chassis.getPose().theta));
   while(fabs(frontSens.get())>= 568){
    chassis.tank(90, 90);
   }
   chassis.tank(0,0);
   pros::delay(120);
   chassis.turnToHeading(-100, 600, {.maxSpeed = 70});
   chassis.waitUntilDone();
   chassis.tank(-90, -90);
   pros::delay(600);
   chassis.tank(-40,-40);
   scoreLongGoal();
   pros::delay(2000);
   chassis.setPose(20, -72, chassis.getPose().theta);
   pros::delay(100);
   chassis.moveToPoint(0, -72.7, 1200, {.forwards = true, .maxSpeed = 70, .minSpeed = 50, .earlyExitRange = 0.2});
   chassis.turnToHeading(-90, 600, {.maxSpeed = 70});
   chassis.waitUntilDone();
   storeIntake();
   chassis.tank(90,90);
   pros::delay(1000);
   chassis.tank(40,40);
   pros::delay(400);
   chassis.tank(-70,-70);
   pros::delay(200);
   chassis.tank(90,90);
   pros::delay(400);
   chassis.tank(40,40);
   pros::delay(700);
   chassis.tank(-80, -80);
   pros::delay(150);
   chassis.moveToPoint(10, -72, 1200, {.forwards = false, .maxSpeed = 70});
   chassis.turnToHeading(-90, 600, {.maxSpeed = 70});
   chassis.moveToPoint(20, -72, 800, {.forwards = false, .maxSpeed = 70});
   chassis.waitUntilDone();
   scoreLongGoal();
   pros::delay(2000);
   chassis.setPose(23, -72, chassis.getPose().theta);
   pros::delay(100);
   chassis.moveToPoint(5, -72, 1200, {.forwards = true, .maxSpeed = 70});
   chassis.turnToHeading(44, 600, {.maxSpeed = 70});
   chassis.waitUntilDone();
   matchloader.set_value(false);
   storeIntake();
   chassis.moveToPoint(20, -52, 3000, {.forwards = true, .maxSpeed = 40});
   chassis.waitUntilDone();
   matchloader.set_value(true);
   chassis.turnToHeading(0, 600, {.maxSpeed = 70});
   chassis.waitUntilDone();
   matchloader.set_value(false);
   chassis.moveToPoint(19, -3, 4000, {.forwards = true, .maxSpeed = 40});
   chassis.waitUntilDone();
   matchloader.set_value(true);
   pros::delay(100);
   chassis.tank(0,0);
   //chassis.moveToPose(20, -14, -45, 3000, {.forwards = false, .lead = 0.3, .maxSpeed = 70});
   chassis.turnToHeading(-45, 600, {.maxSpeed = 70});
   chassis.waitUntilDone();
   chassis.tank(-90, -90);
   pros::delay(450);
   outtake();
   pros::delay(200);
   chassis.tank(-10,-10);
   scoreMiddleGoal();
   pros::delay(2000);
   chassis.tank(0,0);
   matchloader.set_value(false);
   chassis.moveToPoint(-13, -2, 1200, {.forwards = true, .maxSpeed = 120, .minSpeed = 50, .earlyExitRange = 0.2});
   chassis.swingToHeading(-160, lemlib::DriveSide::LEFT, 900, {.maxSpeed = 100});
   outtake();
   chassis.waitUntilDone();
   outtake();
   chassis.tank(100,100);
   pros::delay(1600);
   chassis.tank(0,0);
   


   


}
void oldSkills2(){
    wing.set_value(true);
    storeIntake();
    chassis.moveToPoint(0, -3, 1200, {.forwards = false, .maxSpeed = 70});
    chassis.waitUntilDone();
    //matchloader.set_value(true);
    chassis.tank(127,127);
    pros::delay(200);
    chassis.tank(100,100);
    matchloader.set_value(false);
     pros::delay(2200);
    chassis.turnToHeading(-3, 600, {.maxSpeed = 100});
    while(fabs(frontSens.get())>= 900){
    chassis.tank(90, 90);
   }
   while(fabs(frontSens.get())>= 850){
    chassis.tank(60, 60);
   }
   while(fabs(frontSens.get())>= 820){
    chassis.tank(10, 10);
   }
   chassis.tank(-1, -1);
   pros::delay(10);
   chassis.setPose(-63, 40,chassis.getPose().theta);
   pros::delay(100);
   chassis.swingToHeading(-86, lemlib::DriveSide::RIGHT, 1000, {.maxSpeed = 60});
   chassis.moveToPoint(-19.9, 24, 1300, {.forwards = false, .maxSpeed = 70});
   chassis.turnToHeading(-45, 600, {.maxSpeed = 70});
   chassis.waitUntilDone();
   matchloader.set_value(true);
   chassis.tank(-70, -70);
   outtake();
   pros::delay(100);
   stopIntake();
   pros::delay(500);
   scoreMiddleGoal();
   chassis.tank(-6, -6);
   pros::delay(2000);
   storeIntake();
   chassis.moveToPoint(-45, 53.5, 2000, {.forwards = true, .maxSpeed = 70});
   pros::delay(300);
   middleGoal.set_value(false);
   chassis.turnToHeading(-90, 600, {.maxSpeed = 70});
   chassis.waitUntilDone();
   chassis.tank(110,110);
   pros::delay(700);
   chassis.tank(40,40);
   pros::delay(1000);
   chassis.setPose(-15, 32, chassis.getPose().theta);
   chassis.tank(-80, -80);
   pros::delay(150);
   chassis.moveToPoint(17, 17.5, 800, {.forwards = false, .maxSpeed = 100, .minSpeed = 50, .earlyExitRange = 0.3});
   chassis.turnToHeading(-88, 900, {.maxSpeed = 70});
   chassis.waitUntilDone();
   chassis.moveToPose(70, 17.5, -88,800, {.forwards = false, .lead = 0.1, .maxSpeed = 100, .minSpeed = 70, .earlyExitRange = 2});
   chassis.moveToPose(81, 17.5, -88, 700, {.forwards = false, .lead = 0.1, .maxSpeed = 90, .minSpeed = 50, .earlyExitRange = 2});
   chassis.turnToHeading(1, 600, {.maxSpeed = 70});
   chassis.waitUntilDone();
   while(fabs(frontSens.get())>= 567){
    chassis.tank(90, 90);
   }
   chassis.tank(0,0);
   chassis.turnToHeading(88, 600, {.maxSpeed = 80});
   chassis.waitUntilDone();
   chassis.tank(-110, -110);
   pros::delay(600);
   chassis.tank(-30,-30);
   scoreLongGoal();
   pros::delay(2000);
   chassis.setPose(74, 31, chassis.getPose().theta);
   chassis.moveToPoint(92.5, 31, 1200, {.forwards = true, .maxSpeed = 70, .minSpeed = 50, .earlyExitRange = 0.3});
   matchloader.set_value(true);
   chassis.turnToHeading(90, 600, {.maxSpeed = 70});
   chassis.waitUntilDone();
   storeIntake();
   chassis.tank(90,90);
   pros::delay(900);
   chassis.tank(40,40);
   pros::delay(700);

   chassis.tank(-80, -80);
   pros::delay(150);
   chassis.moveToPoint(80, 32, 800, {.forwards = false, .maxSpeed = 70});
   chassis.turnToHeading(88, 600, {.maxSpeed = 70});
   chassis.moveToPoint(72, 32, 200, {.forwards = false, .maxSpeed = 70});
   chassis.waitUntilDone();
   chassis.tank(-30,-30);
   scoreLongGoal();
   pros::delay(2000);
   chassis.setPose(74, 32, chassis.getPose().theta);
   matchloader.set_value(false);
   chassis.tank(90,90);
   pros::delay(300);
   chassis.moveToPoint(109, 10, 1200, {.forwards = true, .maxSpeed = 70});
   chassis.swingToHeading(178, lemlib::DriveSide::RIGHT, 1000, {.maxSpeed = 127});
   chassis.waitUntilDone();
   storeIntake();
   chassis.tank(127,127);
    pros::delay(200);
    chassis.tank(110,110);
    matchloader.set_value(false);
     pros::delay(2200);
    while(fabs(frontSens.get())>= 1200){
    chassis.tank(90, 90);
   }
   while(fabs(frontSens.get())>= 1000){
    chassis.tank(60, 60);
   }
   while(fabs(frontSens.get())>= 980){
    chassis.tank(10, 10);
   }
   chassis.tank(-1, -1);
   pros::delay(10);
   chassis.setPose(60,-40,chassis.getPose().theta);
   chassis.turnToHeading(45, 1000, {.maxSpeed = 70});
   chassis.moveToPoint(40,-48, 1200, {.forwards = false, .maxSpeed = 90});
   chassis.turnToHeading(90, 1000, {.maxSpeed = 90});
   chassis.waitUntilDone();
   chassis.tank(-90, -90);
   pros::delay(800);
   chassis.tank(-5, -5);
   scoreLongGoal();
   pros::delay(2000);
   chassis.setPose(26,-49,chassis.getPose().theta);
   pros::delay(10);
   chassis.moveToPoint(52, -48.1, 1200, {.forwards = true, .maxSpeed = 70, .minSpeed = 50, .earlyExitRange = 0.3});
   matchloader.set_value(true);
   chassis.turnToHeading(90, 600, {.maxSpeed = 70});
   chassis.waitUntilDone();
   storeIntake();
   chassis.tank(90,90);
   pros::delay(900);
   chassis.tank(40,40);
   pros::delay(500);
   chassis.moveToPoint(50, -48.1, 1200, {.forwards = false, .maxSpeed = 90, .minSpeed = 50, .earlyExitRange = 0.3});
   chassis.waitUntilDone();
   matchloader.set_value(false);
   chassis.turnToHeading(133, 800, {.maxSpeed = 70});
   chassis.moveToPoint(13, -14, 3000, {.forwards = false, .maxSpeed = 90});
   chassis.waitUntilDone();
   chassis.tank(-80, -80);
   pros::delay(200);
   outtake();
   pros::delay(200);
   stopIntake();
   chassis.tank(-1,-1);
   scoreMiddleGoal();
   pros::delay(1400);
   chassis.tank(100, 100);
   pros::delay(400);
   //chassis.swingToHeading(-130, lemlib::DriveSide::RIGHT, 1000, {.maxSpeed = 127});
   chassis.moveToPoint(-30,-38, 3000, {.forwards = true, .maxSpeed = 90, .minSpeed = 50, .earlyExitRange = 1});
   chassis.turnToHeading(180, 1000, {.maxSpeed = 70});
   chassis.waitUntilDone();
   middleGoal.set_value(false);
   while(fabs(frontSens.get())>= 568){
    chassis.tank(90, 90);
   }
   chassis.tank(0,0);
   chassis.turnToHeading(-90, 1000, {.maxSpeed = 70});
   chassis.waitUntilDone();
   chassis.tank(-90,-90);
   pros::delay(800);
   chassis.tank(-10,-10);
   scoreLongGoal();
   pros::delay(2000);
   chassis.setPose(20, -72, chassis.getPose().theta);
   pros::delay(100);
   matchloader.set_value(true);
   chassis.moveToPoint(0, -72.7, 1200, {.forwards = true, .maxSpeed = 70, .minSpeed = 50, .earlyExitRange = 0.2});
   chassis.turnToHeading(-90, 600, {.maxSpeed = 70});
   chassis.waitUntilDone();
   storeIntake();
   chassis.tank(90,90);
   pros::delay(1000);
   chassis.tank(40,40);
   pros::delay(700);
   chassis.moveToPoint(10, -72, 1200, {.forwards = false, .maxSpeed = 70});
   chassis.turnToHeading(-90, 600, {.maxSpeed = 70});
   chassis.moveToPoint(20, -72, 800, {.forwards = false, .maxSpeed = 70});
   chassis.waitUntilDone();
   scoreLongGoal();
   pros::delay(2000);
   chassis.setPose(-29, -46, chassis.getPose().theta);
   matchloader.set_value(false);
   chassis.tank(127, 127);
   pros::delay(200);
   chassis.moveToPoint(-70,-17, 1200, {.forwards = true, .maxSpeed = 70});
   chassis.turnToHeading(0, 600, {.maxSpeed = 70});
   chassis.waitUntilDone();
   chassis.tank(127, 127);
   pros::delay(600);
   chassis.tank(0,0);
}

void diamondSkills(){
    //testSkills();
    wing.set_value(true);
    chassis.moveToPoint(0, -3, 1200, {.forwards = false, .maxSpeed = 70});
    chassis.waitUntilDone();
    storeIntake();
    //matchloader.set_value(true);
    chassis.tank(127,127);
    pros::delay(200);
    chassis.tank(100,100);
    matchloader.set_value(false);
    pros::delay(2400);
    chassis.tank(0,0);
    chassis.turnToHeading(0, 600, {.maxSpeed = 127});
    chassis.waitUntilDone();
    moveDistanceWFrontDist(29, 700, 127);
    chassis.turnToHeading(0, 600, {.maxSpeed = 127});
    chassis.setPose((-67 + (leftSens.get()/25.4) + 5.5), (67 - (frontSens.get()/25.4) - 7),chassis.getPose().theta);
    pros::delay(10);
    //chassis.swingToHeading(-76, lemlib::DriveSide::RIGHT, 700, {.maxSpeed = 90});
    chassis.swingToHeading(-90, lemlib::DriveSide::RIGHT, 1000, {.maxSpeed = 90});
    chassis.waitUntilDone();
    moveDistanceWFrontDistBack(50, 1500, 127);
    //chassis.moveToPoint(-20.5, 18, 1200, {.forwards = false, .maxSpeed = 70});
    chassis.turnToHeading(-45, 600, {.maxSpeed = 100});
    chassis.waitUntilDone();
   matchloader.set_value(true);
   chassis.tank(-90, -90);
   outtake();
   pros::delay(100);
   stopIntake();
   pros::delay(700);
   scoreMiddleGoal();
   chassis.tank(-6, -6);
   pros::delay(2000);
   storeIntake();
   chassis.moveToPoint(-43, 40, 1400, {.forwards = true, .maxSpeed = 80});
   pros::delay(300);
   middleGoal.set_value(false);
   chassis.turnToHeading(-90, 700, {.maxSpeed = 100});
   chassis.waitUntilDone();
   chassis.tank(90, 90);
   pros::delay(700);
   chassis.tank(40, 40);
   pros::delay(1000);
   chassis.tank(0,0);
   chassis.setPose((-67 + (frontSens.get()/25.4) + 5.5), (67 - (rightSens.get()/25.4) - 7),chassis.getPose().theta);
   pros::delay(10);
   chassis.moveToPoint(-35, 33, 1200, {.forwards = false, .maxSpeed = 90, .minSpeed = 30, .earlyExitRange = 1});
   chassis.turnToHeading(-90, 700, {.maxSpeed = 100});
   matchloader.set_value(false);
   //chassis.moveToPoint(-2, 50, 1200, {.forwards = false, .maxSpeed = 90, .minSpeed = 30, .earlyExitRange = 1});
   chassis.moveToPoint(38, 33, 1600, {.forwards = false, .maxSpeed = 90});

   chassis.turnToHeading(0, 800, {.maxSpeed = 90});
   chassis.waitUntilDone();
   while(fabs(frontSens.get())>= 567){
    chassis.tank(90, 90);
   }
   chassis.turnToHeading(90, 900, {.maxSpeed = 70});
   chassis.waitUntilDone();
   chassis.tank(-90,-90);
   pros::delay(800);
   chassis.tank(-10,-10);
   skillsLongGoalScore();
   pros::delay(2000);
   chassis.setPose(74, 31, chassis.getPose().theta);
   chassis.moveToPoint(92.5, 31, 1200, {.forwards = true, .maxSpeed = 70, .minSpeed = 50, .earlyExitRange = 0.3});
   matchloader.set_value(true);
   chassis.turnToHeading(90, 600, {.maxSpeed = 70});
   chassis.waitUntilDone();
   storeIntake();
   chassis.tank(90,90);
   pros::delay(900);
   chassis.tank(40,40);
   pros::delay(700);

   chassis.tank(-80, -80);
   pros::delay(150);
   chassis.moveToPoint(80, 32, 800, {.forwards = false, .maxSpeed = 70});
   chassis.turnToHeading(88, 600, {.maxSpeed = 70});
   chassis.moveToPoint(72, 32, 200, {.forwards = false, .maxSpeed = 70});
   chassis.waitUntilDone();
   chassis.tank(-30,-30);
   skillsLongGoalScore();
   pros::delay(2000);
   chassis.setPose(74, 32, chassis.getPose().theta);
   matchloader.set_value(false);
   chassis.tank(90,90);
   pros::delay(300);
   chassis.moveToPoint(109, 10, 1200, {.forwards = true, .maxSpeed = 70});
   chassis.swingToHeading(172, lemlib::DriveSide::RIGHT, 1000, {.maxSpeed = 127});
   chassis.waitUntilDone();
   storeIntake();
   chassis.tank(127,127);
   pros::delay(200);
   chassis.tank(105,105);
   matchloader.set_value(false);
   pros::delay(2000);
   chassis.tank(0,0);
   chassis.turnToHeading(178, 600, {.maxSpeed = 100});
   moveDistanceWFrontDist(33, 700, 127);
   chassis.turnToHeading(178, 600, {.maxSpeed = 127});
   chassis.waitUntilDone();
   chassis.setPose((67 - (leftSens.get()/25.4) - 5.5), (-67 + (frontSens.get()/25.4) + 7),chassis.getPose().theta);
   chassis.swingToHeading(-135, lemlib::DriveSide::RIGHT, 700, {.maxSpeed = 90});
   chassis.moveToPoint(40, -46.5, 900, {.forwards = true, .maxSpeed = 127});
   chassis.turnToHeading(90, 1000, {.maxSpeed = 90});
   chassis.waitUntilDone();
   chassis.tank(-90, -90);
   pros::delay(800);
   chassis.tank(-5, -5);
   skillsLongGoalScore();
   pros::delay(2000);
   chassis.setPose(26,-49,chassis.getPose().theta);
   pros::delay(10);
   chassis.moveToPoint(52, -48.1, 1200, {.forwards = true, .maxSpeed = 70, .minSpeed = 50, .earlyExitRange = 0.3});
   matchloader.set_value(true);
   chassis.turnToHeading(90, 600, {.maxSpeed = 70});
   chassis.waitUntilDone();
   storeIntake();
   chassis.tank(90,90);
   pros::delay(900);
   chassis.tank(40,40);
   pros::delay(500);
   chassis.moveToPoint(50, -48.1, 1200, {.forwards = false, .maxSpeed = 90, .minSpeed = 50, .earlyExitRange = 0.3});
   chassis.waitUntilDone();
   matchloader.set_value(false);
   chassis.turnToHeading(133, 800, {.maxSpeed = 70});
   chassis.moveToPoint(13, -14, 3000, {.forwards = false, .maxSpeed = 90});
   chassis.waitUntilDone();
   chassis.tank(-80, -80);
   pros::delay(200);
   outtake();
   pros::delay(200);
   stopIntake();
   chassis.tank(-1,-1);
   scoreMiddleGoal();
   pros::delay(1400);
   /*
   scoreMiddleGoalScore();
   */
   chassis.tank(100, 100);
   pros::delay(400);
   middleGoal.set_value(false);
   chassis.turnToHeading(-90, 400, {.maxSpeed = 70});
   chassis.moveToPoint(-30,-38, 3000, {.forwards = true, .maxSpeed = 90, .minSpeed = 50, .earlyExitRange = 1});
   chassis.turnToHeading(180, 1000, {.maxSpeed = 70});
   chassis.waitUntilDone();
   while(fabs(frontSens.get())>= 568){
    chassis.tank(90, 90);
   }
   chassis.tank(0,0);
   chassis.turnToHeading(-91, 1000, {.maxSpeed = 70});
   chassis.waitUntilDone();
   chassis.tank(-90,-90);
   pros::delay(800);
   chassis.tank(-10,-10);
   scoreLongGoal();
   pros::delay(2000);
   chassis.setPose(20, -72, chassis.getPose().theta);
   pros::delay(100);
   matchloader.set_value(true);
   chassis.moveToPoint(0, -72.7, 1200, {.forwards = true, .maxSpeed = 70, .minSpeed = 50, .earlyExitRange = 0.2});
   chassis.turnToHeading(-90, 600, {.maxSpeed = 70});
   chassis.waitUntilDone();
   storeIntake();
   chassis.tank(90,90);
   pros::delay(1000);
   chassis.tank(40,40);
   pros::delay(700);
   chassis.moveToPoint(10, -72, 1200, {.forwards = false, .maxSpeed = 70});
   chassis.turnToHeading(-90, 600, {.maxSpeed = 70});
   chassis.moveToPoint(20, -72, 800, {.forwards = false, .maxSpeed = 70});
   chassis.waitUntilDone();
   skillsLongGoalScore();
   pros::delay(2000);
   chassis.setPose(-29, -46, chassis.getPose().theta);
   matchloader.set_value(false);
   chassis.tank(127, 127);
   pros::delay(200);
   chassis.moveToPoint(-70,-17, 1200, {.forwards = true, .maxSpeed = 70});
   chassis.swingToHeading(-5, lemlib::DriveSide::RIGHT, 700, {.maxSpeed = 100});
   chassis.waitUntilDone();
   chassis.tank(127, 127);
   pros::delay(600);
   chassis.tank(0,0);
}
void autonomous() {
    diamondSkills();  
}

/**
 * Runs in driver control
 */
bool calibrate = false;
void opcontrol() {
    if(calibrate){
        pros::delay(500);
        diamondSkills();
    } else{
        while (true) {
            chassis.arcade(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
                           controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
            pros::delay(20);
        }
    }
}
