#include "api.h"
#include <ctime>
#include <cmath>
#include <thread>
#include "robot.h"

double intakeSpeed = 0;
double hoodSpeed = 0;

void manualIntake(double intake_speed, double hood_speed){
    intakeSpeed = intake_speed;
    hoodSpeed = hood_speed;
    intake.move(intakeSpeed);
    hood.move(hoodSpeed);
}
void stopIntake() {
    intakeSpeed = 0;
    hoodSpeed = 0;
    intake.move(0);
    hood.move(0);
}

void scoreLongGoal(){
    middleGoal.set_value(false);
    intakeSpeed = 127;
    hoodSpeed = 127;
    intake.move(127);
    hood.move(127);
}

void scoreMiddleGoal(){
    middleGoal.set_value(true);
    intakeSpeed = 127;
    hoodSpeed = 84;
    intake.move(127);
    hood.move(84);
}

void outtake(){
    intakeSpeed = -127;
    hoodSpeed = -127;
    intake.move(-127);
    hood.move(-127);
}
void storeIntake(){
    intakeSpeed = 127;
    hoodSpeed = -50;
    intake.move(127);
    hood.move(-50);
}

void skillsLongGoalScore(){
    outtake();
    pros::delay(100);
    scoreLongGoal();
}

void scoreMiddleGoalScore(){
    outtake();
    pros::delay(150);
    while(!(ballSens.get_hue() > 180 && ballSens.get_hue() < 210)){
        scoreMiddleGoal();
    }
    stopIntake();
}

void antiJamTask(){
        if(intakeSpeed != 0 && fabs(intake.get_actual_velocity()) < 5 && intake.get_torque() > 0.5){
            intake.move(-intakeSpeed);
            pros::delay(100);
            intake.move(intakeSpeed);
        }

       
}