#pragma once

#include "api.h"
#include "lemlib/api.hpp"
#include "kinematics.hpp"
#include "path.hpp"
#include "trajectoryGenerator.hpp"

inline pros::Task* ramseteTask = nullptr;
inline bool ramseteActive = false;

struct RamseteParamsPublic {
    bool forwards = true;     // true = forward, false = backwards
    double minSpeed = 0;      // minimum motor output
    double maxSpeed = 127;    // maximum motor output
    bool async = false;       // async = return immediately, sync = wait until settled
};

struct RamseteParamsInternal {
    double targetX;
    double targetY;
    bool useFinalHeading;
    double targetThetaDeg;
    bool backwards;
    int timeout;
    double minSpeed;
    double maxSpeed;
};

struct RamseteParams {
    double targetX;
    double targetY;
    bool   useFinalHeading;
    double targetThetaDeg;
    bool   backwards;
    int    timeout;
    double minSpeed;
    double maxSpeed;
};

extern lemlib::Chassis chassis;
extern pros::Controller controller;
extern pros::Motor intake;
extern pros::Motor hood;
extern pros::ADIDigitalOut middleGoal;
extern pros::ADIDigitalOut wing;
extern pros::ADIDigitalOut matchloader;
extern pros::Distance frontSens;
extern pros::Distance rightSens;
extern pros::Distance leftSens;
extern pros::Optical ballSens;

extern void moveDistance(double inches, int timeout, double maxSpeed = 127);
extern void moveDistanceWFrontDist(double inches, int timeout, double maxSpeed = 127);
extern void moveDistanceWFrontDistBack(double inches, int timeout, double maxSpeed = 127);
extern void moveDistanceWFrontDistAngular(double inches, int timeout, double maxSpeed = 127);
extern void _ramseteCore(double targetX, double targetY,
                         bool useFinalHeading, double targetThetaDeg,
                         bool backwards,
                         int timeout, double minSpeed, double maxSpeed);
extern void cancelRamsete();
extern void ramseteTaskFn(void* p);
extern void ramseteToPoint(double targetX, double targetY,
                           int timeout = 2000,
                           RamseteParamsPublic params = {});
extern TrajectoryGenerator generator;
extern lemlib::Pose relocalize(std::string walls, double headingDeg);
extern void maintainHeadingWVoltage(double maxSpeed, double timeout, bool stop = false);
