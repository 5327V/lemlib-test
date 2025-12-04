#include "robot.h"
#include "kinematics.hpp"
#include "trajectoryGenerator.hpp"
#include "path.hpp"

#define LATERAL_KD 1.5
#define MOVE_DISTANCE_P 3.8
#define LATERAL_SMALL_ERROR 0.6
#define ANGULAR_SMALL_ERROR 1.0
#define LATERAL_SMALL_ERROR_TIMEOUT 300
#define KV 2.4
#define KMAX 3.5
#define ZETA 1
#define BETA 0.02
#define TRACKWIDTH 13.6
#define FIELD_X_MAX 144.0
#define FIELD_Y_MAX 144.0



// motor groups
pros::MotorGroup leftMotors({-11, -13, -14},
                            pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({18, 19, 20}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

// Inertial Sensor
pros::Imu imu(3);
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              11, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(10, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            35, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            50, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            150, // large error range timeout, in milliseconds
                                            0// maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(2.45, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             15, // derivative gain (kD)
                                             3, // anti windup
                                             0.5, // small error range, in degrees
                                             50, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             150, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

//subystems
pros::Motor intake(10);
pros::Motor hood(-2);

pros::ADIDigitalOut matchloader('C');
pros::ADIDigitalOut middleGoal('B');
pros::ADIDigitalOut wing('A');

pros::Distance frontSens(1);
pros::Distance rightSens(21);
pros::Distance leftSens(7);
pros::Optical ballSens(10);

pros::Controller controller(pros::E_CONTROLLER_MASTER);

// simple straight-line move using odom and a PD controller
void moveDistance(double inches, int timeout, double maxSpeed) {
    const double kP = MOVE_DISTANCE_P;
    const double kD = LATERAL_KD;

    const double tolerance         = LATERAL_SMALL_ERROR;
    const int    smallErrorTimeout = LATERAL_SMALL_ERROR_TIMEOUT;
    const int    loopDelay         = 10;

    lemlib::Pose start = chassis.getPose();
    const double DEG_TO_RAD = M_PI / 180.0;
    double theta0Rad = start.theta * DEG_TO_RAD;


    // LemLib frame: heading 0° = +Y, so forward = (sinθ, cosθ)
    double forwardX = std::sin(theta0Rad);
    double forwardY = std::cos(theta0Rad);

    double error     = inches;
    double prevError = error;
    double output    = 0.0;

    uint32_t startTime             = pros::millis();
    uint32_t withinSmallErrorStart = 0;

    while (pros::millis() - startTime < static_cast<uint32_t>(timeout)) {
        lemlib::Pose cur = chassis.getPose();

        double dx = cur.x - start.x;
        double dy = cur.y - start.y;
        double traveled = dx * forwardX + dy * forwardY;

        error = inches - traveled;

        // small-error timeout
        if (std::fabs(error) < tolerance) {
            if (withinSmallErrorStart == 0) withinSmallErrorStart = pros::millis();
            if (pros::millis() - withinSmallErrorStart >= (uint32_t)smallErrorTimeout) break;
        } else {
            withinSmallErrorStart = 0;
        }

        // PD controller with discrete derivative
        double derivative = 0.0;
        if (std::fabs(error) > tolerance * 2.0) {
            double errorDelta = error - prevError;
            derivative = errorDelta;   // dt baked into kD
        }
        prevError = error;

        output = kP * error + kD * derivative;

        // clamp
        if (output > maxSpeed) output = maxSpeed;
        if (output < -maxSpeed) output = -maxSpeed;

        const double minMove = 5.0;
        if (std::fabs(output) < minMove && std::fabs(error) > tolerance) {
            output = (output >= 0 ? 1 : -1) * minMove;
        }

        chassis.tank((int)output, (int)output, true);
        pros::delay(loopDelay);
    }

    chassis.tank(0, 0, true);
}


void moveDistanceWFrontDist(double inches, int timeout, double maxSpeed) {
    const double kP = MOVE_DISTANCE_P;
    const double kD = LATERAL_KD;

    const double tolerance         = LATERAL_SMALL_ERROR;
    const int    smallErrorTimeout = LATERAL_SMALL_ERROR_TIMEOUT;
    const int    loopDelay         = 10;

    double start = frontSens.get()/25.4;

    double error     = inches;
    double prevError = error;
    double output    = 0.0;
    uint32_t startTime             = pros::millis();
    uint32_t withinSmallErrorStart = 0;
    double derivative = 0.0;
    while (pros::millis() - startTime < static_cast<uint32_t>(timeout)) {
        double cur = frontSens.get()/25.4;
        error = (cur - inches);
        // small-error timeout
        if (std::fabs(error) < tolerance) {
            if (withinSmallErrorStart == 0) withinSmallErrorStart = pros::millis();
            if (pros::millis() - withinSmallErrorStart >= (uint32_t)smallErrorTimeout) break;
        } else {
            withinSmallErrorStart = 0;
        }

        // PD controller with discrete derivative
        double derivative = 0.0;
        if (std::fabs(error) > tolerance * 2.0) {
            double errorDelta = error - prevError;
            derivative = errorDelta;   // dt baked into kD
        }
        prevError = error;

        output = kP * error + kD * derivative;

        // clamp
        if (output > maxSpeed) output = maxSpeed;
        if (output < -maxSpeed) output = -maxSpeed;

        const double minMove = 5.0;
        if (std::fabs(output) < minMove && std::fabs(error) > tolerance) {
            output = (output >= 0 ? 1 : -1) * minMove;
        }

        chassis.tank((int)output, (int)output, true);
        if(error <= 0.2){
            break;
        }
        pros::delay(loopDelay);
    }

    chassis.tank(0, 0, true);
}
void moveDistanceWFrontDistAngular(double inches, int timeout, double maxSpeed) {
    const double kP = MOVE_DISTANCE_P;
    const double kD = LATERAL_KD;

    const double tolerance         = LATERAL_SMALL_ERROR;
    const int    smallErrorTimeout = LATERAL_SMALL_ERROR_TIMEOUT;
    const int    loopDelay         = 10;

    double start = frontSens.get()/25.4;
    double angularStart = imu.get_heading();

    lemlib::PID angularPID = lemlib::PID(0.5, 0, 2);
    double error     = inches;
    double prevError = error;
    double output    = 0.0;
    uint32_t startTime             = pros::millis();
    uint32_t withinSmallErrorStart = 0;
    double derivative = 0.0;
    angularPID.reset();
    while (pros::millis() - startTime < static_cast<uint32_t>(timeout)) {
        double cur = frontSens.get()/25.4;
        double angError = imu.get_heading() - angularStart;
        error = (cur - inches);
        // small-error timeout
        if (std::fabs(error) < tolerance) {
            if (withinSmallErrorStart == 0) withinSmallErrorStart = pros::millis();
            if (pros::millis() - withinSmallErrorStart >= (uint32_t)smallErrorTimeout) break;
        } else {
            withinSmallErrorStart = 0;
        }
        double angOutput = angularPID.update(angError);
        // PD controller with discrete derivative
        double derivative = 0.0;
        if (std::fabs(error) > tolerance * 2.0) {
            double errorDelta = error - prevError;
            derivative = errorDelta;   // dt baked into kD
        }
        prevError = error;

        output = kP * error + kD * derivative;

        // clamp
        if (output > maxSpeed) output = maxSpeed;
        if (output < -maxSpeed) output = -maxSpeed;
        if (angOutput > 90) angOutput = 90;
        if (angOutput < -90) angOutput = -90;

        const double minMove = 5.0;
        if (std::fabs(output) < minMove && std::fabs(error) > tolerance) {
            output = (output >= 0 ? 1 : -1) * minMove;
        }
        
        if(error <= 2){
           angOutput = 0;
        }
        chassis.tank((int)output - angOutput, (int)output + angOutput, true);
        if(error <= 0.2){
            break;
        }
        pros::delay(loopDelay);
    }

    chassis.tank(0, 0, true);
}


// ================== RAMSETE CORE (your existing code) ==================
void _ramseteCore(double targetX, double targetY,
                         bool useFinalHeading, double targetThetaDeg,
                         bool backwards,
                         int timeout, double minSpeed, double maxSpeed) {
    // Ramsete-style params
    const double b    = BETA;
    const double zeta = ZETA;

    // "Virtual" forward velocity gain
    const double kV      = KV;
    const double maxVCmd = maxSpeed * 0.6; // don't try to use full power from v_d
    const double kMax    = KMAX;

    // Exit conditions
    const double posTolInches   = LATERAL_SMALL_ERROR;
    const double thetaTolDeg    = ANGULAR_SMALL_ERROR;
    const int    smallErrorTime = LATERAL_SMALL_ERROR_TIMEOUT;
    const int    loopDelay      = 10;

    const double DEG2RAD = M_PI / 180.0;
    const double RAD2DEG = 180.0 / M_PI;

    uint32_t startTime             = pros::millis();
    uint32_t withinSmallErrorStart = 0;

    while (pros::millis() - startTime < static_cast<uint32_t>(timeout)) {
        // ---- Current pose ----
        lemlib::Pose cur = chassis.getPose();
        double x     = cur.x;
        double y     = cur.y;
        double theta = cur.theta * DEG2RAD; // radians

        // ---- Global error ----
        double dx = targetX - x;
        double dy = targetY - y;

        // Pick desired heading
        double targetThetaRad;
        if (useFinalHeading) {
            targetThetaRad = targetThetaDeg * DEG2RAD;
        } else {
            // 0° = +Y ⇒ bearing = atan2(dx, dy)
            if (!backwards) {
                targetThetaRad = std::atan2(dx, dy);     // face toward point
            } else {
                targetThetaRad = std::atan2(-dx, -dy);   // face away, drive backward
            }
        }

        // Angle error wrapped to [-pi, pi]
        double dtheta = targetThetaRad - theta;
        while (dtheta >  M_PI) dtheta -= 2.0 * M_PI;
        while (dtheta < -M_PI) dtheta += 2.0 * M_PI;

        // ---- Transform error into robot frame (LemLib frame) ----
        // x = left/right (right +), y = forward/back (forward +), 0° = +Y
        // forward unit = (sinθ, cosθ)
        // left    unit = (-cosθ, sinθ)
        double cosT = std::cos(theta);
        double sinT = std::sin(theta);

        double x_e = dx * sinT + dy * cosT;      // forward error
        double y_e = -dx * cosT + dy * sinT;     // left error
        double theta_e = dtheta;

        // ---- Tolerances ----
        double posErr      = std::sqrt(x_e * x_e + y_e * y_e);
        double thetaErrDeg = std::fabs(theta_e * RAD2DEG);

        bool posGood   = posErr < posTolInches;
        bool thetaGood = (!useFinalHeading) || (thetaErrDeg < thetaTolDeg);

        if (posGood && thetaGood) {
            if (withinSmallErrorStart == 0)
                withinSmallErrorStart = pros::millis();
            if (pros::millis() - withinSmallErrorStart >= (uint32_t)smallErrorTime)
                break; // done
        } else {
            withinSmallErrorStart = 0;
        }

        // ---- "Virtual" desired velocities ----
        // Forward speed magnitude grows with distance
        double v_d = kV * posErr;
        if (!backwards) {
            if (v_d > maxVCmd) v_d = maxVCmd;
        } else {
            v_d = -v_d;
            if (v_d < -maxVCmd) v_d = -maxVCmd;
        }

        double w_d = 0.0;

        // ---- Ramsete gain (capped) ----
        double k_raw = 2.0 * zeta * std::sqrt(b * v_d * v_d);
        double k = std::min(k_raw, kMax);

        double sinThetaE  = std::sin(theta_e);
        double sincThetaE = (std::fabs(theta_e) < 1e-6) ? 1.0 : (sinThetaE / theta_e);

        // ---- Ramsete control law ----
        double v = v_d * std::cos(theta_e) + k * x_e;
        double w = w_d + k * theta_e + b * v_d * sincThetaE * y_e;

        // Forward-only mode: don't let v go negative
        if (!backwards && v < 0) {
            v = 0;
        }

        // ---- Convert to left/right commands ----
        double leftCmd  = v - w * (TRACKWIDTH / 2.0);
        double rightCmd = v + w * (TRACKWIDTH / 2.0);

        // Normalize to ±maxSpeed
        double maxMag = std::max(std::fabs(leftCmd), std::fabs(rightCmd));
        if (maxMag > maxSpeed && maxMag > 1e-6) {
            double scale = maxSpeed / maxMag;
            leftCmd  *= scale;
            rightCmd *= scale;
        }

        // Small deadband so it doesn't stall just outside tolerance
        if (posErr > posTolInches && minSpeed > 0) {
            if (std::fabs(leftCmd) < minSpeed) {
                leftCmd = (leftCmd >= 0 ? 1 : -1) * minSpeed;
            }
            if (std::fabs(rightCmd) < minSpeed) {
                rightCmd = (rightCmd >= 0 ? 1 : -1) * minSpeed;
            }
        }

        chassis.tank(static_cast<int>(leftCmd), static_cast<int>(rightCmd), true);
        pros::delay(loopDelay);
    }

    chassis.tank(0, 0, true);
}


// ================== ASYNC WRAPPER + WAIT LOGIC ==================


void cancelRamsete() {
    if (ramseteTask != nullptr) {
        ramseteTask->remove();
        delete ramseteTask;
        ramseteTask = nullptr;
    }
    ramseteActive = false;
}


void ramseteTaskFn(void* p) {
    RamseteParamsInternal params = *static_cast<RamseteParamsInternal*>(p);
    delete static_cast<RamseteParamsInternal*>(p);

    ramseteActive = true;

    _ramseteCore(
        params.targetX,
        params.targetY,
        params.useFinalHeading,
        params.targetThetaDeg,
        params.backwards,
        params.timeout,
        params.minSpeed,
        params.maxSpeed
    );

    ramseteActive = false;
}



// ================== PUBLIC API ==================

// go to a point (no final heading)
// waitUntilSettled = false → async (like LemLib moveToPoint)
// waitUntilSettled = true  → blocking until done
void ramseteToPoint(double targetX, double targetY,
                           int timeout,
                           RamseteParamsPublic params) {
    cancelRamsete();

    // Convert user params → internal params
    RamseteParamsInternal* p = new RamseteParamsInternal{
        targetX,
        targetY,
        false,                 // useFinalHeading
        0.0,
        !params.forwards,      // backwards if forwards=false
        timeout,
        params.minSpeed,
        params.maxSpeed
    };

    // Start async task
    ramseteTask = new pros::Task(ramseteTaskFn, p, "RamsetePoint");

    if (!params.async) {
        while (ramseteActive) pros::delay(5);
    }
}


// go to a pose (point + heading)
void ramseteToPose(double targetX, double targetY, double targetThetaDeg,
                          int timeout = 2000,
                          RamseteParamsPublic params = {}) {
    cancelRamsete();

    RamseteParamsInternal* p = new RamseteParamsInternal{
        targetX,
        targetY,
        true,                  // useFinalHeading
        targetThetaDeg,
        !params.forwards,
        timeout,
        params.minSpeed,
        params.maxSpeed
    };

    ramseteTask = new pros::Task(ramseteTaskFn, p, "RamsetePose");

    if (!params.async) {
        while (ramseteActive) pros::delay(5);
    }
}

TrajectoryGenerator generator(new DifferentialKinematics(12, 75, 75, 0.4), 0.01);


double degToRad(double d) { return d * M_PI / 180.0; }

// φ offsets for each sensor relative to robot forward
// Change these if your sensors face different directions
double PHI_FRONT = 0.0;
double PHI_RIGHT = -90.0;
double PHI_LEFT  = 90.0;

double readSensor(pros::Distance &s) {
    return s.get() / 25.4; // mm → inches
}

double getWallCoordinate(char wall) {
    switch (wall) {
        case 'N': return 72; // North wall is +Y max
        case 'S': return -72;         // South wall is Y = -72
        case 'E': return 72; // East wall is +X max
        case 'W': return -72;         // West wall is X = 0
    }
    return 0.0;
}

double computeProjection(double d, double thetaDeg, double phiDeg, bool projectX) {
    double the = degToRad(thetaDeg + phiDeg);

    if (projectX)
        return d * cos(the);
    else
        return d * sin(the);
}

lemlib::Pose relocalize(std::string walls, double headingDeg) {
    lemlib::Pose p = {NAN, NAN, NAN};

    bool useNorth = walls.find('N') != std::string::npos;
    bool useSouth = walls.find('S') != std::string::npos;
    bool useEast  = walls.find('E') != std::string::npos;
    bool useWest  = walls.find('W') != std::string::npos;

    double dFront = readSensor(frontSens);
    double dRight = readSensor(rightSens);
    double dLeft  = readSensor(leftSens);

    // ---------- Y COORDINATE (North/South wall) ----------
    if (useNorth || useSouth) {
        char wall = useNorth ? 'N' : 'S';
        double Ywall = getWallCoordinate(wall);

        // Determine which sensor points toward the wall
        // North wall = +Y direction
        // South wall = -Y direction
        double bestDist = INFINITY;
        double sensorDist, phi;

        if (useNorth) {
            // sensor direction . global +Y
            sensorDist = dFront; phi = PHI_FRONT;
            bestDist = sensorDist * sin(degToRad(headingDeg + phi));

            if (bestDist < 0) bestDist = INFINITY; // Wrong direction
            // Check left
            double projL = dLeft * sin(degToRad(headingDeg + PHI_LEFT));
            if (projL > 0 && projL < bestDist) {sensorDist = dLeft; phi = PHI_LEFT; bestDist = projL;}
            // Check right
            double projR = dRight * sin(degToRad(headingDeg + PHI_RIGHT));
            if (projR > 0 && projR < bestDist) {sensorDist = dRight; phi = PHI_RIGHT; bestDist = projR;}
        }
        else { // South (negative Y)
            sensorDist = dFront; phi = PHI_FRONT;
            bestDist = -sensorDist * sin(degToRad(headingDeg + phi));

            if (bestDist < 0) bestDist = INFINITY;

            double projL = -dLeft * sin(degToRad(headingDeg + PHI_LEFT));
            if (projL > 0 && projL < bestDist) { sensorDist = dLeft; phi = PHI_LEFT; bestDist = projL; }

            double projR = -dRight * sin(degToRad(headingDeg + PHI_RIGHT));
            if (projR > 0 && projR < bestDist) { sensorDist = dRight; phi = PHI_RIGHT; bestDist = projR; }
        }

        double projY = computeProjection(sensorDist, headingDeg, phi, false);
        if (useSouth) projY = -projY;

        p.y = Ywall - projY;
    }


    // ---------- X COORDINATE (East/West wall) ----------
    if (useEast || useWest) {
        char wall = useEast ? 'E' : 'W';
        double Xwall = getWallCoordinate(wall);

        double bestDist = INFINITY;
        double sensorDist, phi;

        if (useEast) { // +X
            sensorDist = dRight; phi = PHI_RIGHT; bestDist = sensorDist * cos(degToRad(headingDeg + phi));

            double projF = dFront * cos(degToRad(headingDeg + PHI_FRONT));
            if (projF > 0 && projF < bestDist) { sensorDist = dFront; phi = PHI_FRONT; bestDist = projF; }

            double projL = dLeft * cos(degToRad(headingDeg + PHI_LEFT));
            if (projL > 0 && projL < bestDist) { sensorDist = dLeft; phi = PHI_LEFT; bestDist = projL; }
        }
        else { // West, -X
            sensorDist = dLeft; phi = PHI_LEFT; bestDist = -sensorDist * cos(degToRad(headingDeg + phi));

            double projF = -dFront * cos(degToRad(headingDeg + PHI_FRONT));
            if (projF > 0 && projF < bestDist) { sensorDist = dFront; phi = PHI_FRONT; bestDist = projF; }

            double projR = -dRight * cos(degToRad(headingDeg + PHI_RIGHT));
            if (projR > 0 && projR < bestDist) { sensorDist = dRight; phi = PHI_RIGHT; bestDist = projR; }
        }

        double projX = computeProjection(sensorDist, headingDeg, phi, true);
        if (useWest) projX = -projX;

        p.x = Xwall - projX;
    }


    return p;
}

void maintainHeadingWVoltage(double maxSpeed, double timeout, bool stop){
    lemlib::PID angularPID      = lemlib::PID(0.5, 0, 5);
    const int    loopDelay      = 10;

    const double DEG2RAD        = M_PI / 180.0;
    const double RAD2DEG        = 180.0 / M_PI;

    uint32_t startTime             = pros::millis();

    double currAngle               = imu.get_heading();
    
    angularPID.reset();

    while((pros::millis() - startTime)>timeout){
        
        //get curr angle
        double angle = imu.get_heading();

        //calculate error
        double error = angle - currAngle;

        //update PID
        double output = angularPID.update(error);

        //set speed
        chassis.tank(maxSpeed - output, maxSpeed + output);

        pros::delay(loopDelay);
    }

    if(stop){
        chassis.tank(0, 0);
    }
}
/*impl
CubicBezier *testPath;
  testPath = new CubicBezier({12, 36}, {12, 60}, {36, 36}, {36, 60});

  Path *multiPath =
      new MultiPath({testPath, new CubicBezier({-36, -60}, {-36, -84},
                                               {-60, -60}, {-60, -84})});

  TrajectoryGenerator generator(new DifferentialKinematics(12, 75, 75, 0.4), 0.01);

  generator.generateTrajectory(multiPath);

  std::vector<Pose> trajectory = generator.getTrajectory();

  std::cout << std::endl;
*/