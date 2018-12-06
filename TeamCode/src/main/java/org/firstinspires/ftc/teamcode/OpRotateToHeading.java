package org.firstinspires.ftc.teamcode;

import com.github.pmtischler.control.Pid;
import com.qualcomm.robotcore.util.RobotLog;

public class OpRotateToHeading extends Operation {
    double targetDegrees, maxPower, targetDegreesAcceptableError;
    Pid pidR;

    Boolean onTarget = false;
    Boolean halt = false;

    public OpRotateToHeading(RoverRobot robot, double targetDegrees, double maxTurnPower, double targetDegreesAcceptableError, long timeoutMS) {
        super(robot);
        this.targetDegrees=targetDegrees;
        this.maxPower=maxTurnPower;
        this.timeoutMS=timeoutMS;
        this.targetDegreesAcceptableError=targetDegreesAcceptableError;

        RobotLog.i("RotateToHeading( targetDegrees=%f, maxPower=%f) ", targetDegrees, maxTurnPower);
        // restart imu angle tracking.
        robot.resetRelativeAngleToZero();

        double drivePidKp = 0.8;     // Tuning variable for PID.
        double drivePidTi = 0.1;   // Eliminate integral error in 1 sec.
        double drivePidTd = 0.4;   // Account for error in 0.1 sec. // Protect against integral windup by limiting integral term.

        double drivePidIntMax = 180.0;
        double drivePidIntMin = -drivePidIntMax;

        pidR = new Pid(drivePidKp, drivePidTi, drivePidTd, drivePidIntMin, drivePidIntMax, -maxTurnPower, maxTurnPower);
        RobotLog.i(pidR.toString());
    }

    public boolean loop() {
        if (!super.loop()) return false;

        RobotLog.i("begin loop %d : %s", numLoops, pidR.toString());

        double relativeAngle = -robot.getRelativeAngle();
        RobotLog.i("update(%f, %f, %f)", targetDegrees, relativeAngle, deltaTime);

        double rotatePower = pidR.update(/*desired*/targetDegrees, /*actual*/relativeAngle, deltaTime);
        RobotLog.i("update= drivePower=%f  %s", rotatePower, pidR.toString());

        robot.drive.driveFRS(0.0, rotatePower, 0.0);

        if (Math.abs(relativeAngle - targetDegrees) < targetDegreesAcceptableError)
            onTarget = true;

        robot.telemetry.addData("angles", "target=" + targetDegrees + " relative=" + relativeAngle);
        robot.telemetry.addData("power", rotatePower);
        robot.telemetry.addData("time", "Delta=" + deltaTime + " total=" + (lastTime - startMS));
        robot.telemetry.addData("pidR", pidR.toString());
        robot.telemetry.addData("status", onTarget ? "On TARGET" : (halt ? "HALT" : "running"));
        RobotLog.i("onTarget=%s Halt=%s", onTarget.toString(), halt.toString());
        robot.telemetry.update();

        if (onTarget) done();
        return !done;
    }
}
