package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.RobotLog;

public class OpDriveToHeading extends Operation {
    double targetDegrees, maxTurnPower, maxDrivePower, targetDegreesAcceptableError, targetDistance;
    public static Pid pidRotatePrototype = new Pid(0.006, 0.0, 0.0, -5.0, 5.0, -.5, .5);
    public Pid pidR;

    double startingEncoder=0.0;
    double targetEncoder;

    Boolean onTarget = false;
    Boolean halt = false;

    public OpDriveToHeading(RoverRobot robot, double targetDegrees, double maxDrivePower, double maxTurnPower, double targetDegreesAcceptableError, long timeoutMS, double targetDistance) {
       super(robot);
        this.targetDegrees=targetDegrees;
        this.maxTurnPower=maxTurnPower;
        this.maxDrivePower=maxDrivePower;
        this.timeoutMS=timeoutMS;
        this.targetDistance=targetDistance;
        this.targetDegreesAcceptableError=targetDegreesAcceptableError;

        // todo; add logging for distance
        RobotLog.i("DriveToHeading( targetDegrees=%f, maxDrivePower=%f, maxTurnPower=%f, targetDistance=%f) ", targetDegrees, maxDrivePower, maxTurnPower, targetDistance);
        // restart imu angle tracking.
        robot.resetRelativeAngleToZero();

        if (targetDistance!=0.0) {
          //  drive.resetEncoders();
            startingEncoder= robot.drive.getEncoderClicksAbs();
            targetEncoder=startingEncoder+ robot.drive.convertInchesToClicks(targetDistance);
            RobotLog.i("DriveToHeading( startingEncoder=%f  targetEncoder=%f) ", startingEncoder, targetEncoder);
        }

        pidR = pidRotatePrototype.clone();
        pidR.setOutputLimits(-maxTurnPower, maxTurnPower);
        RobotLog.i(pidR.toString());
    }

    public boolean loop() {
        if (!super.loop()) return false;

        RobotLog.i("begin loop %d : %s", numLoops, pidR.toString());

        double relativeAngle = -robot.getRelativeAngle();
        RobotLog.i("update(%f, %f, %f)", targetDegrees, relativeAngle, deltaTime);

        double rotatePower = pidR.update(/*desired*/targetDegrees, /*actual*/relativeAngle, deltaTime);
        RobotLog.i("update= drivePower=%f  %s", rotatePower, pidR.toString());

        robot.drive.driveFRS(maxDrivePower, rotatePower, 0.0);

        double curEncoder= robot.drive.getEncoderClicksAbs();

        RobotLog.i("DriveToHeading( startingEncoder=%f  targetEncoder=%f  currentEncoder=%f", startingEncoder, targetEncoder, curEncoder);

        // problem with this is that it needs to use a pid to scale back power to hit this distance accurately.
        if (targetDistance!=0.0 && robot.isDistanceWithinRange(curEncoder,targetEncoder)) {
            onTarget = true;
            RobotLog.i("ON TARGET!");
        } else {
            RobotLog.i("NOT ON TARGET!");
        }

        robot.telemetry.addData("angles", "target=" + targetDegrees + " relative=" + relativeAngle);
        robot.telemetry.addData("targetD", "trgtD=" + targetDistance+ " trgtEnc=" + targetEncoder+ " curEnc="+curEncoder);
        robot.telemetry.addData("power", maxDrivePower);
        robot.telemetry.addData("time", "Delta=" + deltaTime + " total=" + (lastTime - startMS));
        robot.telemetry.addData("pidR", pidR.toString());
        robot.telemetry.addData("status", onTarget ? "On TARGET" : (halt ? "HALT" : "running"));
        RobotLog.i("onTarget=%s Halt=%s", onTarget.toString(), halt.toString());
        robot.telemetry.update();

        if (onTarget) done();
        return !done;
    }


}
