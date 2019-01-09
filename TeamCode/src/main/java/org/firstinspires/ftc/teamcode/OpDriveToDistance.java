package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class OpDriveToDistance extends Operation {

    static Pid pidDrivePrototype = new Pid(.02, 0.0, 0.0, -100.0, 100.0, -.5, .5);
    static Pid pidRotatePrototype = new Pid(.008, 0.00, 0.0, -4, 4, -.5, .5);
    static Pid pidStrafePrototype = new Pid(.01, 0.00, 0.0, -20, 20, 0, 0);

    Pid pidDrive, pidRotate, pidStrafe;

    double maxDrivePower, maxRotatePower, maxStrafePower;
    double targetDistance, targetDistanceToWall;
    double targetTolerance;
    double startingEncoder, targetEncoder;

    double currentEncoder=0;
    public int onTargetLoopCount=0, maxTargetLoopCount=3;
    public double targetHeading=0.0;
    public double lastError=0.0;


    public OpDriveToDistance(RoverRobot robot, double maxDrivePower, long timeoutMS, double targetDistance, double targetTolerance) {
        this(robot,maxDrivePower,.06,timeoutMS,targetDistance,targetTolerance);
    }

    public OpDriveToDistance(RoverRobot robot, double maxDrivePower, double maxRotatePower, long timeoutMS, double targetDistance, double targetTolerance) {
        super(robot);
        this.maxDrivePower=maxDrivePower;
        this.maxRotatePower=maxRotatePower;
        this.timeoutMS=timeoutMS;
        this.targetDistance=targetDistance;

        this.maxStrafePower=0.0;
        this.targetDistanceToWall=0.0;  // NA

        // this forces it to halt or brake
        coastOnStop = false;

        pidDrive=pidDrivePrototype.clone();
        pidDrive.setOutputLimits(-maxDrivePower,maxDrivePower);
        pidRotate=pidRotatePrototype.clone();
        pidRotate.setOutputLimits(-maxRotatePower,maxRotatePower);
        pidStrafe=pidStrafePrototype.clone();

        startingEncoder= robot.drive.getEncoderClicksFront();
        targetEncoder=startingEncoder+robot.drive.convertInchesToClicks(targetDistance);
        this.targetTolerance=Math.abs(robot.drive.convertInchesToClicks(targetTolerance));

        robot.resetRelativeAngleToZero();
    }

    public void setWallride(double maxStrafePower, double targetWallDistance) {
        this.maxStrafePower=0.0;
        this.targetDistanceToWall=0.0;  // NA
        pidStrafe.setOutputLimits(-maxStrafePower, maxStrafePower);
    }

    @Override
    public String toString() {
        return "OpWallride{" +
                ", maxDrivePower=" + maxDrivePower +
                ", targetDistance=" + targetDistance +
                ", targetToleranceClicks=" + targetTolerance +
                ", curEncoder=" + currentEncoder +
                '}';
    }

    public boolean loop() {
        if (!super.loop()) return false;

        currentEncoder= robot.drive.getEncoderClicksFront();
        robot.telemetry.addData("encoders", "start=%f  target=%f  current=%f", startingEncoder, targetEncoder, currentEncoder);
        RobotLog.i("DriveToDistance: start=%f  target=%f  current=%f", startingEncoder, targetEncoder, currentEncoder);

        lastError=targetEncoder - currentEncoder;
        if (Math.abs(lastError)<= targetTolerance) {
            onTargetLoopCount++;
        } else {
            onTargetLoopCount = 0;
        }

        if (onTargetLoopCount>=maxTargetLoopCount) {
            done();
        } else {
            double drivePower = pidDrive.update(targetEncoder, currentEncoder, deltaTime);

            double currentHeading = -robot.getRelativeAngle();
            double rotatePower = pidRotate.update(targetHeading, currentHeading, deltaTime);

            double currentDistanceToWall= robot.sideDistance.getDistance(DistanceUnit.INCH);
            double strafePower = pidStrafe.update(targetDistanceToWall, currentDistanceToWall, deltaTime);

            robot.drive.driveFRS(drivePower, rotatePower, strafePower);
            String s=String.format("Power: F=%3.2f  R=%3.2f  Heading: tgt=%4.1f cur=%4.1f",drivePower, rotatePower, targetHeading, currentHeading);
            robot.telemetry.addData("DriveToHeading", s);
            RobotLog.i("DriveToHeading "+s);

        }
        return !done;
    }

    public String getResult() {
        double inchesError=robot.drive.convertClicksToInches(lastError);
        return super.getResult()+String.format(" TicksError=%3.1f  Inches=%3.1f", lastError, inchesError);
    }

}
