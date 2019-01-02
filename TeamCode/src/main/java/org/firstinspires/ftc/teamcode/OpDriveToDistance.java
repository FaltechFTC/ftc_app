package org.firstinspires.ftc.teamcode;

import com.github.pmtischler.control.Pid;
import com.qualcomm.robotcore.util.RobotLog;

public class OpDriveToDistance extends Operation {


    double maxDrivePower, maxRotatePower;
    double targetDistance;
    double targetTolerance;
    double startingEncoder, targetEncoder;

    Pid pidDrive, pidRotate;

    double curEncoder=0;
    public int onTargetLoopCount=0, maxTargetLoopCount=3;


    public OpDriveToDistance(RoverRobot robot, double maxDrivePower, long timeoutMS, double targetDistance, double targetTolerance) {
        this(robot,maxDrivePower,.06,timeoutMS,targetDistance,targetTolerance);
    }

    public OpDriveToDistance(RoverRobot robot, double maxDrivePower, double maxRotatePower, long timeoutMS, double targetDistance, double targetTolerance) {
        super(robot);
        this.maxDrivePower=maxDrivePower;
        this.maxRotatePower=maxRotatePower;
        this.timeoutMS=timeoutMS;
        this.targetDistance=targetDistance;

        // this forces it to halt or brake
        coastOnStop = false;

        pidDrive = new Pid(.6, 0.01, 0.2, -100, 100, -maxDrivePower, maxDrivePower);
        pidRotate = new Pid(.5, 0.04, 0.3, -20, 20, -maxRotatePower, maxRotatePower);

        startingEncoder= robot.drive.getEncoderClicksFront();
        targetEncoder=startingEncoder+robot.drive.convertInchesToClicks(targetDistance);
        this.targetTolerance=Math.abs(robot.drive.convertInchesToClicks(targetTolerance));

        robot.resetRelativeAngleToZero();
    }

    @Override
    public String toString() {
        return "OpWallride{" +
                ", maxDrivePower=" + maxDrivePower +
                ", targetDistance=" + targetDistance +
                ", targetToleranceClicks=" + targetTolerance +
                ", curEncoder=" + curEncoder +
                '}';
    }

    public boolean loop() {
        if (!super.loop()) return false;

        curEncoder= robot.drive.getEncoderClicksFront();
        robot.telemetry.addData("encoders", "start=%f  target=%f  current=%f", startingEncoder, targetEncoder, curEncoder);
        RobotLog.i("DriveToDistance: start=%f  target=%f  current=%f", startingEncoder, targetEncoder, curEncoder);

        if (Math.abs(targetEncoder - curEncoder)<= targetTolerance) {
            onTargetLoopCount++;
        } else {
            onTargetLoopCount = 0;
        }

        if (onTargetLoopCount>=maxTargetLoopCount) {
            done();
        } else {
            double relativeAngle = -robot.getRelativeAngle();
            double rotatePower = pidRotate.update(/*desired*/0, /*actual*/relativeAngle, deltaTime);
            double drivePower = pidDrive.update(/*desired*/targetEncoder, /*actual*/curEncoder, deltaTime);
            robot.drive.driveFRS(drivePower, rotatePower, 0);
            String s=String.format("forward=%3.2f  rotate=%3.2f",drivePower, rotatePower);
            robot.telemetry.addData("DriveToHeading Powers", s);
            RobotLog.i("DriveToHeading Powers "+s);

        }
        return !done;
    }
}
