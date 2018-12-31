package org.firstinspires.ftc.teamcode;

import com.github.pmtischler.control.Pid;
import com.qualcomm.robotcore.util.RobotLog;

public class OpDriveToDistance extends Operation {


    double maxDrivePower;
    double targetDistance;
    double targetTolerance;
    double startingEncoder, targetEncoder;

    Pid pidDrive;

    double curEncoder=0;

    public OpDriveToDistance(RoverRobot robot, double maxDrivePower, long timeoutMS, double targetDistance, double targetTolerance) {
        super(robot);
        this.maxDrivePower=maxDrivePower;
        this.timeoutMS=timeoutMS;
        this.targetDistance=targetDistance;

        // this forces it to halt or brake
        coastOnStop = false;

        pidDrive = new Pid(.6, 0.01, 0.2, -100, 100, -maxDrivePower, maxDrivePower);

        startingEncoder= robot.drive.getEncoderClicksFront();
        targetEncoder=startingEncoder+robot.drive.convertInchesToClicks(targetDistance);
        this.targetTolerance=Math.abs(robot.drive.convertInchesToClicks(targetTolerance));
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
            done();
        } else {
            double drivePower = pidDrive.update(/*desired*/targetEncoder, /*actual*/curEncoder, deltaTime);
            robot.drive.driveFRS(drivePower, 0, 0);
            robot.telemetry.addData("forwardPower", drivePower);
        }
        return !done;
    }
}
