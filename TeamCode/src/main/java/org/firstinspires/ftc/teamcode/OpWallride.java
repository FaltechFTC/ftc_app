package org.firstinspires.ftc.teamcode;

import com.github.pmtischler.control.Pid;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class OpWallride extends Operation {

    double targetDegrees;
    double maxTurnPower, maxDrivePower, maxStrafePower;
    double targetDistance, targetDistanceToWall;
    Pid pidStrafe, pidDrive, pidRotate;

    double fd=0, sd=0;

    public OpWallride(RoverRobot robot, double targetDegrees, double maxDrivePower, double maxTurnPower, double maxStrafePower, long timeoutMS, double targetDistance, double targetDistanceToWall) {
        super(robot);
        this.targetDegrees=targetDegrees;
        this.maxDrivePower=maxDrivePower;
        this.maxTurnPower=maxTurnPower;
        this.maxStrafePower=maxStrafePower;
        this.timeoutMS=timeoutMS;
        this.targetDistance=targetDistance;
        this.targetDistanceToWall=targetDistanceToWall;
        this.targetDegrees=targetDegrees;


        pidDrive = new Pid(.6, 0.01, 0.2, -100, 100, -maxDrivePower, maxDrivePower);
        pidRotate = new Pid(.5, 0.04, 0.3, -20, 20, -maxTurnPower, maxTurnPower);
        pidStrafe = new Pid(.5, .04, .3, -10, 10,  -maxStrafePower, maxStrafePower);

        robot.resetRelativeAngleToZero();
    }

    @Override
    public String toString() {
        return "OpWallride{" +
                ", maxTurnPower=" + maxTurnPower +
                ", maxDrivePower=" + maxDrivePower +
                ", targetDistance=" + targetDistance +
                ", fd=" + fd +
                ", targetDistanceToWall=" + targetDistanceToWall +
                ", sd=" + sd +
                '}';
    }

    public boolean loop() {
        if (!super.loop()) return false;

        RobotLog.i("begin loop %d : %s", numLoops, pidStrafe.toString());

        fd= robot.frontDistance.getDistance(DistanceUnit.INCH);
        sd= robot.sideDistance.getDistance(DistanceUnit.INCH);
        double relativeAngle = -robot.getRelativeAngle();

        double drivePower = -pidDrive.update(/*desired*/targetDistance, /*actual*/fd, deltaTime);
        double strafePower = pidStrafe.update(/*desired*/ targetDistanceToWall, /*actual*/ sd, deltaTime);
        double rotatePower = pidRotate.update(/*desired*/targetDegrees, /*actual*/relativeAngle, deltaTime);
        robot.drive.driveFRS(drivePower, rotatePower, strafePower);


        robot.telemetry.addData("Operation", "WallRide deltaTime="+deltaTime);
        robot.telemetry.addData("forward", "target=" + targetDistance+ " actual=" + fd + " power="+drivePower);
        robot.telemetry.addData("rotate", "target=" + targetDegrees+ " actual=" + relativeAngle + " power="+rotatePower);
        robot.telemetry.addData("strafe", "target=" + targetDistanceToWall+ " actual=" + sd + " power="+strafePower);
        robot.telemetry.update();

        if (Math.abs(fd-targetDistance)<5) done();
        return !done;
    }
}
