package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class OpWallride extends Operation {


    static Pid pidDrivePrototype = new Pid(.008, 0.0, 0.0, -100.0, 100.0, -.5, .5);
    static Pid pidRotatePrototype = new Pid(.008, 0.00, 0.0, -4, 4, -.1, .1);
    static Pid pidStrafePrototype = new Pid(.01, 0.00, 0.0, -20, 20, -.15, .15);

    Pid pidStrafe, pidDrive, pidRotate;
    double targetDegrees;
    double maxTurnPower, maxDrivePower, maxStrafePower;
    double targetDistance, targetDistanceToWall;
    public double lastError=0.0;

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

        // this forces it to halt or brake
        coastOnStop = false;


        pidDrive = pidDrivePrototype.clone();
        pidDrive.setOutputLimits(-maxDrivePower, maxDrivePower);
        pidRotate = pidRotatePrototype.clone();
        pidRotate.setOutputLimits(-maxTurnPower, maxTurnPower);
        pidStrafe = pidStrafePrototype.clone();
        pidStrafe.setOutputLimits(-maxStrafePower, maxStrafePower);

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

        lastError=fd-targetDistance;
        if (Math.abs(lastError)<2) done();
        return !done;
    }

    public String getResult() {
        double inchesError=robot.drive.convertClicksToInches(lastError);
        return super.getResult()+String.format(" TicksError=%3.1f  Inches=%3.1f", lastError, inchesError);
    }


}
