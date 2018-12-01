package org.firstinspires.ftc.teamcode;

import com.github.pmtischler.control.Pid;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class OpWallride extends Operation {

    double targetDegrees, maxTurnPower, maxDrivePower, targetDistance, targetDistanceToWall;
    double fd=0, sd=0;
    long lastTime = System.currentTimeMillis();
    Pid pidR, pidDrive;

    Boolean onTarget = false;

    public OpWallride(RoverRobot robot, double targetDegrees, double maxDrivePower, double maxTurnPower, long timeoutMS, double targetDistance, double targetDistanceToWall) {
        super(robot);
        this.targetDegrees=targetDegrees;
        this.maxTurnPower=maxTurnPower;
        this.maxDrivePower=maxDrivePower;
        this.timeoutMS=timeoutMS;
        this.targetDistance=targetDistance;
        this.targetDistanceToWall=targetDistanceToWall;

        double drivePidKp = 0.4;     // Tuning variable for PID.
        double drivePidTi = 0.05;   // Eliminate integral error in 1 sec.
        double drivePidTd = 0.2;   // Account for error in 0.1 sec. // Protect against integral windup by limiting integral term.

        double drivePidIntMax = 10;
        double drivePidIntMin = -10;
        pidR = new Pid(drivePidKp, drivePidTi, drivePidTd, drivePidIntMin, drivePidIntMax, -maxTurnPower, maxTurnPower);


        RobotLog.i(pidR.toString());
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
                ", onTarget=" + onTarget +
                '}';
    }

    public void done() {
        super.done();
        robot.drive.stop();
    }

    public boolean loop() {
        if (!super.loop()) return false;

        RobotLog.i("begin loop %d : %s", numLoops, pidR.toString());

        fd= robot.frontDistance.getDistance(DistanceUnit.INCH);
        sd= robot.sideDistance.getDistance(DistanceUnit.INCH);

        double rotatePower = pidR.update(/*desired*/ targetDistanceToWall, /*actual*/ sd, deltaTime);
        robot.drive.driveFRS(maxDrivePower, -rotatePower, 0.0);

        if (fd<= targetDistance)  onTarget = true;

        robot.telemetry.addData("Operation", "WallRide deltaTime="+deltaTime);
        robot.telemetry.addData("front", "target=" + targetDistance+ " actual=" + fd + " power="+maxDrivePower);
        robot.telemetry.addData("side", "target=" + targetDistanceToWall+ " actual=" + sd + " power="+rotatePower);
        robot.telemetry.addData("pidR", pidR.toString());
        robot.telemetry.update();

        if (onTarget) done();
        return !done;
    }
}
