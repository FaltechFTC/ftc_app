package org.firstinspires.ftc.teamcode;

import com.github.pmtischler.control.Pid;
import com.qualcomm.robotcore.util.RobotLog;

public class OpRotateToHeading extends Operation {
    double targetDegrees, maxPower, targetDegreesAcceptableError;
    Pid pidR;

    public int onTargetLoopCount=0, maxTargetLoopCount=6;

    public static double drivePidKp = 0.47;
    public static double drivePidTi = 0.4;
    public static double drivePidTd = 0.25;
    public static double drivePidIntMax = 4.0;



    public OpRotateToHeading(RoverRobot robot, double targetDegrees, double maxTurnPower, double targetDegreesAcceptableError, long timeoutMS) {
        super(robot);
        this.targetDegrees=targetDegrees;
        this.maxPower=maxTurnPower;
        this.timeoutMS=timeoutMS;
        this.targetDegreesAcceptableError=targetDegreesAcceptableError;

        RobotLog.i("RotateToHeading( targetDegrees=%f, maxPower=%f) ", targetDegrees, maxTurnPower);
        // restart imu angle tracking.
        robot.resetRelativeAngleToZero();

        double drivePidIntMin = -drivePidIntMax;

        pidR = new Pid(drivePidKp, drivePidTi, drivePidTd, drivePidIntMin, drivePidIntMax, -maxTurnPower, maxTurnPower);
        RobotLog.i(pidR.toString());
    }

    public boolean loop() {
        if (!super.loop()) return false;

        double curDegrees = -robot.getRelativeAngle();
        double rotatePower = pidR.update(/*desired*/targetDegrees, /*actual*/curDegrees, deltaTime);


        if (Math.abs(curDegrees - targetDegrees) < targetDegreesAcceptableError)
            onTargetLoopCount++;
        else
            onTargetLoopCount=0;

        if (onTargetLoopCount>=maxTargetLoopCount)
            done();
        else {
            if(onTargetLoopCount == 0 && Math.abs(rotatePower) < .02) {
                if(rotatePower < 0) rotatePower = -.02;
                else rotatePower = .02;

            }
            else if (onTargetLoopCount > 0) rotatePower = 0;
            robot.drive.driveFRS(0.0, rotatePower, 0.0);
        }


        String s= String.format("RotateToHeading curD=%3.1f targetD=%3.1f rPower=%3.2f deltaT=%3.2f totalT=%d onTarget=%d",curDegrees, targetDegrees, rotatePower, deltaTime,(lastTime-startMS), onTargetLoopCount);
        RobotLog.i(s);
        robot.telemetry.addData("Op",s);
        RobotLog.i("loop#"+numLoops+"  pidR = "+pidR.toString());

        return !done;
    }
}
