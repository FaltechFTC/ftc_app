package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.RobotLog;

public class OpRotateToHeading extends Operation {

    public static double minTurnPower =0.019;
    public static Pid pidRotatePrototype = new Pid(0.006, 0.0, 0.0, -5.0, 5.0, -.5, .5);

    public Pid pidR;
    double targetDegrees, maxPower, targetDegreesAcceptableError;
    public int onTargetLoopCount=0, maxTargetLoopCount=7;
    double lastError=0.0;

    public OpRotateToHeading(RoverRobot robot, double targetDegrees, double maxTurnPower, double targetDegreesAcceptableError, long timeoutMS) {
        super(robot);
        this.targetDegrees=targetDegrees;
        this.maxPower=maxTurnPower;
        this.timeoutMS=timeoutMS;
        this.targetDegreesAcceptableError=targetDegreesAcceptableError;

        RobotLog.i("RotateToHeading( targetDegrees=%f, maxPower=%f) ", targetDegrees, maxTurnPower);
        // restart imu angle tracking.
        robot.resetRelativeAngleToZero();

        pidR = pidRotatePrototype.clone();
        pidR.setOutputLimits(-maxTurnPower, maxTurnPower);

        RobotLog.i(pidR.toString());
    }

    public boolean loop() {
        if (!super.loop()) return false;

        double curDegrees = -robot.getRelativeAngle();
        double rotatePower = pidR.update(/*desired*/targetDegrees, /*actual*/curDegrees, deltaTime);

        lastError= curDegrees - targetDegrees;
        if (Math.abs(lastError) < targetDegreesAcceptableError)
            onTargetLoopCount++;
        else
            onTargetLoopCount=0;

        if (onTargetLoopCount>=maxTargetLoopCount)
            done();
        else {

            // if we're not on target, then make sure that power isn't deadzoned to zero, or it won't move
            if(onTargetLoopCount == 0 && Math.abs(rotatePower) < minTurnPower) {
                if(rotatePower < 0) rotatePower = -minTurnPower;
                else rotatePower = minTurnPower;

            }
            else if (onTargetLoopCount > 0) {
                // if we're on target, then stop
                rotatePower = 0;
            }

            robot.drive.driveFRS(0.0, rotatePower, 0.0);
        }


        String s= String.format("RotateToHeading curD=%3.1f targetD=%3.1f rPower=%3.2f deltaT=%3.2f totalT=%3.1f onTarget=%d",curDegrees, targetDegrees, rotatePower, deltaTime, getRuntime(), onTargetLoopCount);
        RobotLog.i(s);
        robot.telemetry.addData("Op",s);
        RobotLog.i("loop#"+numLoops+"  pidR = "+pidR.toString());

        return !done;
    }

    public String getResult() {
        return super.getResult()+String.format(" DegError=%3.2f", lastError);
    }

}
