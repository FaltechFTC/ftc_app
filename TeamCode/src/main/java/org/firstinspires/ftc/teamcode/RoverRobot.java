package org.firstinspires.ftc.teamcode;

import com.github.pmtischler.control.Pid;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class RoverRobot {

    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();
    public Telemetry telemetry = null;

    IDrive drive = null;
    RoverCollector roverCollector = new RoverCollector();
    RoverLift roverLift = new RoverLift();

    BNO055IMU imu = null;
    Orientation lastAngles = new Orientation();
    public final AxesOrder axesOrder = AxesOrder.ZYX;
    double globalAngle;
    public Servo teamMarker = null;
    boolean isAutonomous=false;
    Rev2mDistanceSensor frontDistance;
    Rev2mDistanceSensor sideDistance;

    //Constructor//
    public RoverRobot(IDrive drive) {
        this.drive = drive;
    }

    public void init(HardwareMap hwMap, Telemetry telemetry, boolean isAutonomous) {
        // Save reference to Hardware map
        this.hwMap = hwMap;
        this.telemetry = telemetry;
        this.isAutonomous=isAutonomous;

        teamMarker = hwMap.get(Servo.class, "teamMarker");

        frontDistance = (Rev2mDistanceSensor) hwMap.get(DistanceSensor.class, "frontDistance");
        sideDistance = (Rev2mDistanceSensor) hwMap.get(DistanceSensor.class, "sideDistance");

        drive.init(hwMap, telemetry);
        drive.setRunModeEncoder(isAutonomous);
        roverCollector.init(hwMap, telemetry,isAutonomous);
        roverLift.init(hwMap, telemetry, isAutonomous);
        teamMarker.setPosition(0);
        initIMU();
    }

    private void initIMU() {
        RobotLog.i("initIMU() start");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);


        // make sure the imu gyro is calibrated before continuing.
        waitForGyroCalibration();
        resetRelativeAngleToZero();

        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.addData("angles", "relative=" + getRelativeAngle() + " absolute" + getCurrentAbsoluteAngle());
        telemetry.update();

        resetRelativeAngleToZero();

    }

    public void stop() {
        drive.stop();
    }

    // probably don't need these...
    public void forwardTeleOp(double power) {
        drive.driveFRS(power, 0.0, 0.0);
    }

    public void strafeTeleOp(double power) {
        drive.driveFRS(0.0, 0.0, power);
    }

    public void turnTeleOp(double power) {
        drive.driveFRS(0.0, power, 0.0);
    }

    protected void waitForGyroCalibration() {
        RobotLog.i("Calibrating...");

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();
        while (!imu.isGyroCalibrated()) {
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                // eat the exception
            }
        }
        telemetry.addData("Mode", "Calibrated");
        telemetry.update();
        RobotLog.i("DoneCalibrating...");

    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    protected void resetRelativeAngleToZero() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, axesOrder, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     *
     * @return Angle in degrees. + = left, - = right from zero point.
     */
    protected double getRelativeAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, axesOrder, AngleUnit.DEGREES);
        double deltaAngle = mapDegreesTo180(angles.firstAngle - lastAngles.firstAngle);
        return deltaAngle;
    }

    /* should allow to get a angle relative to the reset that goes past 180 and beyond.*/
    private double getAngleViaIncrementals() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = mapDegreesTo180(angles.firstAngle - lastAngles.firstAngle);
        globalAngle += deltaAngle;
        lastAngles = angles;
        return globalAngle;
    }

    protected double mapDegreesTo180(double d) {
        while (true) {
            if (d < -180) d += 360;
            else if (d > 180) d -= 360;
            else return d;
        }
    }

    protected double getCurrentAbsoluteAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, axesOrder, AngleUnit.DEGREES);
        return angles.firstAngle;
    }


    public class Operation {
        public long timeoutMS=9999;
        public long startMS=System.currentTimeMillis();
        public long curMS=0;
        public boolean done=false;
        public boolean timeout=false;
        public long numLoops=0;
        public Operation() {
        }

        public boolean run() {
            return run(10);
        }
        public boolean run(long timeBetweenLoopsMS) {
            while(loop()) {
                sleep(timeBetweenLoopsMS);
            }
            return !timeout; // true if we ran to completion, else false means we timed out
        }

        public boolean loop() {
            if (done) return false;
            numLoops++;
            curMS=System.currentTimeMillis();
            if (curMS-startMS > timeoutMS) {timeout=true; done(); return false;}
            return true;
            // keep looping ...
        }

        public void done() {
            done=true;
        }
    }


    public class RotateToHeading extends Operation {
        double targetDegrees, maxPower, targetDegreesAcceptableError;
        long lastTime = System.currentTimeMillis();
        Pid pidR;

        Boolean onTarget = false;
        Boolean halt = false;
        double deltaTime = .001;

        public RotateToHeading (double targetDegrees, double maxTurnPower, double targetDegreesAcceptableError, long timeoutMS) {
            this.targetDegrees=targetDegrees;
            this.maxPower=maxTurnPower;
            this.timeoutMS=timeoutMS;
            this.targetDegreesAcceptableError=targetDegreesAcceptableError;

            RobotLog.i("RotateToHeading( targetDegrees=%f, maxPower=%f) ", targetDegrees, maxTurnPower);
            // restart imu angle tracking.
            resetRelativeAngleToZero();

            double drivePidKp = 1.0;     // Tuning variable for PID.
            double drivePidTi = 0.5;   // Eliminate integral error in 1 sec.
            double drivePidTd = 0.4;   // Account for error in 0.1 sec. // Protect against integral windup by limiting integral term.

            double drivePidIntMax = 180.0;
            double drivePidIntMin = -drivePidIntMax;

            pidR = new Pid(drivePidKp, drivePidTi, drivePidTd, drivePidIntMin, drivePidIntMax, -maxTurnPower, maxTurnPower);
            RobotLog.i(pidR.toString());
        }

        public void done() {
            super.done();
            drive.stop();
            RobotLog.i("rotate2() done");
        }

        public boolean loop() {
            if (!super.loop()) return false;

            RobotLog.i("begin loop %d : %s", numLoops, pidR.toString());

            double relativeAngle = -getRelativeAngle();
            double deltaTime = ((double) (curMS - lastTime)) / 1000.0;

            RobotLog.i("update(%f, %f, %f)", targetDegrees, relativeAngle, deltaTime);

            double rotatePower = pidR.update(/*desired*/targetDegrees, /*actual*/relativeAngle, deltaTime);
            RobotLog.i("update= drivePower=%f  %s", rotatePower, pidR.toString());

            drive.driveFRS(0.0, rotatePower, 0.0);

            lastTime = curMS;

            if (Math.abs(relativeAngle - targetDegrees) < targetDegreesAcceptableError)
                onTarget = true;

            telemetry.addData("angles", "target=" + targetDegrees + " relative=" + relativeAngle);
            telemetry.addData("power", rotatePower);
            telemetry.addData("time", "Delta=" + deltaTime + " total=" + (lastTime - startMS));
            telemetry.addData("pidR", pidR.toString());
            telemetry.addData("status", onTarget ? "On TARGET" : (halt ? "HALT" : "running"));
            RobotLog.i("onTarget=%s Halt=%s", onTarget.toString(), halt.toString());
            telemetry.update();

            if (onTarget) done();
            return !done;
        }
    }

    public boolean isDistanceWithinRange(double startingEncoder, double targetEncoder){
        if (Math.abs(startingEncoder - targetEncoder)<= drive.convertInchesToClicks(1)) return true;
        return false;
    }

    public class DriveToHeading extends Operation {
        double targetDegrees, maxTurnPower, maxDrivePower, targetDegreesAcceptableError, targetDistance;
        long lastTime = System.currentTimeMillis();
        Pid pidR, pidDrive;
        double startingEncoder=0.0;
        double targetEncoder;

        Boolean onTarget = false;
        Boolean halt = false;
        double deltaTime = .001;

        public DriveToHeading (double targetDegrees, double maxDrivePower, double maxTurnPower, double targetDegreesAcceptableError, long timeoutMS, double targetDistance) {
            this.targetDegrees=targetDegrees;
            this.maxTurnPower=maxTurnPower;
            this.maxDrivePower=maxDrivePower;
            this.timeoutMS=timeoutMS;
            this.targetDistance=targetDistance;
            this.targetDegreesAcceptableError=targetDegreesAcceptableError;

            // todo; add logging for distance
            RobotLog.i("DriveToHeading( targetDegrees=%f, maxDrivePower=%f, maxTurnPower=%f, targetDistance=%f) ", targetDegrees, maxDrivePower, maxTurnPower, targetDistance);
            // restart imu angle tracking.
            resetRelativeAngleToZero();

            double drivePidKp = 0.5;     // Tuning variable for PID.
            double drivePidTi = 0;   // Eliminate integral error in 1 sec.
            double drivePidTd = 0.2;   // Account for error in 0.1 sec. // Protect against integral windup by limiting integral term.

            double drivePidIntMax = 180.0;
            double drivePidIntMin = -drivePidIntMax;

            if (targetDistance!=0.0) {
              //  drive.resetEncoders();
                startingEncoder=drive.getEncoderClicksAbs();
                targetEncoder=startingEncoder+drive.convertInchesToClicks(targetDistance);
                RobotLog.i("DriveToHeading( startingEncoder=%f  targetEncoder=%f) ", startingEncoder, targetEncoder);
            }

            pidR = new Pid(drivePidKp, drivePidTi, drivePidTd, drivePidIntMin, drivePidIntMax, -maxTurnPower, maxTurnPower);
            RobotLog.i(pidR.toString());
        }

        public void done() {
            super.done();
            drive.stop();
            RobotLog.i("rotate2() done");
        }

        public boolean loop() {
            if (!super.loop()) return false;

            RobotLog.i("begin loop %d : %s", numLoops, pidR.toString());

            double relativeAngle = -getRelativeAngle();
            double deltaTime = ((double) (curMS - lastTime)) / 1000.0;

            RobotLog.i("update(%f, %f, %f)", targetDegrees, relativeAngle, deltaTime);

            double rotatePower = pidR.update(/*desired*/targetDegrees, /*actual*/relativeAngle, deltaTime);
            RobotLog.i("update= drivePower=%f  %s", rotatePower, pidR.toString());

            drive.driveFRS(maxDrivePower, rotatePower, 0.0);

            lastTime = curMS;

            double curEncoder=drive.getEncoderClicksAbs();

            RobotLog.i("DriveToHeading( startingEncoder=%f  targetEncoder=%f  currentEncoder=%f", startingEncoder, targetEncoder, curEncoder);

            // problem with this is that it needs to use a pid to scale back power to hit this distance accurately.
            if (targetDistance!=0.0 && isDistanceWithinRange(curEncoder,targetEncoder)) {
                onTarget = true;
                RobotLog.i("ON TARGET!");
            } else {
                RobotLog.i("NOT ON TARGET!");
            }

            telemetry.addData("angles", "target=" + targetDegrees + " relative=" + relativeAngle);
            telemetry.addData("targetD", "trgtD=" + targetDistance+ " trgtEnc=" + targetEncoder+ " curEnc="+curEncoder);
            telemetry.addData("power", maxDrivePower);
            telemetry.addData("time", "Delta=" + deltaTime + " total=" + (lastTime - startMS));
            telemetry.addData("pidR", pidR.toString());
            telemetry.addData("status", onTarget ? "On TARGET" : (halt ? "HALT" : "running"));
            RobotLog.i("onTarget=%s Halt=%s", onTarget.toString(), halt.toString());
            telemetry.update();

            if (onTarget) done();
            return !done;
        }
    }
    public class DriveToDistance extends Operation {
        double targetDegrees, maxTurnPower, maxDrivePower, targetDegreesAcceptableError, targetDistance;
        long lastTime = System.currentTimeMillis();
        Pid pidR, pidDrive;
        double startingEncoder=0.0;
        double targetEncoder;

        Boolean onTarget = false;
        Boolean halt = false;
        double deltaTime = .001;

        public DriveToDistance (double targetDegrees, double maxDrivePower, double maxTurnPower, double targetDegreesAcceptableError, long timeoutMS, double targetDistance) {
            this.targetDegrees=targetDegrees;
            this.maxTurnPower=maxTurnPower;
            this.maxDrivePower=maxDrivePower;
            this.timeoutMS=timeoutMS;
            this.targetDistance=targetDistance;
            this.targetDegreesAcceptableError=targetDegreesAcceptableError;

            // todo; add logging for distance
            RobotLog.i("DriveToDistance( targetDegrees=%f, maxDrivePower=%f, maxTurnPower=%f, targetDistance=%f) ", targetDegrees, maxDrivePower, maxTurnPower, targetDistance);
            // restart imu angle tracking.
            resetRelativeAngleToZero();

            double drivePidKp = 0.5;     // Tuning variable for PID.
            double drivePidTi = 0;   // Eliminate integral error in 1 sec.
            double drivePidTd = 0.2;   // Account for error in 0.1 sec. // Protect against integral windup by limiting integral term.

            double drivePidIntMax = 180.0;
            double drivePidIntMin = -drivePidIntMax;

//            if (targetDistance!=0.0) {
//                //  drive.resetEncoders();
//                startingEncoder=drive.getEncoderClicksAbs();
//                targetEncoder=startingEncoder+drive.convertInchesToClicks(targetDistance);
//                RobotLog.i("DriveToDistance( startingEncoder=%f  targetEncoder=%f) ", startingEncoder, targetEncoder);
//            }

            pidR = new Pid(drivePidKp, drivePidTi, drivePidTd, drivePidIntMin, drivePidIntMax, -maxTurnPower, maxTurnPower);
            RobotLog.i(pidR.toString());
        }

        public void done() {
            super.done();
            drive.stop();
            RobotLog.i("rotate2() done");
        }

        public boolean loop() {
            if (!super.loop()) return false;

            RobotLog.i("begin loop %d : %s", numLoops, pidR.toString());

            double relativeAngle = -getRelativeAngle();
            double deltaTime = ((double) (curMS - lastTime)) / 1000.0;

            RobotLog.i("update(%f, %f, %f)", targetDegrees, relativeAngle, deltaTime);

            double rotatePower = pidR.update(/*desired*/targetDegrees, /*actual*/relativeAngle, deltaTime);
            RobotLog.i("update= drivePower=%f  %s", rotatePower, pidR.toString());

            drive.driveFRS(maxDrivePower, 0.0, 0.0);

            lastTime = curMS;

            double curEncoder=drive.getEncoderClicksAbs();

            RobotLog.i("DriveToDistance( startingEncoder=%f  targetEncoder=%f  currentEncoder=%f", startingEncoder, targetEncoder, curEncoder);

            // problem with this is that it needs to use a pid to scale back power to hit this distance accurately.
           // if (targetDistance!=0.0 && isDistanceWithinRange(curEncoder,targetEncoder)) {

            double fd= frontDistance.getDistance(DistanceUnit.INCH);
            double sd= sideDistance.getDistance(DistanceUnit.INCH);
            telemetry.addData("Distance", "front=" + fd + " side=" + sd);
            telemetry.update();

            if(frontDistance.getDistance(DistanceUnit.INCH)<= targetDistance){

                onTarget = true;
                RobotLog.i("ON TARGET!");
            } else {
                RobotLog.i("NOT ON TARGET!");
            }

            telemetry.addData("angles", "target=" + targetDegrees + " relative=" + relativeAngle);
            telemetry.addData("targetD", "trgtD=" + targetDistance+ " trgtEnc=" + targetEncoder+ " curEnc="+curEncoder);
            telemetry.addData("power", maxDrivePower);
            telemetry.addData("time", "Delta=" + deltaTime + " total=" + (lastTime - startMS));
            telemetry.addData("pidR", pidR.toString());
            telemetry.addData("status", onTarget ? "On TARGET" : (halt ? "HALT" : "running"));
            RobotLog.i("onTarget=%s Halt=%s", onTarget.toString(), halt.toString());
            telemetry.update();

            if (onTarget) done();
            return !done;
        }
    }



    Operation getOperationRotateToHeading(double targetDegrees, double maxPower, double targetDegreesAcceptableError, long timeoutMS) {
        return new RotateToHeading(targetDegrees, maxPower, targetDegreesAcceptableError, timeoutMS);
    }

    Operation getOperationDriveToHeading(double targetDegrees, double maxDrivePower, double maxTurnPower, double targetDegreesAcceptableError, long timeoutMS, double targetDistance) {
        return new DriveToHeading(targetDegrees, maxDrivePower, maxTurnPower, targetDegreesAcceptableError, timeoutMS, targetDistance);
    }

    Operation getOperationDriveToDistance(double targetDegrees, double maxDrivePower, double maxTurnPower, double targetDegreesAcceptableError, long timeoutMS, double targetDistance) {
        return new DriveToDistance(targetDegrees, maxDrivePower, maxTurnPower, targetDegreesAcceptableError, timeoutMS, targetDistance);
    }

    public void logSensors()
    {
       double fd= frontDistance.getDistance(DistanceUnit.CM);
       double sd= sideDistance.getDistance(DistanceUnit.CM);
       telemetry.addData("Distance", "front=" + fd + " side=" + sd);
    }

    protected void sleep(long millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            // eat it
        }

    }
}

