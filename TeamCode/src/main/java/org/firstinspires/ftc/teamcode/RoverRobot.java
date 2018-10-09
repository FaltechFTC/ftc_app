package org.firstinspires.ftc.teamcode;

import com.github.pmtischler.control.Pid;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class RoverRobot {

    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();
    public Telemetry telemetry = null;

    IDrive drive = null;
    BNO055IMU imu = null;
    Orientation lastAngles = new Orientation();
    public final AxesOrder axesOrder = AxesOrder.ZYX;
    double globalAngle;

    //Constructor//
    public RoverRobot(IDrive drive) {
        this.drive = drive;
    }

    public void init(HardwareMap hwMap, Telemetry telemetry) {
        // Save reference to Hardware map
        this.hwMap = hwMap;
        this.telemetry = telemetry;

        drive.init(hwMap, telemetry);
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
                ; // eat the exception
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

    public void rotate2(double targetDegrees, double maxPower) {
        rotate2(targetDegrees, maxPower, 6000);
    }

    public void rotate2(double targetDegrees, double maxPower, double maxTimeMS) {
        RobotLog.i("rotate2( targetDegrees=%f, maxPower=%f) ", targetDegrees, maxPower);
        if (targetDegrees == 0.0) return;

        // restart imu angle tracking.
        resetRelativeAngleToZero();

        long startTime = System.currentTimeMillis();

        double drivePidKp = 1.0;     // Tuning variable for PID.
        double drivePidTi = 0.5;   // Eliminate integral error in 1 sec.
        double drivePidTd = 0.5;   // Account for error in 0.1 sec. // Protect against integral windup by limiting integral term.

        double drivePidIntMax = 180.0;
        double drivePidIntMin = -drivePidIntMax;
        double targetDegreesAcceptableError = 1.0;

        Pid pidR = new Pid(drivePidKp, drivePidTi, drivePidTd, drivePidIntMin, drivePidIntMax, -maxPower, maxPower);
        RobotLog.i(pidR.toString());

        double direction = 1.0;  // (degrees>=0.0) ? 1.0 : -1.0;
        double deltaTime = .001;
        long lastTime = System.currentTimeMillis();
        Boolean onTarget = false;
        Boolean halt = false;
        int loops = 0;
        do {

            RobotLog.i("begin loop %d : %s", loops, pidR.toString());

            double relativeAngle = -getRelativeAngle();
            double absoluteAngle = getCurrentAbsoluteAngle();

            long curTime = System.currentTimeMillis();
            deltaTime = ((double) (curTime - lastTime)) / 1000.0;

            RobotLog.i("target=%f  Relative=%f, Absolute=%f", targetDegrees, relativeAngle, absoluteAngle);
            RobotLog.i("update(%f, %f, %f)", targetDegrees, relativeAngle, deltaTime);

            double rotatePower = pidR.update(/*desired*/targetDegrees, /*actual*/relativeAngle, deltaTime);
            RobotLog.i("update= drivePower=%f  %s", rotatePower, pidR.toString());

            drive.driveFRS(0.0, rotatePower, 0.0);

            lastTime = curTime;

            if (Math.abs(relativeAngle - targetDegrees) < targetDegreesAcceptableError)
                onTarget = true;
            if (curTime - startTime > maxTimeMS) halt = true;

            telemetry.addData("angles", "target=" + targetDegrees + " relative=" + relativeAngle + " absolute" + absoluteAngle);
            telemetry.addData("power", rotatePower);
            telemetry.addData("time", "Delta=" + deltaTime + " total=" + (lastTime - startTime));
            telemetry.addData("pidR", pidR.toString());
            telemetry.addData("status", onTarget ? "On TARGET" : (halt ? "HALT" : "running"));
            RobotLog.i("onTarget=%s Halt=%s", onTarget.toString(), halt.toString());
            telemetry.update();
            sleep(1); // maybe not necessary
            loops++;
        } while (!onTarget && !halt);

        drive.stop();
        RobotLog.i("rotate2() done");
    }


    public void driveToHeading(double targetDegrees, double maxDrivePower, double maxTurningPower, double maxTimeMS) {
        RobotLog.i("driveToHeading( targetDegrees=%f, maxPower=%f, maxTurningPower=%f) ", targetDegrees, maxDrivePower, maxTurningPower);
        //   if (targetDegrees==0.0) return ;

        // restart imu angle tracking.
        resetRelativeAngleToZero();

        long startTime = System.currentTimeMillis();

        double drivePidKp = 1.0;     // Tuning variable for PID.
        double drivePidTi = 0.5;   // Eliminate integral error in 1 sec.
        double drivePidTd = 0.5;   // Account for error in 0.1 sec. // Protect against integral windup by limiting integral term.

        double drivePidIntMax = 180.0;
        double drivePidIntMin = -drivePidIntMax;
        double targetDegreesAcceptableError = 1.0;

        Pid pidR = new Pid(drivePidKp, drivePidTi, drivePidTd, drivePidIntMin, drivePidIntMax, -maxTurningPower, maxTurningPower);
        RobotLog.i(pidR.toString());

        double direction = 1.0;  // (degrees>=0.0) ? 1.0 : -1.0;
        double deltaTime = .001;
        long lastTime = System.currentTimeMillis();
        Boolean onTarget = false;
        Boolean halt = false;
        int loops = 0;
        do {

            RobotLog.i("begin loop %d : %s", loops, pidR.toString());

            double relativeAngle = -getRelativeAngle();
            double absoluteAngle = getCurrentAbsoluteAngle();

            long curTime = System.currentTimeMillis();
            deltaTime = ((double) (curTime - lastTime)) / 1000.0;

            RobotLog.i("target=%f  Relative=%f, Absolute=%f", targetDegrees, relativeAngle, absoluteAngle);
            RobotLog.i("update(%f, %f, %f)", targetDegrees, relativeAngle, deltaTime);

            double rotatePower = pidR.update(/*desired*/targetDegrees, /*actual*/relativeAngle, deltaTime);
            RobotLog.i("update= drivePower=%f  %s", rotatePower, pidR.toString());

            drive.driveFRS(maxDrivePower, rotatePower, 0.0);

            lastTime = curTime;

            if (curTime - startTime > maxTimeMS) halt = true;

            telemetry.addData("angles", "target=" + targetDegrees + " relative=" + relativeAngle + " absolute" + absoluteAngle);
            telemetry.addData("rotatePower", rotatePower);
            telemetry.addData("time", "Delta=" + deltaTime + " total=" + (lastTime - startTime));
            telemetry.addData("pidR", pidR.toString());
            telemetry.addData("status", onTarget ? "On TARGET" : (halt ? "HALT" : "running"));
            RobotLog.i("onTarget=%s Halt=%s", onTarget.toString(), halt.toString());
            telemetry.update();
            sleep(1); // maybe not necessary
            loops++;
        } while (!onTarget && !halt);

        drive.stop();
        RobotLog.i("driveToHeading() done");
    }

    protected void sleep(long millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            ; // eat it
        }

    }
}

