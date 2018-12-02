package org.firstinspires.ftc.teamcode;

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
    // Using this variable to track the angle from the start position
    double relativeAngleFromStart;
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
       // teamMarker.setPosition(0);
        //if(isAutonomous)
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
        relativeAngleFromStart += lastAngles.firstAngle;
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

    public boolean isDistanceWithinRange(double startingEncoder, double targetEncoder){
        if (Math.abs(startingEncoder - targetEncoder)<= drive.convertInchesToClicks(1)) return true;
        return false;
    }

    Operation getOperationRotateToHeading(double targetDegrees, double maxPower, double targetDegreesAcceptableError, long timeoutMS) {
        return new OpRotateToHeading(this, targetDegrees, maxPower, targetDegreesAcceptableError, timeoutMS);
    }

    Operation getOperationDriveToHeading(double targetDegrees, double maxDrivePower, double maxTurnPower, double targetDegreesAcceptableError, long timeoutMS, double targetDistance) {
        return new OpDriveToHeading(this, targetDegrees, maxDrivePower, maxTurnPower, targetDegreesAcceptableError, timeoutMS, targetDistance);
    }

    Operation getOperationDriveToDistance(double targetDegrees, double maxDrivePower, double maxTurnPower, double targetDegreesAcceptableError, long timeoutMS, double targetDistance) {
        return new OpDriveToDistance(this, targetDegrees, maxDrivePower, maxTurnPower, targetDegreesAcceptableError, timeoutMS, targetDistance);
    }
    Operation getOperationWallDrive(double targetDegrees, double maxDrivePower, double maxTurnPower, double maxStrafePower, long timeoutMS, double targetDistance, double targetDistanceToWall) {
        return new OpWallride(this,    targetDegrees, maxDrivePower, maxTurnPower, maxStrafePower, timeoutMS,  targetDistance, targetDistanceToWall);

    }

    // OpWallride(RoverRobot robot, double targetDegrees, double maxDrivePower, double maxTurnPower, long timeoutMS, double targetDistance, double targetDistanceToWall)
    public void logSensors()
    {
       double fd= frontDistance.getDistance(DistanceUnit.INCH);
       double sd= sideDistance.getDistance(DistanceUnit.INCH);
       telemetry.addData("Distance", "(inches) front=" + fd + " side=" + sd);
    }

    protected void sleep(long millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}

