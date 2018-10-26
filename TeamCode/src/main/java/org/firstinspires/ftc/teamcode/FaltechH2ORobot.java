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
/**
 * This is NOT an opmode.
 * <p>
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 * <p>
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 * <p>
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class FaltechH2ORobot {

    Telemetry telemetry=null;

    /* Public OpMode members. */
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;


//    public Servo clawHinge = null;
//    public Servo claw = null;
//    public Servo dude = null;

    public static final double MID_SERVO = 0.5;
    public static final double ARM_UP_POWER = 0.45;
    public static final double ARM_DOWN_POWER = -0.45;

    //  DigitalChannel          touch;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    AxesOrder axesOrder= AxesOrder.ZYX;
    double                  globalAngle, power = .30, correction;
    boolean                 aButton, bButton, touched;
    PIDController           pidDrive, pidRotate;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public FaltechH2ORobot() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        init(ahwMap,null);
    }
    public void init(HardwareMap ahwMap, Telemetry telemetry) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        this.telemetry=telemetry;
        initDrive();
        initIMU();
        initDude();
        initClaw();
    }

    private void initDrive() {
        // Define and Initialize Motors
        leftDrive = hwMap.get(DcMotor.class, "mtrFL");
        rightDrive = hwMap.get(DcMotor.class, "mtrFR");

        leftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void initIMU() {
        RobotLog.i("initIMU() start");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        // Set PID proportional value to start reducing power at about 50 degrees of rotation.
        pidRotate = new PIDController(1, .5, 0.01);

        // Set PID proportional value to produce non-zero correction value when robot veers off
        // straight line. P value controls how sensitive the correction is.
        pidDrive = new PIDController(.05, 0, 0);

        // make sure the imu gyro is calibrated before continuing.
        waitForGyroCalibration();
        resetRelativeAngleToZero();

        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.addData("angles", "relative="+getRelativeAngle()+" absolute"+getCurrentAbsoluteAngle());
        telemetry.update();

        resetRelativeAngleToZero();

    }

    protected void waitForGyroCalibration() {
        RobotLog.i("Calibrating...");

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();
        while (!imu.isGyroCalibrated())
        {
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
    private void initDude() {
//        dude = hwMap.get(Servo.class, "dude");
//        dude.setPosition(1.0);
    }

    private void initClaw() {
//        clawHinge = hwMap.get(Servo.class, "clawHinge");
//        clawHinge.setPosition(MID_SERVO);
//
//        claw = hwMap.get(Servo.class, "claw");
//        claw.setPosition(MID_SERVO);
    }



    /**
     * Resets the cumulative angle tracking to zero.
     */
    protected void resetRelativeAngleToZero()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, axesOrder, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right from zero point.
     */
    protected double getRelativeAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, axesOrder, AngleUnit.DEGREES);
        double deltaAngle = mapDegreesTo180(angles.firstAngle - lastAngles.firstAngle);
        return deltaAngle;
    }

    /* should allow to get a angle relative to the reset that goes past 180 and beyond.*/
    private double getAngleViaIncrementals()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

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

    protected double getCurrentAbsoluteAngle()
    {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, axesOrder, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    public void rotate(double degrees, double maxPower)
    {
        if (degrees==0.0) return ;
        // restart imu angle tracking.
        resetRelativeAngleToZero();


        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle with a minimum of 20%.
        // This is to prevent the robots momentum from overshooting the turn after we turn off the
        // power. The PID controller reports onTarget() = true when the difference between turn
        // angle and target angle is within 2% of target (tolerance). This helps prevent overshoot.
        // The minimum power is determined by testing and must enough to prevent motor stall and
        // complete the turn. Note: if the gap between the starting power and the stall (minimum)
        // power is small, overshoot may still occur. Overshoot is dependant on the motor and
        // gearing configuration, starting power, weight of the robot and the on target tolerance.

        pidRotate.reset();
        pidRotate.setSetpoint(-degrees);
        pidRotate.setInputRange(0, 90);
        pidRotate.setOutputRange(.20, maxPower);
        pidRotate.setTolerance(3);
        pidRotate.enable();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).
        // rotate until turn is completed.

        double direction= (degrees>=0.0) ? 1.0 : -1.0;
        do
        {
            double relativeAngle = getRelativeAngle();
            double currentAngle = getCurrentAbsoluteAngle();
            double drivepower = pidRotate.performPID(relativeAngle); // power will be - on right turn.
            leftDrive.setPower(direction *drivepower);
            rightDrive.setPower(-direction * drivepower);
            telemetry.addData(" last first angle", lastAngles.firstAngle);
            telemetry.addData("relativeAngle", relativeAngle);
            telemetry.addData("currentAngle", currentAngle);
            telemetry.addData("Power is", drivepower);
            telemetry.addData(" correction", correction);
            dumpPID("pidR",pidRotate);
            telemetry.update();

        } while (/*TODO: figure out interrupt opModeIsActive() &&*/ !pidRotate.onTarget());

        // turn the motors off.
        rightDrive.setPower(0);
        leftDrive.setPower(0);

    }

    protected  void dumpPID (String caption, PIDController pid) {
        telemetry.addData(caption, "P="+pid.getP()+", I="+pid.getI()+", D="+pid.getD()+", E="+pid.getError()+", SP="+pid.getSetpoint());
    }


    public void rotate2(double targetDegrees, double maxPower) {
        rotate2(targetDegrees,maxPower,6000);
    }
    public void rotate2(double targetDegrees, double maxPower, double maxTimeMS) {
        RobotLog.i("rotate2( targetDegrees=%f, maxPower=%f) ", targetDegrees, maxPower);
        if (targetDegrees==0.0) return ;

        // restart imu angle tracking.
        resetRelativeAngleToZero();

        long startTime=System.currentTimeMillis();

        double drivePidKp = 1.0;     // Tuning variable for PID.
        double drivePidTi = 0.5;   // Eliminate integral error in 1 sec.
        double drivePidTd = 0.5;   // Account for error in 0.1 sec. // Protect against integral windup by limiting integral term.

        double drivePidIntMax = 180.0;
        double drivePidIntMin = -drivePidIntMax;
        double targetDegreesAcceptableError = 1.0;

        Pid pidR = new Pid(drivePidKp, drivePidTi, drivePidTd, drivePidIntMin, drivePidIntMax, -maxPower, maxPower);
        RobotLog.i(pidR.toString());

        double direction= 1.0;  // (degrees>=0.0) ? 1.0 : -1.0;
        double deltaTime=.001;
        long lastTime=System.currentTimeMillis();
        Boolean onTarget=false;
        Boolean halt=false;
        int loops=0;
        do
        {

            RobotLog.i("begin loop %d : %s", loops, pidR.toString());

            double relativeAngle = -getRelativeAngle();
            double absoluteAngle = getCurrentAbsoluteAngle();

            long curTime=System.currentTimeMillis();
            deltaTime=((double)(curTime-lastTime))/1000.0;

            RobotLog.i("target=%f  Relative=%f, Absolute=%f",targetDegrees, relativeAngle, absoluteAngle);
            RobotLog.i("update(%f, %f, %f)",targetDegrees,relativeAngle, deltaTime);

            double drivePower= pidR.update(/*desired*/targetDegrees, /*actual*/relativeAngle, deltaTime);
            RobotLog.i("update= drivePower=%f  %s",drivePower, pidR.toString());

            leftDrive.setPower(direction *drivePower);
            rightDrive.setPower(-direction * drivePower);

            lastTime=curTime;

            if (Math.abs(relativeAngle-targetDegrees)<targetDegreesAcceptableError) onTarget=true;
            if (curTime-startTime > maxTimeMS)  halt=true;

            telemetry.addData("angles", "target="+targetDegrees+" relative="+relativeAngle+" absolute"+absoluteAngle);
            telemetry.addData("power", drivePower);
            telemetry.addData("time", "Delta="+deltaTime+" total="+(lastTime-startTime));
            telemetry.addData("pidR", pidR.toString());
            telemetry.addData("status",onTarget?"On TARGET": (halt?"HALT":"running"));
            RobotLog.i("onTarget=%s Halt=%s", onTarget.toString(), halt.toString());
            telemetry.update();
            sleep(1); // maybe not necessary
            loops++;
        } while (!onTarget && !halt);

        // turn the motors off.
        rightDrive.setPower(0);
        leftDrive.setPower(0);
        sleep(1000);
        RobotLog.i("rotate2() done");
    }


    public void driveToHeading(double targetDegrees, double maxDrivePower, double maxTurningPower, double maxTimeMS) {
        RobotLog.i("driveToHeading( targetDegrees=%f, maxPower=%f, maxTurningPower=%f) ", targetDegrees, maxDrivePower, maxTurningPower);
     //   if (targetDegrees==0.0) return ;

        // restart imu angle tracking.
        resetRelativeAngleToZero();

        long startTime=System.currentTimeMillis();

        double drivePidKp = 1.0;     // Tuning variable for PID.
        double drivePidTi = 0.5;   // Eliminate integral error in 1 sec.
        double drivePidTd = 0.5;   // Account for error in 0.1 sec. // Protect against integral windup by limiting integral term.

        double drivePidIntMax = 180.0;
        double drivePidIntMin = -drivePidIntMax;
        double targetDegreesAcceptableError = 1.0;

        Pid pidR = new Pid(drivePidKp, drivePidTi, drivePidTd, drivePidIntMin, drivePidIntMax, -maxTurningPower, maxTurningPower);
        RobotLog.i(pidR.toString());

        double direction= 1.0;  // (degrees>=0.0) ? 1.0 : -1.0;
        double deltaTime=.001;
        long lastTime=System.currentTimeMillis();
        Boolean onTarget=false;
        Boolean halt=false;
        int loops=0;
        do
        {

            RobotLog.i("begin loop %d : %s", loops, pidR.toString());

            double relativeAngle = -getRelativeAngle();
            double absoluteAngle = getCurrentAbsoluteAngle();

            long curTime=System.currentTimeMillis();
            deltaTime=((double)(curTime-lastTime))/1000.0;

            RobotLog.i("target=%f  Relative=%f, Absolute=%f",targetDegrees, relativeAngle, absoluteAngle);
            RobotLog.i("update(%f, %f, %f)",targetDegrees,relativeAngle, deltaTime);

            double drivePower= pidR.update(/*desired*/targetDegrees, /*actual*/relativeAngle, deltaTime);
            RobotLog.i("update= drivePower=%f  %s",drivePower, pidR.toString());

            leftDrive.setPower(direction *drivePower + maxDrivePower);
            rightDrive.setPower(-direction * drivePower + maxDrivePower);

            lastTime=curTime;

//            if (Math.abs(relativeAngle-targetDegrees)<targetDegreesAcceptableError) onTarget=true;
            if (curTime-startTime > maxTimeMS)  halt=true;

            telemetry.addData("angles", "target="+targetDegrees+" relative="+relativeAngle+" absolute"+absoluteAngle);
            telemetry.addData("power", drivePower);
            telemetry.addData("time", "Delta="+deltaTime+" total="+(lastTime-startTime));
            telemetry.addData("pidR", pidR.toString());
            telemetry.addData("status",onTarget?"On TARGET": (halt?"HALT":"running"));
            RobotLog.i("onTarget=%s Halt=%s", onTarget.toString(), halt.toString());
            telemetry.update();
            sleep(1); // maybe not necessary
            loops++;
        } while (!onTarget && !halt);

        // turn the motors off.
        rightDrive.setPower(0);
        leftDrive.setPower(0);
        sleep(1000);
        RobotLog.i("driveToHeading() done");
    }

    protected void sleep(long millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            // eat it
        }

    }
}

