package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/**
 *
 * Borrowed heavily from original author and source from https://github.com/pmtischler/ftc_app/blob/master/SharedCode/src/main/java/com/github/pmtischler/control/Mecanum.java
 *
 *  Modified by Faltech 7079 ...
 *
 * Mecanum wheel drive calculations.
 * Input controls:
 *   V_d = desired robot speed.
 *   theta_d = desired robot velocity angle.
 *   V_theta = desired robot rotational speed.
 *
 *  Example:
 *    // Convert joysticks to wheel powers.
 *    Mecanum.Wheels wheels = Mecanum.motionToWheels(
 *        Mecanum.joystickToMotion(
 *            gamepad1.left_stick_x, gamepad1.left_stick_y,
 *            gamepad1.right_stick_x, gamepad1.right_stick_y));
 *    // Set power on the motors.
 *    frontLeftMotor.setPower(wheels.frontLeft);
 */
public class DriveTank extends IDrive{

    public DcMotor mtrFL = null;
    public DcMotor mtrFR = null;
    public DcMotor[] motors=null;

    HardwareMap hwMap = null;
    public Telemetry telemetry = null;

    private ElapsedTime runTime = new ElapsedTime();

    public DriveTank() {

    }


    @Override
    public void init(HardwareMap hwMap, Telemetry telemetry) {
        // Save reference to Hardware map
        this.hwMap = hwMap;
        this.telemetry = telemetry;

        // Define and Initialize Motors
        mtrFL = hwMap.get(DcMotor.class, "mtrFL");
        mtrFR = hwMap.get(DcMotor.class, "mtrFR");

        motors = new DcMotor[] {mtrFL, mtrFR};

        // set motor direction based on installed orientation.
        mtrFL.setDirection(DcMotor.Direction.FORWARD);
        mtrFR.setDirection(DcMotor.Direction.REVERSE);


        setHaltModeCoast(true);
        setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stop();
    }



    @Override
    public void driveFRS(double forward, double rotate, double sideways,double maxPower) {
        mtrFL.setPower(forward+rotate);
        mtrFR.setPower(forward-rotate);
        //TODO NEEDS CLAMP
    }

    @Override
    public void setHaltModeCoast(boolean coastModeOn) {
        DcMotor.ZeroPowerBehavior behavior;
        if (coastModeOn) behavior=DcMotor.ZeroPowerBehavior.FLOAT;
        else behavior=DcMotor.ZeroPowerBehavior.BRAKE;

        for (DcMotor m : motors) m.setZeroPowerBehavior(behavior);
    }

    @Override
    public void setRunModeEncoder(boolean encoderModeOn) {
        if (encoderModeOn) setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        else setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setRunMode(DcMotor.RunMode runMode) {
        for (DcMotor m : motors) m.setMode(runMode);
    }

    @Override
    public void stop() {
        for (DcMotor m : motors) m.setPower(0);
    }
    private void setRobotMode(){
//        mtrBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        mtrBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Set them to run to position
//        mtrBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        mtrBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    double motor_count = 1440;
    double gear_reduction = 0.5;
    double wheel_circumference = 11.12; // the diameter is 90mm
    double counts_per_inch = (motor_count * gear_reduction / wheel_circumference);

    private void setTargetPositions(double inches){
        double BLtarget;
        double BRtarget;
        double FLtarget;
        double FRtarget;

//        BLtarget = mtrBL.getCurrentPosition() + (inches * counts_per_inch);
//        BRtarget = mtrBR.getCurrentPosition() + (inches * counts_per_inch);
        FLtarget = mtrFL.getCurrentPosition() + (inches * counts_per_inch);
        FRtarget = mtrFR.getCurrentPosition() + (inches * counts_per_inch);

//        mtrBL.setTargetPosition((int) BLtarget);
//        mtrBR.setTargetPosition((int) BRtarget);
        mtrFL.setTargetPosition((int) FLtarget);
        mtrFR.setTargetPosition((int) FRtarget);

    }

    private void timeOutExit(double timeout){

        while ((runTime.seconds() < (timeout))
                //               && (mtrBL.isBusy() &&mtrBR.isBusy()
                && mtrFL.isBusy() && mtrFR.isBusy()) {

            // Display it for the driver.
            telemetry.addData("Path1",  "Running to target position");
            telemetry.addData("Path2",  "Running at:",
//                    mtrBL.getCurrentPosition(),
//                    mtrBR.getCurrentPosition(),
                    mtrFL.getCurrentPosition(),
                    mtrFR.getCurrentPosition());
            telemetry.update();
        }

    }

    private void resetEncoderMode(){
//        mtrBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        mtrBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public void goInches(double inches, double speed, double power, double timeout){
        runTime.reset();
        setRobotMode();
        setTargetPositions(inches);
        driveFRS(speed, 0.0, 0.0, power);
        timeOutExit(timeout);
        resetEncoderMode();
        stop();
    }

}
