package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
    public void driveFRS(double forward, double rotate, double sideways) {
        mtrFL.setPower(forward+rotate);
        mtrFR.setPower(forward-rotate);
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

}
