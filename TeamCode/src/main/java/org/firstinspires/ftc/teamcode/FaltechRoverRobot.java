package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FaltechRoverRobot {

    public DcMotor mtrFL = null;
    public DcMotor mtrFR = null;
    public DcMotor mtrBL = null;
    public DcMotor mtrBR = null;

    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();
    public Telemetry myTelemetry = null;

    //Constructor//
    public FaltechRoverRobot() {

    }

    public void init(HardwareMap ahwMap, Telemetry myTelemetry) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        initDrive();
    }

    private void initDrive() {
        // Define and Initialize Motors
        mtrFL = hwMap.get(DcMotor.class, "mtrFL");
        mtrFR = hwMap.get(DcMotor.class, "mtrFR");
        mtrBL = hwMap.get(DcMotor.class, "mtrBL");
        mtrBR = hwMap.get(DcMotor.class, "mtrBR");

        // Set all motors to zero power
        mtrFL.setPower(0);
        mtrFR.setPower(0);
        mtrBL.setPower(0);
        mtrBR.setPower(0);

        mtrFL.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        mtrFR.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        mtrBL.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        mtrBR.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        mtrFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void forwardTeleOp(double power){
        mtrFR.setPower(power);
        mtrFL.setPower(power);
        mtrBR.setPower(power);
        mtrBL.setPower(power);
    }
    public void strafeTeleOp(double power){
    mtrFR.setPower(-power);
    mtrFL.setPower(power);
    mtrBR.setPower(power);
    mtrBL.setPower(-power);
    }
    public void turnTeleOp(double power){
        mtrFR.setPower(power);
        mtrBR.setPower(power);
        mtrFL.setPower(-power);
        mtrBL.setPower(-power);
    }
}

