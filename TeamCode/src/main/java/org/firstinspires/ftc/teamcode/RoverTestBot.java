package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RoverTestBot {
    public DcMotor mtrFL = null;
    public DcMotor mtrFR = null;
    public DistanceSensor rangeSensor = null;
    public BNO055IMU imu = null;

    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();
    public Telemetry myTelemetry = null;

    public RoverTestBot() {

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
        rangeSensor = hwMap.get(DistanceSensor.class, "rangeSensor");
        imu = hwMap.get(BNO055IMU.class, "imu");

        // Set all motors to zero power
        mtrFL.setPower(0);
        mtrFR.setPower(0);

        mtrFL.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        mtrFR.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        mtrFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
    public void forwardTeleOp(double power){
        mtrFR.setPower(power);
        mtrFL.setPower(power);

    }
    public void strafeTeleOp(double power){
        mtrFR.setPower(-power);
        mtrFL.setPower(power);

    }
    public void turnTeleOp(double power){
        mtrFR.setPower(power);
        mtrFL.setPower(power);

    }

}
