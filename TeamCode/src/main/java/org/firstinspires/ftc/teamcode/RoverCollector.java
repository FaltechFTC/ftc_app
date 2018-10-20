package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RoverCollector {
    HardwareMap hwMap = null;
    IDrive drive = null;

    public DcMotor mtrLeftCollector = null;
    public DcMotor mtrRightCollector = null;

    public Servo leftClaw = null;
    public Servo rightClaw = null;

    private ElapsedTime period = new ElapsedTime();
    public Telemetry telemetry = null;
    public RoverCollector(){
    }

    public void init(HardwareMap hwMap, Telemetry telemetry) {
        // Save reference to Hardware map
        this.hwMap = hwMap;
        this.telemetry = telemetry;

        mtrLeftCollector = hwMap.get(DcMotor.class, "mtrLeftCollector");
        mtrRightCollector = hwMap.get(DcMotor.class, "mtrRightCollector");

        mtrLeftCollector.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrRightCollector.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // TODO: Initialize servo motors.
        leftClaw = hwMap.get(Servo.class, "leftClaw");
        rightClaw = hwMap.get(Servo.class, "rightClaw");
    }

    public void setClawsToCloselPosition(){
        leftClaw.setPosition(0.0);
        rightClaw.setPosition(0.0);
    }
    public void setClawsToOpenPosition(){
        leftClaw.setPosition(1.0);
        rightClaw.setPosition(1.0);
    }

}
