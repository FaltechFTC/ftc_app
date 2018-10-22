package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RoverLift {
    HardwareMap hwMap = null;
    public DcMotor mtrRoverLift = null;
    private ElapsedTime period = new ElapsedTime();
    public Telemetry telemetry = null;

    public void init(HardwareMap hwMap, Telemetry telemetry) {
        // Save reference to Hardware map
        this.hwMap = hwMap;
        this.telemetry = telemetry;

        mtrRoverLift = hwMap.get(DcMotor.class, "mtrRoverLift");
        mtrRoverLift.setDirection(DcMotor.Direction.FORWARD);
//        mtrRoverLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        mtrRoverLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    double encoderTick = 1120;
    double diameter = 1;
    double encoderPerInch = encoderTick / (diameter * Math.PI);

    public void setTargetPosition(double inches){
        double targetPosition = mtrRoverLift.getCurrentPosition() + (encoderPerInch * inches);
        mtrRoverLift.setTargetPosition((int) targetPosition);
    }

    public void setPower(double power){
        mtrRoverLift.setPower(power);
    }

}
