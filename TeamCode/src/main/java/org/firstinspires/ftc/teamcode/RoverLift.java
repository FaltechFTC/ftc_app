/*
This class has the information related to our lift on the robot.  We mainly have intialization
Our robot lift currently is working based on timing.  In future we may have to implement based on encoders
to get more accuracy.
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RoverLift {
    HardwareMap hwMap = null;
    public DcMotor mtrRoverLift = null;
    private ElapsedTime period = new ElapsedTime();
    public Telemetry telemetry = null;

    public void init(HardwareMap hwMap, Telemetry telemetry, boolean isAutonomous) {
        // Save reference to Hardware map
        this.hwMap = hwMap;
        this.telemetry = telemetry;

        mtrRoverLift = hwMap.get(DcMotor.class, "mtrRoverLift");
        mtrRoverLift.setDirection(DcMotor.Direction.REVERSE);
        mtrRoverLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrRoverLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrRoverLift.setPower(0);

//        if (isAutonomous) {
//            mtrRoverLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//           mtrRoverLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//      }
    }
    double encoderTick = 560;
    double diameter = .5;
    double encoderPerInch = encoderTick / (diameter * Math.PI);

    public void setTargetPosition(double inches){
        double targetPosition = mtrRoverLift.getCurrentPosition() + (encoderPerInch * inches);
        mtrRoverLift.setTargetPosition((int) targetPosition);
    }

    public void setPower(double power){
        mtrRoverLift.setPower(power);
    }

}
