package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
@TeleOp(name="RoverTeleOp", group="7079")
public class FaltechRoverTeleOp extends OpMode{
    FaltechRoverRobot robot = new FaltechRoverRobot();
    @Override
    public void init() {
        robot.init(hardwareMap, telemetry);
        telemetry.addData("Robot" , "Initialized");
        telemetry.update();
    }

    public double WeightAvg(double x, double y, double z) {
        double speed_D = 0;


        if ((Math.abs(x) + Math.abs(y) + Math.abs(z))  != 0.0) {
            speed_D = ((x * Math.abs(x)) + (y * Math.abs(y)) + (z * Math.abs(z)))
                    / (Math.abs(x) + Math.abs(y) + Math.abs(z));
        }
        return (speed_D);
    }


    @Override
    public void loop() {

        double FwdBack_D = -gamepad1.right_stick_y;
        double Turn_D = gamepad1.left_stick_x;
        double Strafe_D = -gamepad1.right_stick_x;
        double deadzone_D = 0.1;

        if (Math.abs(FwdBack_D) > deadzone_D){
            FwdBack_D = FwdBack_D;
        }
        else {
            FwdBack_D = 0;
        }
        if (Math.abs(Strafe_D) > deadzone_D ){
            Strafe_D = Strafe_D;
        }
        else {
            Strafe_D = 0;
        }

        if (Math.abs(Turn_D) > deadzone_D){
            Turn_D = Turn_D;
        }
        else{
            Turn_D = 0;
        }
        robot.mtrFR.setPower(WeightAvg(FwdBack_D,Strafe_D,-Turn_D));
        robot.mtrFL.setPower(WeightAvg(FwdBack_D,-Strafe_D,Turn_D));
        robot.mtrBR.setPower(WeightAvg(FwdBack_D,-Strafe_D,-Turn_D));
        robot.mtrBL.setPower(WeightAvg(FwdBack_D,Strafe_D,Turn_D));
    }
}
