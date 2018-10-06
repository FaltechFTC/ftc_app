package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

@TeleOp(name="RoverTestTeleOp", group="7079")
public class RoverTestTeleOp extends OpMode{
    RoverTestBot robot = new RoverTestBot();





    @Override
    public void init() {
        robot.init(hardwareMap, telemetry);
        telemetry.addData("Robot" , "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        double fwdBack_D = -gamepad1.right_stick_y;
        robot.mtrFR.setPower(fwdBack_D);
        robot.mtrFL.setPower(fwdBack_D);
        telemetry.addData("Distance (cm)",
                String.format(Locale.US, "%.02f", robot.rangeSensor.getDistance(DistanceUnit.CM)));

        telemetry.update();
    }
}
