package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

public class RoverTeleop extends OpMode{
    @TeleOp(name="RoverMecanumTeleop", group="7079")
    public static class RoverMecanumTeleop extends RoverTeleop {

        @Override public void init() {
            robot = new RoverRobot(new DriveMecanum());
            roverCollector = new RoverCollector();

            super.init();
        }
    }

    @TeleOp(name="RoverTankTeleop", group="7079")
    public static class RoverTankTeleop extends RoverTeleop {

        @Override public void init() {
            robot = new RoverRobot(new DriveTank());
            super.init();
        }
    }

    RoverRobot robot = null;
    RoverCollector roverCollector = null;
    RoverRobot.Operation operation= null;

    float clawLeft = 0;
    boolean clawArmLeft = false;
    float clawRight = 0;
    boolean clawArmRight = false;



    @Override
    public void init() {
        robot.init(hardwareMap, telemetry);
        telemetry.addData("Robot" , "Initialized");
        telemetry.update();
        roverCollector.init(hardwareMap, telemetry);
    }

    public void start() {
        // do anything we need on button start
    }


    public void stop() {
        robot.stop();
    };

    @Override
    public void loop() {
      //  doOperations();
        if (operation==null) {
            doDrive();
            doArm();
            doClaw();
        }

    }

    void doDrive() {
        double forward = FaltechUtilities.clipDeadzone(-gamepad1.right_stick_y);
        double rotate = FaltechUtilities.clipDeadzone(gamepad1.left_stick_x);
        double sideways = FaltechUtilities.clipDeadzone(-gamepad1.right_stick_x);

//        telemetry.addData("Forward Value" , forward);
//        telemetry.addData("Rotate Value" , rotate);
//        telemetry.addData("Sideways Value" , sideways);
//
//        telemetry.update();
        robot.drive.driveFRS(forward,rotate,sideways);
    }


    public void doOperations() {
        double maxTurningPower=.3;
        double maxDrivePower=.5;
        double degreesError=2.0;
        long timeoutMS=4000;

        if (operation!=null) {      // if we have an existing operation
            if (!operation.loop())  // loop the operation
                operation=null;     // and throw it away if it finished
        } else {
            if (gamepad1.x) operation = robot.getOperationRotateToHeading(45, maxTurningPower, degreesError, timeoutMS);
            if (gamepad1.y) operation = robot.getOperationRotateToHeading(-45, maxTurningPower, degreesError, timeoutMS);
            if (gamepad1.dpad_up) robot.resetRelativeAngleToZero();
        }
    }

    void doArm() {
        if (gamepad1.left_trigger > 0.2 ){

        }
    }

    public void doClaw() {
         if (!clawArmLeft && gamepad1.b){
             clawArmLeft = true;
             clawLeft = 1 - clawLeft;
             roverCollector.leftClaw.setPosition(clawLeft);
         }
         else if (!gamepad1.b){
             clawArmLeft = false;
         }

        if (!clawArmRight && gamepad1.x){
            clawArmRight = true;
            clawRight = 1 - clawRight;
            roverCollector.rightClaw.setPosition(clawRight);
        }
        else if (!gamepad1.x){
            clawArmRight = false;
        }


    }
}




