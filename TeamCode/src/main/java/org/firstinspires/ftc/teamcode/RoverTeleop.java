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
    float motorClicks = 1120;


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
            doArmExtender();
        }

    }

    void doDrive() {
        double forward = FaltechUtilities.scaleSpeedFunction(-gamepad1.left_stick_y);
        double rotate = FaltechUtilities.scaleSpeedFunction(gamepad1.right_stick_x);
        double sideways = FaltechUtilities.scaleSpeedFunction(-gamepad1.left_stick_x);

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

    double currentPosition = 0.0;



    public void doArm() {

        double armSpeed= FaltechUtilities.clipDeadzone(gamepad1.right_trigger-gamepad1.left_trigger,.2);
        double targetPosDegrees=armSpeed>=0 ? 120 : 0;
        roverCollector.setPositionDegrees(targetPosDegrees);
        roverCollector.setSpeed(armSpeed);
        telemetry.addData("Arm Speed =", armSpeed);
        telemetry.update();
    }
    public void doClaw() {
         if (!clawArmLeft && gamepad1.x){
             clawArmLeft = true;
             clawLeft = 1 - clawLeft;
             roverCollector.leftClaw.setPosition(clawLeft);
         }
         else if (!gamepad1.x){
             clawArmLeft = false;
         }

        if (!clawArmRight && gamepad1.b){
            clawArmRight = true;
            clawRight = 1 - clawRight;
            roverCollector.rightClaw.setPosition(clawRight);
        }
        else if (!gamepad1.b){
            clawArmRight = false;
        }
    }
    public void doArmExtender() {
        if (gamepad1.left_bumper) {
            roverCollector.mtrArmExtender.setPower(0.8);
        } else if (gamepad1.right_bumper) {
            roverCollector.mtrArmExtender.setPower(-0.4);
        } else {
            roverCollector.mtrArmExtender.setPower(0.0);
        }
    }
}




