package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

public class RoverTeleop extends OpMode{
    @TeleOp(name="RoverMecanumTeleop", group="7079")
    public static class RoverMecanumTeleop extends RoverTeleop {
        @Override public void init() {
            robot = new RoverRobot(new DriveMecanum());
            driverMode=0;
            super.init();
        }
    }

    @TeleOp(name="RoverMecanumTeleopMode1", group="7079")
    public static class RoverMecanumTeleopMode1 extends RoverTeleop {
        @Override public void init() {
            robot = new RoverRobot(new DriveMecanum());
            driverMode=1;
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
    RoverRobot.Operation operation= null;
    int driverMode =0;

    @Override
    public void init() {
        robot.init(hardwareMap, telemetry, false);
        telemetry.addData("Robot" , "Initialized");
        telemetry.update();
    }

    public void start() {
        // do anything we need on button start
    }

    public void stop() {
        robot.stop();
    }

    @Override
    public void loop() {
        doOperations();
        if (operation==null) {
            doDrive();
            doArmLift();
            doClaw();
            doArmExtender();
            doRobotLift();
            doTeamMarker();
        }
        telemetry.update();
    }

    void doDrive() {
        double leftX = FaltechUtilities.scaleSpeedFunction(gamepad1.left_stick_x);
        double leftY = FaltechUtilities.scaleSpeedFunction(-gamepad1.left_stick_y);
        double rightX = FaltechUtilities.scaleSpeedFunction(-gamepad1.right_stick_x);
        double rightY = FaltechUtilities.scaleSpeedFunction(-gamepad1.right_stick_y);

        double forward, rotate, sideways;
        if (driverMode==0) {
            forward=leftY;
            rotate=rightX;
            sideways=leftX*.5;
        } else {
            forward=rightY;
            rotate=leftX;
            sideways=rightX*.7;
        }

        double maxPower=Math.max(Math.abs(forward),Math.abs(rotate));
        maxPower=Math.max(maxPower,Math.abs(sideways));
        maxPower=Math.max(maxPower,0.7);

        telemetry.addData("Drive:" , "F="+forward+" S="+sideways+" R="+rotate);
        robot.drive.driveFRS(forward,rotate,sideways,maxPower);
    }


    double currentPosition = 0.0;

    int armLiftOpMode=0;
    double armLiftHoldPower=0;

    public void doArmLift() {
        if (gamepad1.dpad_down) armLiftOpMode=1-armLiftOpMode;
        telemetry.addData("Arm Lift Mode:" , ""+armLiftOpMode);

        if (gamepad1.dpad_left) armLiftHoldPower=Math.max(armLiftHoldPower-.1,0);
        if (gamepad1.dpad_right) armLiftHoldPower=Math.min(armLiftHoldPower+.1,.7);
        if (gamepad1.a) {
            if (armLiftHoldPower == 0 ) armLiftHoldPower = 0.15;
            else armLiftHoldPower = 0;
        }

        telemetry.addData("Arm Lift Hold Power:" , armLiftHoldPower);

        if (armLiftOpMode==0) {
            /*From Coach Ted : my concern with this approach is that it tells both motors ro race to 120 or 0
              So any variations in the load on the motors will cause one to run ahead of the other, and that
              twist will cause failure.
             */
            double armSpeed = FaltechUtilities.clipDeadzone(gamepad1.right_trigger - gamepad1.left_trigger, .2);
            double targetPosDegrees = armSpeed >= 0 ? 120 : 0;
            robot.roverCollector.setPositionDegrees(targetPosDegrees);
            armSpeed=Math.abs(armSpeed/4); // speed is absolute when using position.  scale it down
            robot.roverCollector.setSpeed(armSpeed);
            telemetry.addData("Arm Speed =", armSpeed);
        } else {
            /* From Coach Ted:  This version attempts to bump the encoder just a bit each loop
               which is like setting a short term goal on the encoder.  This should keep the motors more in sync
               as we reset the goals every loop.
             */
            double armDelta = FaltechUtilities.clipDeadzone(gamepad1.right_trigger - gamepad1.left_trigger, .2);
            double armSpeed = armDelta/4;
            double degreesToMoveEachLoop=.2;
            double targetChangeDegrees = armDelta*degreesToMoveEachLoop;
            robot.roverCollector.setPositionIncremental(targetChangeDegrees, armSpeed, armLiftHoldPower);


        }
    }

    float clawLeft = 0, clawRight = 0;
    boolean clawArmLeft = false, clawArmRight = false;

    public void doClaw() {
         if (!clawArmLeft && gamepad1.x){
             clawArmLeft = true;
             clawLeft = 1 - clawLeft;
             robot.roverCollector.leftClaw.setPosition(clawLeft);
         }
         else if (!gamepad1.x){
             clawArmLeft = false;
         }

        if (!clawArmRight && gamepad1.b){
            clawArmRight = true;
            clawRight = 1 - clawRight;
            robot.roverCollector.rightClaw.setPosition(clawRight);
        }
        else if (!gamepad1.b){
            clawArmRight = false;
        }
    }

    public void doArmExtender() {
        double armPower;
        if (gamepad1.left_bumper) armPower=.8;
        else if (gamepad1.right_bumper) armPower=-0.4;
        else armPower=0.0;
        robot.roverCollector.mtrArmExtender.setPower(armPower);
    }

    public void doRobotLift() {
        double liftPower = FaltechUtilities.clipDeadzone(gamepad2.right_trigger - gamepad2.left_trigger, .1);
        robot.roverLift.setPower(liftPower);
    }

    public void doTeamMarker(){
         if (gamepad2.y)
             robot.teamMarker.setPosition(1);
         else if (gamepad2.x)
             robot.teamMarker.setPosition(0);
    }

    public void doOperations() {
        if (operation!=null) {      // if we have an existing operation
            if (!operation.loop())  // loop the operation
                operation=null;     // and throw it away if it finished
        } else {
            double maxTurningPower=.3;
            double maxDrivePower=.5;
            double degreesError=2.0;
            long timeoutMS=4000;

            if (gamepad2.dpad_right) operation = robot.getOperationRotateToHeading(45, maxTurningPower, degreesError, timeoutMS);
            else if (gamepad2.dpad_left) operation = robot.getOperationRotateToHeading(-45, maxTurningPower, degreesError, timeoutMS);
            else if (gamepad2.dpad_up) {
                if (operation!=null) {operation.done(); operation=null;}
                robot.resetRelativeAngleToZero();
            }
        }
    }

}




