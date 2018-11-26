package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


public class RoverTeleop extends OpMode{
//    @TeleOp(name="RoverMecanumTeleop", group="7079")
//    public static class RoverMecanumTeleop extends RoverTeleop {
//        @Override public void init() {
//            robot = new RoverRobot(new DriveMecanum());
//            driverMode=0;
//            super.init();
//        }
//    }

    @TeleOp(name="RoverMecanumTeleOp", group="7079")
    public static class RoverMecanumTeleopMode1 extends RoverTeleop {
        @Override public void init() {
            robot = new RoverRobot(new DriveMecanum());
            driverMode=1;
            super.init();
        }
    }

    @TeleOp(name="RoverTankTeleop", group="7079")
    @Disabled
    public static class RoverTankTeleop extends RoverTeleop {
        @Override public void init() {
            robot = new RoverRobot(new DriveTank());
            super.init();
        }
    }

    RoverRobot robot = null;
    Operation operation= null;
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
            doGears();
            robot.logSensors();
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

        rotate*=robot.drive.getRotateGearMultiplier();
        forward*=robot.drive.getForwardGearMultiplier();
        sideways*=robot.drive.getSidewaysGearMultiplier();

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

        double armSpeed = FaltechUtilities.clipDeadzone((gamepad1.right_trigger - gamepad1.left_trigger),.15);
        if (armSpeed==0)
            armSpeed=FaltechUtilities.clipDeadzone((gamepad2.right_trigger - gamepad2.left_trigger), .15);
        robot.roverCollector.setPowerToArmExtender(armSpeed/3);
       // telemetry.addData("ArmLift Speed =", armSpeed);
    }


    double clawLeft=0.0;
    double clawRight = 0.0;
    public void doClaw() {


        if (FaltechUtilities.isValueChangedAndEqualTo("1.a", gamepad1.a, true)
                || FaltechUtilities.isValueChangedAndEqualTo("2.a", gamepad2.a, true)
                ) {
            clawLeft = 1 - clawLeft;
            robot.roverCollector.leftClaw.setPosition(clawLeft);
        }
        if (FaltechUtilities.isValueChangedAndEqualTo("1.x", gamepad1.b, true)
                || FaltechUtilities.isValueChangedAndEqualTo("2.x", gamepad2.b, true)
                ) {
            clawRight = 1 - clawRight;
            robot.roverCollector.rightClaw.setPosition(clawRight);


        }
    }
    public void doArmExtender() {
        double armPower;
        if (gamepad1.right_bumper || gamepad2.right_bumper) armPower=1.0;
        else if (gamepad1.left_bumper || gamepad2.left_bumper) armPower=-1.0;
        else armPower=0.0;
        robot.roverCollector.mtrArmExtender.setPower(armPower);
    }

    public void doRobotLift() {
        double liftPower = FaltechUtilities.clipDeadzone(gamepad2.left_stick_y, .1);
        robot.roverLift.setPower(-liftPower);
    }

    public void doTeamMarker(){
         if (gamepad2.y)
             robot.teamMarker.setPosition(1);
         else if (gamepad2.x)
             robot.teamMarker.setPosition(0);
    }
    public void doGears(){
        if (FaltechUtilities.isValueChangedAndEqualTo("1.dpd", gamepad1.dpad_down, true))
            robot.drive.addGearValue(-1);
        if (FaltechUtilities.isValueChangedAndEqualTo("1.dpu", gamepad1.dpad_up, true))
            robot.drive.addGearValue(1);
        telemetry.addData("Gear", robot.drive.gearValue);
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




