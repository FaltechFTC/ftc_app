package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


public class RoverTeleop extends OpMode{

    @TeleOp(name="Rover TeleOp", group="7079")
    public static class RoverMecanumTeleop extends RoverTeleop {
        @Override public void init() {
            robot = new RoverRobot(new DriveMecanum());
            super.init();
        }
    }

    @TeleOp(name="Rover OperationsOnly Tele", group="7079")
    public static class RoverMecanumOpsOnly extends RoverTeleop {
        @Override public void init() {
            robot = new RoverRobot(new DriveMecanum());
            super.init();
            operationsOnly=true;
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
    double turnDegrees=0;
    boolean operationsOnly=false;
    ElapsedTime timer = new ElapsedTime();
    double lastOperationTime = 0;
    @Override
    public void init() {
        robot.init(hardwareMap, telemetry, true);
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
        if (operationsOnly || operation!=null)
            doOperations();
        else if (operation==null) {
            doDrive();
            doArmLift();
            doClaw();
            doArmExtender();
            doRobotLift();
            doTeamMarker();
            doGears();
            robot.logSensors();
        }
        doTestAngles();
        telemetry.update();
    }

    void doTestAngles(){
        telemetry.addData("Absolute Angle", robot.getCurrentAbsoluteAngle());
    }

    void doDrive() {
        double leftX = FaltechUtilities.scaleSpeedFunction(gamepad1.left_stick_x);
        double leftY = FaltechUtilities.scaleSpeedFunction(-gamepad1.left_stick_y);
        double rightX = FaltechUtilities.scaleSpeedFunction(-gamepad1.right_stick_x);
        double rightY = FaltechUtilities.scaleSpeedFunction(-gamepad1.right_stick_y);

        double forward, rotate, sideways;
        forward=rightY*robot.drive.getForwardGearMultiplier();;
        rotate=leftX*robot.drive.getRotateGearMultiplier();
        sideways=rightX*robot.drive.getSidewaysGearMultiplier();

        telemetry.addData("Drive:" , "F="+forward+" S="+sideways+" R="+rotate);
        robot.drive.driveFRS(forward,rotate,sideways);
    }


    double currentPosition = 0.0;

    int armLiftOpMode=0;
    double armLiftHoldPower=0;

    public void doArmLift() {

        double armSpeed = FaltechUtilities.clipDeadzone((gamepad1.right_trigger - gamepad1.left_trigger),.10);
        if (armSpeed==0)
            armSpeed=FaltechUtilities.clipDeadzone((gamepad2.right_trigger - gamepad2.left_trigger), .10);
        if( gamepad2.dpad_down) robot.roverCollector.lowerArmLiftReset(-0.35, 650);
        else robot.roverCollector.setPowerToArmExtender(armSpeed/1.5);
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

        if (gamepad1.right_bumper || gamepad2.right_bumper) armPower = 1.0;
        else if (gamepad1.left_bumper || gamepad2.left_bumper) armPower = -1.0;
        else armPower = 0.0;

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

    double opRotateTestMaxPower=.15;
    double opRotateTestDegreesError=2.0;

    public void doOperations() {
        if (operation!=null) {      // if we have an existing operation
            if (!operation.loop())  // loop the operation
            {operation=null;
              lastOperationTime = timer.seconds();
            }    // and throw it away if it finished
        } else {

            if ((gamepad2.right_bumper && gamepad2.left_bumper) || (gamepad1.right_bumper && gamepad1.left_bumper)) {
                if (gamepad1.a) opRotateTestMaxPower += .01;
                if (gamepad1.b) opRotateTestMaxPower -= .01;
                if (gamepad1.x) opRotateTestDegreesError += .2;
                if (gamepad1.y) opRotateTestDegreesError -= .2;

                if (gamepad2.a) OpRotateToHeading.drivePidKp += .01;
                if (gamepad2.b) OpRotateToHeading.drivePidKp -= .01;
                if (gamepad2.x) OpRotateToHeading.drivePidTd += .01;
                if (gamepad2.y) OpRotateToHeading.drivePidTd -= .01;
                if (gamepad2.dpad_right) OpRotateToHeading.drivePidTi += .01;
                if (gamepad2.dpad_left) OpRotateToHeading.drivePidTi -= .01;
                if (gamepad2.dpad_up) OpRotateToHeading.drivePidIntMax += 1;
                if (gamepad2.dpad_down) OpRotateToHeading.drivePidIntMax -= 1;

                telemetry.addData("RotatePID", String.format("maxPower=%3.2f degErr=%3.2f p=%3.2f i=%3.2f d=%3.2f maxInt=%3.2f",
                        opRotateTestMaxPower, opRotateTestDegreesError,
                        OpRotateToHeading.drivePidKp, OpRotateToHeading.drivePidTi, OpRotateToHeading.drivePidTd,
                        OpRotateToHeading.drivePidIntMax));
                robot.sleep(300);
                return;
            }


            double maxTurningPower = .3;
            double maxDrivePower = .5;
            double degreesError = 1.0;
            long timeoutMS = 4000;

//            if (gamepad2.dpad_right) operation = robot.getOperationDriveToDistance(0.4,5000,48,2);
//            else if (gamepad2.dpad_left) operation = robot.getOperationDriveToDistance(0.3,5000,-9,0.5);
//            else if (gamepad2.dpad_up) operation = robot.getOperationDriveToDistance(0.2,5000,48,0.5);
//            else if (gamepad2.dpad_down) operation = new OpWallride(robot,0, .3, .05,0, 10000,14, 5);

            if (gamepad2.dpad_right) {
                turnDegrees = robot.mapDegreesTo180( -90);
                operation = robot.getOperationRotateToHeading(-robot.convertAbsoluteToRelativeAngle(turnDegrees), opRotateTestMaxPower, opRotateTestDegreesError, 9000);
                timer.reset();
                lastOperationTime = 0;

            } else if (gamepad2.dpad_left) {
                turnDegrees = robot.mapDegreesTo180(0);
                operation = robot.getOperationRotateToHeading(-robot.convertAbsoluteToRelativeAngle(turnDegrees),opRotateTestMaxPower,opRotateTestDegreesError,9000);
                timer.reset();
                lastOperationTime = 0;
            } else if (gamepad2.dpad_up) operation = robot.getOperationDriveToDistance(0.2,5000,48,0.5);
            else if (gamepad2.dpad_down) operation = new OpWallride(robot,0, .3, .05,0, 10000,14, 5);

            telemetry.addData("Turn Test", "actual = "+ robot.getCurrentAbsoluteAngle() +" target="+turnDegrees+" diffDegrees="+Math.abs(robot.getCurrentAbsoluteAngle()-turnDegrees) );
            telemetry.addData("Timer =", lastOperationTime);
        }
    }

}




