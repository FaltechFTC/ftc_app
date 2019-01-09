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
    Operation operation= null, lastOperation= null;

    double turnDegrees=0;
    boolean operationsOnly=false;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        telemetry.addData("Robot" , "Initializing...");
        telemetry.update();

        robot.init(hardwareMap, telemetry, true);

        telemetry.addData("Robot" , "Initialized.  Ready to start.");
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
        telemetry.addData("Robot" , "Running Teleop.");
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
        telemetry.addData("Robot Angle", robot.getCurrentAbsoluteAngle());
        telemetry.update();
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


    int opTestMode=0;
    public void doOperations() {
        if (operation!=null) {      // if we have an existing operation
            boolean keepLooping=operation.loop();  // loop the operation
            if (!keepLooping)
            {
              lastOperation=operation; // store it for reporting
              operation=null; // operation is done
            }
        } else {
            if (gamepad2.start && gamepad2.left_bumper) {
                opTestMode++;
                if (opTestMode<0 || opTestMode>3) opTestMode=0;
                robot.sleep(300);
            }
            if (opTestMode==1) doOpTurnTesting();
            else if (opTestMode==2) doOpDistanceTesting();
            else if (opTestMode==3) doOpWallrideTesting();

            if (lastOperation!=null)
                telemetry.addData("Last Operation", lastOperation.getResult());
        }
    }

    double opRotateTestMaxPower=.45;
    double opRotateTestDegreesError=1.5;
    public void doOpTurnTesting() {
        telemetry.addData("TEST MODE", "Op Turn Testing");

        Pid pidPrototype=OpRotateToHeading.pidRotatePrototype;
        if ((gamepad2.right_bumper && gamepad2.left_bumper) || (gamepad1.right_bumper && gamepad1.left_bumper)) {
            if (gamepad1.a) opRotateTestMaxPower += .01;
            if (gamepad1.b) opRotateTestMaxPower -= .01;
            if (gamepad1.x) opRotateTestDegreesError += .2;
            if (gamepad1.y) opRotateTestDegreesError -= .2;

            if (gamepad2.a) pidPrototype.kp += .01;
            if (gamepad2.b) pidPrototype.kp -= .002;
            if (gamepad2.x) pidPrototype.kd += .01;
            if (gamepad2.y) pidPrototype.kd -= .01;
            if (gamepad2.dpad_right) pidPrototype.ki += .001;
            if (gamepad2.dpad_left) pidPrototype.ki -= .001;
            if (gamepad2.dpad_up) { pidPrototype.integralMax += 1;  pidPrototype.integralMin= -pidPrototype.integralMax; }
            if (gamepad2.dpad_down) { pidPrototype.integralMax -= 1; pidPrototype.integralMin= -pidPrototype.integralMax;}

            telemetry.addData("RotatePID", "maxPower=%3.2f degErr=%3.2f p=%4.3f i=%4.3f d=%3.2f maxInt=%3.2f",
                    opRotateTestMaxPower, opRotateTestDegreesError,
                    pidPrototype.kp, pidPrototype.ki, pidPrototype.kd, pidPrototype.integralMax);
            robot.sleep(300);
            return;
        }

        long timeoutMS = 9000;

        if (gamepad2.dpad_right) {
            turnDegrees = robot.mapDegreesTo180( -120);
            operation = robot.getOperationRotateToHeading(-robot.convertAbsoluteToRelativeAngle(turnDegrees), opRotateTestMaxPower, opRotateTestDegreesError, timeoutMS);
        } else if (gamepad2.dpad_left) {
            turnDegrees = robot.mapDegreesTo180(0);
            operation = robot.getOperationRotateToHeading(-robot.convertAbsoluteToRelativeAngle(turnDegrees),opRotateTestMaxPower,opRotateTestDegreesError,timeoutMS);
        }else if (gamepad2.dpad_down) {
            turnDegrees = robot.mapDegreesTo180(30);
            operation = robot.getOperationRotateToHeading(-robot.convertAbsoluteToRelativeAngle(turnDegrees),opRotateTestMaxPower,opRotateTestDegreesError,timeoutMS);
        }else if (gamepad2.dpad_down) {
            turnDegrees = robot.mapDegreesTo180(170);
            operation = robot.getOperationRotateToHeading(-robot.convertAbsoluteToRelativeAngle(turnDegrees),opRotateTestMaxPower,opRotateTestDegreesError,timeoutMS);
        }
    }

    double opDriveTestMaxPower=.5;
    double opDriveTestError=1;
    public void doOpDistanceTesting() {
        telemetry.addData("TEST MODE", "Op Distance Testing");

        long timeoutMS = 9000;

        Pid pidPrototype=OpDriveToDistance.pidDrivePrototype;
        if ((gamepad2.right_bumper && gamepad2.left_bumper) || (gamepad1.right_bumper && gamepad1.left_bumper)) {
            if (gamepad1.a) opDriveTestMaxPower += .01;
            if (gamepad1.b) opDriveTestMaxPower -= .01;
            if (gamepad1.x) opDriveTestError += .1;
            if (gamepad1.y) opDriveTestError -= .1;

            if (gamepad2.a) pidPrototype.kp += .002;
            if (gamepad2.b) pidPrototype.kp -= .002;
            if (gamepad2.x) pidPrototype.kd += .01;
            if (gamepad2.y) pidPrototype.kd -= .01;
            if (gamepad2.dpad_right) pidPrototype.ki += .001;
            if (gamepad2.dpad_left) pidPrototype.ki -= .001;
            if (gamepad2.dpad_up) { pidPrototype.integralMax += 1;  pidPrototype.integralMin= -pidPrototype.integralMax; }
            if (gamepad2.dpad_down) { pidPrototype.integralMax -= 1; pidPrototype.integralMin= -pidPrototype.integralMax;}

            telemetry.addData("DrivePID", "maxPower=%3.2f degErr=%3.2f p=%4.3f i=%4.3f d=%3.2f maxInt=%3.2f",
                    opRotateTestMaxPower, opRotateTestDegreesError,
                    pidPrototype.kp, pidPrototype.ki, pidPrototype.kd, pidPrototype.integralMax);
            robot.sleep(300);
            return;
        }


        if (gamepad2.dpad_right)        operation = robot.getOperationDriveToDistance(opDriveTestMaxPower,timeoutMS,12,opDriveTestError);
        else if (gamepad2.dpad_left)    operation = robot.getOperationDriveToDistance(opDriveTestMaxPower,timeoutMS,-12,opDriveTestError);
        else if (gamepad2.dpad_up)      operation = robot.getOperationDriveToDistance(opDriveTestMaxPower,timeoutMS,24,opDriveTestError);
        else if (gamepad2.dpad_down)    operation = robot.getOperationDriveToDistance(opDriveTestMaxPower,timeoutMS,-24,opDriveTestError);

    }

    public void doOpWallrideTesting() {
        telemetry.addData("TEST MODE", "Op Wallride Testing");

        long timeoutMS = 9000;
        double maxDrivePower=.5;
        double maxStrafePower=.15;
        double maxTurnPower=.05;

        if (gamepad2.dpad_right)
            operation = robot.getOperationWallDrive(0, maxDrivePower, maxTurnPower,maxStrafePower, timeoutMS,13, 5);
        else if (gamepad2.dpad_up) {
            operation = robot.getOperationDriveToDistance(maxDrivePower, timeoutMS, 48, 2);
            ((OpDriveToDistance)operation).setWallride(maxStrafePower, 5.0);
        } else if (gamepad2.dpad_down) {
            operation = robot.getOperationDriveToDistance(maxDrivePower, timeoutMS, -48, 2);
            ((OpDriveToDistance)operation).setWallride(maxStrafePower, 10.0);
        }

    }


}




