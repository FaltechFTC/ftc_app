package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

public class RoverTeleop extends OpMode{
    @TeleOp(name="RoverMecanumTeleop", group="7079")
    public static class RoverMecanumTeleop extends RoverTeleop {

        @Override public void init() {
            robot = new RoverRobot(new DriveMecanum());
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

    @Override
    public void init() {
        robot.init(hardwareMap, telemetry);
        telemetry.addData("Robot" , "Initialized");
        telemetry.update();
    }

    public void start() {
        // do anything we need on button start
    };


    public void stop() {
        robot.stop();
    };

    @Override
    public void loop() {
        doOperations();
        if (operation==null) {
            doDrive();
            doArm();
        }

    }

    void doDrive() {
        double forward = FaltechUtilities.clipDeadzone(-gamepad1.right_stick_y);
        double rotate = FaltechUtilities.clipDeadzone(gamepad1.left_stick_x);
        double sideways = FaltechUtilities.clipDeadzone(-gamepad1.right_stick_x);
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
    /*Created by Error87C on 9/6/18   */
/*    if (rightTrigger > 0.2 && rightTrigger < 0.5){

        robot.mtrCollector.setPower(0.5);

    } else if (rightTrigger > 0.5){

        robot.mtrCollector.setPower(1.0);

        { else

            robot.mtrCollector.setPower(0.0);

        }
*/
    }

    public void doClaw() {
        double posclaw = 0.0;
        double posclawhinge = 0.0;

        if (gamepad1.right_trigger > 0.3) posclaw = 1.0;
        if (gamepad1.left_trigger > 0.3) posclawhinge = 1.0;

//        robot.claw.setPosition(posclaw);
//        robot.clawHinge.setPosition(posclawhinge);
    }
}
