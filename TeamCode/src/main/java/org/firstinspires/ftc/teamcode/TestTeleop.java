package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


public class TestTeleop extends OpMode{
//    @TeleOp(name="RoverMecanumTeleop", group="7079")
//    public static class RoverMecanumTeleop extends RoverTeleop {
//        @Override public void init() {
//            robot = new RoverRobot(new DriveMecanum());
//            driverMode=0;
//            super.init();
//        }
//    }

    @TeleOp(name="TestTeleOp", group="7079")
    public static class RoverMecanumTeleopMode1 extends TestTeleop {
        @Override public void init() {
            robot = new RoverRobot(new DriveMecanum());
            driverMode=1;
            super.init();
        }
    }

    @TeleOp(name="RoverTankTeleop", group="7079")
    @Disabled
    public static class RoverTankTeleop extends TestTeleop {
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
        doOperations();
        if (operation==null) {
            doDrive();
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

    public void doOperations() {
        if (operation!=null) {      // if we have an existing operation
            if (!operation.loop())  // loop the operation
                operation=null;     // and throw it away if it finished
        } else {
            double maxTurningPower=.3;
            double maxDrivePower=.3;
            double degreesError=2.0;
            long timeoutMS=4000;

            if (gamepad2.dpad_right){
                robot.drive.driveFRS(0.4, 0,0,0.4);
                robot.sleep(1500);
                robot.stop();
            }
            else if (gamepad2.dpad_left) operation = robot.getOperationDriveToHeading(0,0.5,0.1,0,6000, 48);
            else if (gamepad2.dpad_up) operation = new OpWallride(robot,0, .7, .05,.3, 10000,70, 5);
            else if (gamepad2.dpad_down) operation = robot.getOperationDriveToDistance(0.4,4000,2,1);
        }
    }


}




