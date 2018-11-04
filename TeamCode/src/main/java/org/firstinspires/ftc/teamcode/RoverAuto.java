package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

@Autonomous(name="RoverAuto", group="7079")
public class RoverAuto extends LinearOpMode {
    /*
    1. Lower from lander
    2. Detach from lander
    3. Turn to see minerals
    4. Use openCV to knock off the gold mineral
    5. Turn to face direction of wall
    6. Drive to wall
    7. Turn at wall
    8. "Wall ride" using range sensor to go to the depot
    9. Drop off marker
    10. Turn around
    11. Go back to the crater for parking

    public void lowerRobot{
      1.  Measure and bring up the motor using the encoders to land
      2. Strafe right to move out hook
    }

    public void detectMinerals{
    1. Turn robot to see minerals
    2. Identify position
    3. Drive to position
    4. Knock off mineral
    5. Back up 18 inches
    }

    public void wallRide{
    1. Turn left to face wall
    2. Drive up to wall
    3. Turn left, and use the range sensor to keep a good distance away from the wall
    4. "Wall ride" till the depot and drop off marker
    5. Drive back till the crater and park in the crater

    }
     */

    // Update from 10/13/18
    private RoverRobot robot = null;
    RoverRobot.Operation operation= null;
    boolean isRedAlliance=false;
    boolean isStartFacingCrater=true;
    boolean isEnableCV=false;
    boolean isStartLatched=false;
    double maxPowerAuto = 0.5;
    double maxTurningPower = 0.1;
    double degreesError =3;
    int goldPosition = 1;
    double goldDegrees = 5;
    double knockOffDistance =24;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Begin Robot Initialization");
        telemetry.update();
        robot = new RoverRobot(new DriveMecanum());


        robot.init(hardwareMap, telemetry, true);

        telemetry.addData("Status", "Robot Initialized");
        telemetry.update();

        configMode();

        telemetry.addData("Status", "Configured. Waiting for Start");
        telemetry.update();

        waitForStart();

        telemetry.addData("Status", "Running Autonomous!");
        telemetry.update();


        doLander();
        doMinerals();
        doDepot();
        doCrater();
    }

    public void doLander() {
        // Rover Lift Code
        // TODO : Check if there is need to go up before going down, if so we need to add that code
        if (isStartLatched) {
            robot.roverLift.setTargetPosition(4);
            robot.roverLift.setPower(0.4);

            // Strafe right for 300 milliseconds

            robot.drive.driveFRS(0, 0, 1, maxPowerAuto);
            sleep(300);
        }
        // TODO: We need to check for which spot is the mineral in

    }

    public void doMinerals() {
        if (goldPosition == 1) {
            goldDegrees = 30;
            knockOffDistance = 28;
        } else if (goldPosition == 2) {
            goldDegrees = 5;
            knockOffDistance = 60;
        } else {
            goldDegrees = -30;
            knockOffDistance = 28;
        }
        robot.drive.setRunModeEncoder(false);
        operation = robot.getOperationDriveToHeading(goldDegrees, maxPowerAuto, maxTurningPower, degreesError, 10000, knockOffDistance);
        operation.run();
        robot.stop();
    }

    public void doDepot() {

    }

    public void doCrater() {

    }

    public void doParkingLot() {

//        // Need to drive back 12 inches to start the turn
//
//        robot.drive.driveToInches(-12,0.3,5);
//        robot.drive.setRunModeEncoder(false);
//
//        // Turn 50 degrees left
//        operation = robot.getOperationRotateToHeading(-50, maxTurningPower, degreesError, 3000);
//        operation.run();
//        robot.stop();
//
//        robot.drive.driveToInches(15,0.3,5);
//        robot.drive.setRunModeEncoder(false);
//
//        // Turn 50 degrees left
//        operation = robot.getOperationRotateToHeading(-50, maxTurningPower, degreesError, 3000);
//        operation.run();
//        robot.stop();
//
//        // TODO : Drive up to the Team Depot, need to measure the length and use driveInches or driveToHeading function
//        // TODO : Drop the team Marker once we reach the Team Depot
//        // TODO : Drive back to the crater
//        // TODO : Extend the arm based on time so that it crosses the crater
//

//
//
        telemetry.addData("About to execute drive ","inches");
        telemetry.update();
        // TODO: Write the code for robot drop and back up to lander

//        robot.drive.driveToInches(12,0.3,5);
//        robot.drive.setRunModeEncoder(false);
//
//        telemetry.addData("About to execute rotate "," to 75 left");
//        telemetry.update();
//
//        operation = robot.getOperationRotateToHeading(-60, maxTurningPower, degreesError, timeoutMS);
//        operation.run();
//        robot.stop();
//
//        robot.drive.driveToInches(-12,0.3,5);
//        robot.drive.setRunModeEncoder(false);
//
//        operation = robot.getOperationRotateToHeading(-60, maxTurningPower, degreesError, timeoutMS);
//        operation.run();
//        robot.drive.driveFRS(0,0,1,0.3);
//        sleep(500);
//        robot.stop();
//        robot.drive.setRunModeEncoder(false);
//        operation = robot.getOperationDriveToHeading(5,0.3,0.2,5,10000,15);
//        operation.run();
//        robot.stop();
//
//        operation = robot.getOperationDriveToHeading(5,0.3,0,5,10000,-15);
//        operation.run();
//        robot.stop();

    }

    void configMode() {
        String lastModes="";

        RobotLog.i("configMode() start");
        telemetry.addData("ConfigMode" , "Press right bumper to leave config mode.");
        telemetry.update();

        do {

            if (FaltechUtilities.isValueChangedAndEqualTo("1.x",gamepad1.x,true))
                isRedAlliance= !isRedAlliance;
            if (FaltechUtilities.isValueChangedAndEqualTo("1.y",gamepad1.y,true))
                isStartFacingCrater= !isStartFacingCrater;
            if (FaltechUtilities.isValueChangedAndEqualTo("1.a",gamepad1.a,true))
                isEnableCV= !isEnableCV;
            if (FaltechUtilities.isValueChangedAndEqualTo("1.b",gamepad1.b,true))
                isStartLatched= !isStartLatched;


            if (FaltechUtilities.isValueChangedAndEqualTo("1.dd",gamepad1.dpad_down,true)) {
                goldPosition++;
                if (goldPosition > 3) goldPosition = 1;

            }

/*            // just a test...  to be reworked...  can we do this at start?
            double liftPower=FaltechUtilities.clipDeadzone(gamepad1.right_trigger-gamepad1.left_trigger,.2);
            if (liftPower>0) robot.roverLift.mtrRoverLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // TODO: remove this...
            robot.roverLift.setPower(liftPower);
*/

            String modes="";
            modes+="Alliance="+(isRedAlliance?"Red":"Blue");
            modes+=", Facing="+(isStartFacingCrater?"Crater":"Depot");
            modes+=", CV="+(isEnableCV?"Enabled":"Disabled");
            modes+=", Hanging="+(isStartLatched?"Latched":"On The Ground");
            modes+=", Gold = "+goldPosition;

            if (!modes.equals(lastModes)) {
                telemetry.addData("Alliance", isRedAlliance?"Red":"Blue");
                telemetry.addData("Facing", isStartFacingCrater?"Crater":"Depot");
                telemetry.addData("CV", isEnableCV?"Enabled":"Disabled");
                telemetry.addData("Hanging", isStartLatched?"Latched":"On The Ground");
                telemetry.addData("Gold Position = ", goldPosition);
                telemetry.addData("ConfigMode" , "Press right bumper to leave config mode.");
                telemetry.update();

                RobotLog.i(modes);
                lastModes=modes;
            }

            sleep(100);
        } while (!gamepad1.right_bumper);

        telemetry.addData("ConfigMode" , lastModes);
        telemetry.update();

        RobotLog.i("configMode() stop");
    }

}