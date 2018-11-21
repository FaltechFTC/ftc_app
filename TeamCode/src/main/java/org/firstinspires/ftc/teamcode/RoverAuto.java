package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.Properties;

@Autonomous(name="RoverAuto", group="7079")
public class RoverAuto extends LinearOpMode {

    // Update from 10/13/18
    private RoverRobot robot = null;
    private RoverVision vision = null;
    RoverRobot.Operation operation= null;
    boolean isRedAlliance=true;
    boolean isEnableDepotRun=false;
    boolean isEnableCraterRun=true;
    boolean isStartFacingCrater=true;
    boolean isEnableCV=true;
    boolean isStartLatched=true;
    double maxPowerAuto = 0.5;
    double maxTurningPower = 0.3;
    double degreesError =3;
    int goldPosition = 1;
    double targetDegrees = 5;
    double targetDistance =24;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Begin Robot Initialization");
        telemetry.update();
        robot = new RoverRobot(new DriveMecanum());

        robot.init(hardwareMap, telemetry, true);

        telemetry.addData("Status", "Robot Initialized");
        telemetry.update();

    //    configValues();
        configMode();



        if (isEnableCV &&  !isStopRequested()) {
            vision=new RoverVision(hardwareMap, telemetry);
            vision.initVision();
        }

        telemetry.addData("Status", "Configured. Waiting for Start");
        telemetry.update();

        waitForStart();

        telemetry.addData("Status", "Running Autonomous!");
        telemetry.update();

    //    doTempDistance();

       if (!isStopRequested()) doLander();
       if (!isStopRequested()) doMinerals();
       if (!isStopRequested()) doDepot();
       if (!isStopRequested()) doCrater();
       robot.stop();
    }

    public void doLander() {
        // Rover Lift Code
        // TODO : Check if there is need to go up before going down, if so we need to add that code

        if (isStartLatched) {


            telemetry.addData("In the lander", "trying to lower");
            telemetry.update();

            // lift to disengage holding pin
            robot.roverLift.setPower(1.0);
            sleep(600);



            robot.roverLift.setPower(-.05);   // lower the robot to slowly to ground
            sleep(1000);
            if (isStopRequested()) return;
            robot.roverLift.setPower(-.2);   // lower more, and release arm higher to clear
            sleep(1000);
            robot.roverLift.setPower(0);  // stop lift

            if (isStopRequested()) return;

            // rotate hook out of latch
            robot.drive.driveFRS(0, -.25, 0, 0.3);
            sleep(200);
            robot.drive.stop();

            // put the lift arm away
            robot.roverLift.setPower(0.6);
            sleep(2200);
            robot.roverLift.setPower(0.0);
            if (isStopRequested()) return;

            robot.drive.driveFRS(0, .25, 0, 0.3);
            sleep(200);
            robot.drive.stop();
            if (isStopRequested()) return;

            // back up clean against lander
            robot.drive.driveFRS(-.15, 0, 0, 0.3);
            sleep(600);
            robot.drive.stop();

        }
    }

    public void doMinerals() {

        if (isEnableCV) {
            goldPosition=vision.doVision(3000, this);
            if (goldPosition==0) goldPosition=1;  // vision failed completely, default to 1
        }

        telemetry.addData("Gold Position Target", goldPosition);
        telemetry.update();

        if (goldPosition == 1) {  // LEFT of Robot   (as the robot faces forward)
            targetDegrees = -30;
            targetDistance = 18;
        } else if (goldPosition == 2) { // middle
            targetDegrees = 0;
            targetDistance = 22;
        } else {   // right of the robot
            targetDegrees = 30;
            targetDistance = 20;
        }
        robot.drive.setRunModeEncoder(false);

        operation = robot.getOperationRotateToHeading(targetDegrees, maxTurningPower, degreesError, 3000);
        operation.run();
        robot.stop();
        robot.drive.setRunModeEncoder(false);
        if (isStopRequested()){
            return;
        }
        operation = robot.getOperationDriveToHeading(5, maxPowerAuto, 0, degreesError, 10000, targetDistance);
        operation.run();
        robot.stop();
        sleep(1500);

//    public void doWallride(){
//    if (goldPosition == 1){
//    robot.driveFRS(-1,0,0,0.5);
//


    }

    public void doDepot() {
        if (!isEnableDepotRun) return;
        /*
            wallRide:
                1. Turn left to face wall
                2. Drive up to wall
                3. Turn left, and use the range sensor to keep a good distance away from the wall
                4. "Wall ride" till the depot and drop off marker
                5. Drive back till the crater and park in the crater
         */
        double turnToDepotDegrees;
        double goToDepotDistance;

        if (goldPosition == 1) {
            operation = robot.getOperationDriveToHeading(5, maxPowerAuto, 0, degreesError, 10000, 10);
            operation.run();
            turnToDepotDegrees = 40;
            goToDepotDistance = 20;
        } else if (goldPosition == 2){
            turnToDepotDegrees = 30;
            goToDepotDistance = 20;
        } else {
            turnToDepotDegrees = -50;
            goToDepotDistance = 20;
        }
        operation = robot.getOperationRotateToHeading(turnToDepotDegrees, maxTurningPower, degreesError, 3000);
        operation.run();

        operation = robot.getOperationDriveToHeading(5, maxPowerAuto, 0, degreesError, 10000, goToDepotDistance);
        operation.run();

        robot.teamMarker.setPosition(1);

        robot.drive.driveFRS(0, .25, 0, 0.3);
        sleep(200);
        robot.drive.stop();


    }

    public void doArmOverCrater() {
        robot.roverCollector.setPowerToArmExtender(0.35);
        sleep(400);
        robot.roverCollector.setPowerToArmExtender(0);
        // sleep(1000);
        robot.roverCollector.mtrArmExtender.setPower(0.9);
        sleep(2250);
        robot.roverCollector.mtrArmExtender.setPower(0);
    }

    public void doCrater() {

        if (!isEnableCraterRun) return;
        targetDistance = -10;
        if (goldPosition == 1){
            targetDegrees = -30;
            targetDistance = 20;

        } else if (goldPosition == 2){
            targetDegrees = -60;
            targetDistance = 32;
        } else {
            targetDegrees = -90;
            targetDistance = 44;
        }
        targetDistance = 15;
        //backing up 12 inches
        telemetry.addData("backUpDistance=", 12);
        telemetry.update();
        operation = robot.getOperationDriveToHeading(5, -maxPowerAuto, 0, degreesError, 10000, -12);
        operation.run();
        robot.stop();
        // turning towards wall
        operation = robot.getOperationRotateToHeading(targetDegrees, maxTurningPower, degreesError, 3000);
        operation.run();
        robot.stop();
        // driving closer to the wall
        operation = robot.getOperationDriveToDistance(5, maxPowerAuto, 0, degreesError, 10000, targetDistance);
        operation.run();
        robot.stop();
//        doArmOverCrater();


    }
    public void doTempDistance(){
        targetDistance = 15;
        operation = robot.getOperationDriveToDistance(5, maxPowerAuto, 0, degreesError, 10000, targetDistance);
        operation.run();
        robot.stop();
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
            if (FaltechUtilities.isValueChangedAndEqualTo("1.dl",gamepad1.dpad_left,true))
                isEnableDepotRun= !isEnableDepotRun;
            if (FaltechUtilities.isValueChangedAndEqualTo("1.dr",gamepad1.dpad_right,true))
                isEnableCraterRun= !isEnableCraterRun;



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
            modes+=", Depot Run="+(isEnableDepotRun?"On":"Off");
            modes+=", Crater Run="+(isEnableCraterRun?"On":"Off");

            double liftPower = FaltechUtilities.clipDeadzone( gamepad2.left_stick_y, .1);
            robot.roverLift.setPower(-liftPower);
            // joystick up is negative need to switch
            if (!modes.equals(lastModes)) {
                telemetry.addData("Alliance (X)", isRedAlliance?"Red":"Blue");
                telemetry.addData("Facing (Y)", isStartFacingCrater?"Crater":"Depot");
                telemetry.addData("Vision(A)", isEnableCV?"Enabled":"Disabled");
                telemetry.addData("Hanging(B)", isStartLatched?"Latched":"On The Ground");
                telemetry.addData("Gold Position(Dpad_Down) = ", goldPosition);
                telemetry.addData("Depot(Dpad_Left) = ", isEnableDepotRun?"On":"Off");
                telemetry.addData("Crater(Dpad_Right) = ", isEnableCraterRun?"On":"Off");
                telemetry.addData("ConfigMode" , "Press right bumper to leave config mode.");
                telemetry.update();

                RobotLog.i(modes);
                lastModes=modes;
            }

            sleep(100);
        } while (!gamepad1.right_bumper && !isStopRequested());

        telemetry.addData("ConfigMode" , lastModes);
        telemetry.update();

        RobotLog.i("configMode() stop");
    }
    void configValues(){

        FaltechUtilities.readProperties();
        isRedAlliance = FaltechUtilities.getPropBoolean("isRedAlliance", isRedAlliance);
        isEnableDepotRun = FaltechUtilities.getPropBoolean("isEnableDepotRun", isEnableDepotRun);
        isEnableCraterRun = FaltechUtilities.getPropBoolean("isEnableCraterRun", isEnableCraterRun);
        isStartFacingCrater = FaltechUtilities.getPropBoolean("isStartFacingCrater", isStartFacingCrater);
        isEnableCV = FaltechUtilities.getPropBoolean("isEnableCV", isEnableCV);
        isStartLatched = FaltechUtilities.getPropBoolean("isStartLatched", isStartLatched);
        maxPowerAuto = FaltechUtilities.getPropDouble("maxPowerAuto",maxPowerAuto);
        maxTurningPower = FaltechUtilities.getPropDouble("maxTurningPower", maxTurningPower);
        degreesError = FaltechUtilities.getPropDouble("degreesError", degreesError);
        goldPosition = FaltechUtilities.getPropInteger("goldPosition", goldPosition);
        targetDegrees = FaltechUtilities.getPropDouble("targetDegrees", targetDegrees);
        targetDistance = FaltechUtilities.getPropDouble("targetDistance", targetDistance);

    }


}