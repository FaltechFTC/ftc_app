package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

public abstract class RoverAuto extends LinearOpMode {

    // Update from 10/13/18
    private RoverRobot robot = null;
    private RoverVision vision = null;
    Operation operation= null;
    boolean isRedAlliance=true;
    boolean isEnableDepotRun=false;
    boolean isEnableCraterRun=true;
    boolean isStartFacingCrater=true;
    boolean isEnableCV=true;
    boolean isStartLatched=true;
    double maxPowerAuto = 0.55;
    double maxTurningPower = 0.5;
    double degreesError =2.0;
    int goldPosition = 1;
    double targetDegrees = 5;
    double targetDistance =24;
    int delayStartTime = 0;
    long rotateTimeOut = 2500;
    double maxStrafePower = 0.25;



    boolean configOnly=false;

    @Autonomous(name="RoverAuto Configure", group="7079")
    public static class RoverAutoConfig extends RoverAuto {
        public RoverAutoConfig () {
            configOnly=true;
        }
    }

    @Autonomous(name="RoverAuto Run", group="7079")
    public static class RoverAutoRun extends RoverAuto {
        public RoverAutoRun () {
            configOnly=false;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException{
        telemetry.addData("Status", "Started");
        telemetry.update();

        if (configOnly) {
            readConfigValues();
            configMode();
            writeConfigValues();
            logConfigModes(false);
            telemetry.addData("Status", "Configuration SAVED.");
            telemetry.update();
        } else {
            telemetry.addData("Status", "Initializing...");
            telemetry.update();

            initRobot(!configOnly);

            telemetry.addData("Status", "Initialized.");
            telemetry.update();

//            readConfigValues();
            configMode();
            writeConfigValues();
            logConfigModes(false);

            telemetry.addData("Status", "Configuration loaded.");
            telemetry.update();

            if (isEnableCV && !isStopRequested()) {
                vision = new RoverVision(hardwareMap, telemetry);
                vision.initVision();
            }

            telemetry.addData("Status", "Everything Ready. Waiting for Start.");
            telemetry.update();


            while (!(isStarted() || isStopRequested())) { // equivalent to waitForStart();
                telemetry.addData("Status", "waiting for start");
                telemetry.update();
                idle();
            }

            telemetry.addData("Status", "Running Autonomous!");
            telemetry.update();

            if (!isStopRequested()) doLander();
            if (!isStopRequested()) doMinerals();
            if (!isStopRequested()) doDepot();
            if (!isStopRequested()) doCrater();
        }
        robot.stop();
    }

    public void initRobot(boolean isAutonomous) {
        telemetry.addData("Status", "Begin Robot Initialization");
        telemetry.update();
        robot = new RoverRobot(new DriveMecanum());
        robot.opMode = this;
        robot.init(hardwareMap, telemetry, isAutonomous);

        telemetry.addData("Status", "Robot Initialized");
        telemetry.update();
    }

    public void doLander() {
        // Rover Lift Code

        if (isStartLatched) {


            telemetry.addData("In the lander", "trying to lower");
            telemetry.update();

            // lift to disengage holding pin
            robot.roverLift.setPower(1.0);
            sleep(750);

            robot.roverLift.setPower(-.05);   // lower the robot to slowly to ground
            sleep(1000);
            if (isStopRequested()) return;
            robot.roverLift.setPower(-.2);   // lower more, and release arm higher to clear
            sleep(1100);
            robot.roverLift.setPower(0);  // stop lift

            if (isStopRequested()) return;

            // rotate hook out of latch
            robot.drive.driveFRS(0, -.25, 0, 0.3);
            sleep(200);
            robot.drive.stop();

            // put the lift arm away
            robot.roverLift.setPower(0.9);  // need to sleep for 2200 to get arm down, but going to drive
            sleep(600);    // normally 2200 minus the time spent driving last 2 steps
            if (isStopRequested()) return;

            robot.drive.driveFRS(0, .25, 0, 0.3);
            sleep(200);
            robot.drive.stop();
            if (isStopRequested()) return;

            // back up clean against lander
            robot.drive.driveFRS(-.15, 0, 0, 0.3);
            sleep(500);
            robot.drive.stop();

            robot.roverLift.setPower(0.5);
            sleep(200);
            robot.roverLift.setPower(0.0);
        }
    }

    public void doMinerals() {



        if (isEnableCV) {
            goldPosition=vision.doVision(1800, this);
            if (goldPosition==0) goldPosition=1;  // vision failed completely, default to 3
        }

        telemetry.addData("Gold Position Target", goldPosition);
        telemetry.update();

        if (goldPosition == 1) {  // LEFT of Robot   (as the robot faces forward)
            targetDegrees = -35;
            targetDistance = 27;
        } else if (goldPosition == 2) { // middle
            targetDegrees = 0;
            targetDistance = 25;
        } else {   // right of the robot
            targetDegrees = 30;
            targetDistance = 27;
        }
     //   robot.drive.setRunModeEncoder(false);

        operation = robot.getOperationRotateToHeading(targetDegrees, maxTurningPower, degreesError, rotateTimeOut);
        operation.run();
        //robot.stop();
        //sleep(1500);
      //  robot.drive.setRunModeEncoder(false);
        if (isStopRequested()){
            return;
        }
//        operation = robot.getOperationDriveToHeading(0, maxPowerAuto, 0, degreesError, 10000, targetDistance);
//        operation.run();
//        robot.stop();
        operation = robot.getOperationDriveToDistance(maxPowerAuto,5000,targetDistance,2.0);
        operation.run();
        if (isStopRequested()){
            return;
        }
        //sleep(1500);

//    public void doWallride(){
//    if (goldPosition == 1){
//    robot.driveFRS(-1,0,0,0.5);
//


    }

    public void doDepot() {
        if (isStartFacingCrater) return;
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



        //backing up 9 inches

        operation = robot.getOperationDriveToDistance(0.5,3000,-12,2);
        operation.run();




        // turning towards wall
        targetDegrees = -robot.convertAbsoluteToRelativeAngle(90);
        operation = robot.getOperationRotateToHeading(targetDegrees, maxTurningPower, degreesError, rotateTimeOut);
        operation.run();

        if (goldPosition == 1){
            targetDistance = 42;

        } else if (goldPosition == 2){
            targetDistance = 51;

        } else {
            targetDistance = 59;

        }

        operation = robot.getOperationDriveToDistance(0.7,3000,targetDistance,2);
        operation.run();

        // turn towards wall
//        targetDegrees = -robot.convertAbsoluteToRelativeAngle(135);
//        operation = robot.getOperationRotateToHeading(targetDegrees, maxTurningPower, degreesError, rotateTimeOut);
//        operation.run();
//
//        operation = robot.getOperationDriveToDistance(0.7,3000,-70,2);
//        ((OpDriveToDistance)operation).setWallride(maxStrafePower, 6.0);
//        operation.run();
//
//        robot.teamMarker.setPosition(1);
//        robot.sleep(3000);
//
//        operation = robot.getOperationDriveToDistance(0.7,3000,72,2);
//        ((OpDriveToDistance)operation).setWallride(maxStrafePower, 6.0);
//        operation.run();



        targetDegrees = -robot.convertAbsoluteToRelativeAngle(-30);
        operation = robot.getOperationRotateToHeading(targetDegrees, maxTurningPower, degreesError, rotateTimeOut);
        operation.run();

        operation = robot.getOperationDriveToDistance(0.7,3000,55,2);
        operation.run();




        robot.teamMarker.setPosition(1);
        robot.sleep(800);

        operation = robot.getOperationDriveToDistance(0.7,3000,-40,2);
        operation.run();

        if (!isEnableCraterRun) return;


//        targetDegrees = -robot.convertAbsoluteToRelativeAngle(110);
//        operation = robot.getOperationRotateToHeading(targetDegrees, .6, degreesError, rotateTimeOut);
//        operation.run();
        robot.drive.driveFRS(0,-0.7,0);
        sleep(500);
        robot.stop();
        sleep(200);

//        targetDegrees = -robot.convertAbsoluteToRelativeAngle(130);
//        operation = robot.getOperationRotateToHeading(targetDegrees, maxTurningPower, degreesError, rotateTimeOut);
//        operation.run();

        operation = robot.getOperationDriveToDistance(0.7,3000,33,2);
        ((OpDriveToDistance)operation).setWallride(maxStrafePower, 6.0);
        operation.run();

        doArmOverCrater();
    }

    public void doArmOverCrater() {
        robot.roverCollector.setCollectorToCoast();
        robot.roverCollector.mtrArmExtender.setPower(1);
        robot.sleep(300);
        robot.roverCollector.mtrArmExtender.setPower(0);
        //Arm goes down
        robot.roverCollector.setPowerToArmLift(-0.5);
        robot.sleep(300);
        robot.roverCollector.setPowerToArmLift(0);

        //arm extends
        robot.roverCollector.mtrArmExtender.setPower(1);
        robot.sleep(300);
        robot.roverCollector.mtrArmExtender.setPower(0);

    }

    public void doCrater() {
        if (!isStartFacingCrater) return;
        if (!isEnableCraterRun) return;

        if (!isEnableDepotRun){
            if (goldPosition == 1){
                targetDegrees = 30; //20;

            } else if (goldPosition == 2){
                targetDegrees = 0;
            } else {
                targetDegrees = -30;
            }

            telemetry.addData("Desired Global Angle=", targetDegrees);
           // targetDegrees=targetDegrees+robot.getRelativeAngleFromStart();
            telemetry.addData("Desired Relative Angle=", targetDegrees);
            telemetry.update();

            operation = robot.getOperationRotateToHeading(targetDegrees, maxTurningPower, degreesError, rotateTimeOut);
            operation.coastOnStop = false;
            operation.run();

            operation = robot.getOperationDriveToDistance(maxPowerAuto,3000,10,2);
            operation.run();
            doArmOverCrater();
            return;
        }



    //backing up 9 inches

//        operation = robot.getOperationDriveToHeading(0, -maxPowerAuto, 0, degreesError, 10000, -9);
//        operation.run();
        operation = robot.getOperationDriveToDistance(0.5,3000,-10,2);
        operation.run();




        // turning towards wall
        targetDegrees = -robot.convertAbsoluteToRelativeAngle(90);
        operation = robot.getOperationRotateToHeading(targetDegrees, maxTurningPower, degreesError, rotateTimeOut);
        operation.run();


        if (goldPosition == 1){
            targetDistance = 42;

        } else if (goldPosition == 2){
            targetDistance = 49;

        } else {
            targetDistance = 57;

        }




        operation = robot.getOperationDriveToDistance(0.7,3000,targetDistance,2);
        operation.run();

        // turn towards wall
        targetDegrees = -robot.convertAbsoluteToRelativeAngle(135);
        operation = robot.getOperationRotateToHeading(targetDegrees, maxTurningPower, degreesError, rotateTimeOut);
        operation.run();

        //        // We need to do the wall run here
//        operation = robot.getOperationWallDrive(0, 0.6, .05,.15, 10000,13, 5);
//        operation.run();

         operation = robot.getOperationDriveToDistance(0.7,3000,54,2);
         ((OpDriveToDistance)operation).setWallride(maxStrafePower, 6.0);
         operation.run();


         robot.teamMarker.setPosition(1);
//
        robot.sleep(800);
//        robot.teamMarker.setPosition(0);
//        robot.sleep(50);
//        robot.teamMarker.setPosition(1);
//        robot.sleep(150);

//        operation = robot.getOperationDriveToHeading(0, -maxPowerAuto, 0, degreesError, 10000, -48);
//        operation.run();
        operation = robot.getOperationDriveToDistance(0.7,3000,-42,2);
        ((OpDriveToDistance)operation).setWallride(maxStrafePower, 6.0);
        operation.run();



       // targetDegrees = -130;        // was 150, want to turn away from wall
      //  operation = robot.getOperationRotateToHeading(targetDegrees, .6, degreesError, rotateTimeOut);
      //  operation.run();
        robot.drive.driveFRS(0,-0.7,0);
        sleep(500);
        robot.stop();
        sleep(200);

        targetDegrees = -robot.convertAbsoluteToRelativeAngle(-40);
        operation = robot.getOperationRotateToHeading(targetDegrees, maxTurningPower, degreesError, rotateTimeOut);
        operation.run();

        targetDistance = 28;
//        operation = robot.getOperationDriveToHeading(0, maxPowerAuto, 0, degreesError, 10000, targetDistance);
//        operation.run();
        operation = robot.getOperationDriveToDistance(0.7,3000,targetDistance,2);
        operation.run();

//        robot.stop();
//
        doArmOverCrater();


    }


    void configMode() {
        String lastModes="";

        RobotLog.i("configMode() start");
        telemetry.addData("ConfigMode" , "Press right bumper to leave config mode.");
        telemetry.update();
         ElapsedTime timer = new ElapsedTime();

        do {

//            if (FaltechUtilities.isValueChangedAndEqualTo("1.x",gamepad1.x,true))
//                isRedAlliance= !isRedAlliance;
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
            if (FaltechUtilities.isValueChangedAndEqualTo("1.du",gamepad1.dpad_up,true)) {
                delayStartTime++;
                if (delayStartTime > 8) delayStartTime = 0;

            }

            double liftPower = FaltechUtilities.clipDeadzone( gamepad2.left_stick_y, .1);
            if (robot!=null) robot.roverLift.setPower(-liftPower);
            logConfigModes(true);
        } while (!gamepad1.right_bumper && !isStarted() &&  !isStopRequested());

        telemetry.addData("ConfigMode" , lastModes);
        telemetry.update();

        RobotLog.i("configMode() stop");
    }

    private String lastModes="";
    void logConfigModes(boolean update) {
        String modes="";
      //  modes+="Alliance="+(isRedAlliance?"Red":"Blue");
        modes+=", Facing="+(isStartFacingCrater?"Crater":"Depot");
        modes+=", CV="+(isEnableCV?"Enabled":"Disabled");
        modes+=", Hanging="+(isStartLatched?"Latched":"On The Ground");
        modes+=", Gold = "+goldPosition;
        modes+=", Depot Run="+(isEnableDepotRun?"On":"Off");
        modes+=", Crater Run="+(isEnableCraterRun?"On":"Off");
        modes+=", Delay Time="+delayStartTime;

     //   telemetry.addData("Alliance (X)", isRedAlliance?"Red":"Blue");
        telemetry.addData("Facing (Y)", isStartFacingCrater?"Crater":"Depot");
        telemetry.addData("Vision(A)", isEnableCV?"Enabled":"Disabled");
        telemetry.addData("Hanging(B)", isStartLatched?"Latched":"On The Ground");
        telemetry.addData("Gold Position(Dpad_Down) = ", goldPosition);
        telemetry.addData("Depot(Dpad_Left) = ", isEnableDepotRun?"On":"Off");
        telemetry.addData("Crater(Dpad_Right) = ", isEnableCraterRun?"On":"Off");
        telemetry.addData("Delay(Dpad_Up)=",delayStartTime );

        if (configOnly) telemetry.addData("ConfigMode" , "Press right bumper to leave config mode.");
        if (update) telemetry.update();

        if (!modes.equals(lastModes)) {
            RobotLog.i(modes);
            lastModes=modes;
        }


    }
    void readConfigValues(){

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

    void writeConfigValues(){

        FaltechUtilities.setPropBoolean("isRedAlliance", isRedAlliance);
        FaltechUtilities.setPropBoolean("isEnableDepotRun", isEnableDepotRun);
        FaltechUtilities.setPropBoolean("isEnableCraterRun", isEnableCraterRun);
        FaltechUtilities.setPropBoolean("isStartFacingCrater", isStartFacingCrater);
        FaltechUtilities.setPropBoolean("isEnableCV", isEnableCV);
        FaltechUtilities.setPropBoolean("isStartLatched", isStartLatched);
        FaltechUtilities.setPropDouble("maxPowerAuto",maxPowerAuto);
        FaltechUtilities.setPropDouble("maxTurningPower", maxTurningPower);
        FaltechUtilities.setPropDouble("degreesError", degreesError);
        FaltechUtilities.setPropInteger("goldPosition", goldPosition);
        FaltechUtilities.setPropDouble("targetDegrees", targetDegrees);
        FaltechUtilities.setPropDouble("targetDistance", targetDistance);
        FaltechUtilities.writeProperties();

    }


}