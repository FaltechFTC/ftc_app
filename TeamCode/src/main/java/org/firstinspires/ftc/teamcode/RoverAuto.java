package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/* all TensorFlow originally from examples provided by First from class ConceptTensorFlowObjectDetection.java */

@Autonomous(name="RoverAuto", group="7079")
public class RoverAuto extends LinearOpMode {

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    // Update from 10/13/18
    private RoverRobot robot = null;
    RoverRobot.Operation operation= null;
    boolean isRedAlliance=false;
    boolean isEnableDepotRun=false;
    boolean isEnableCraterRun=true;
    boolean isStartFacingCrater=false;
    boolean isEnableCV=false;
    boolean isStartLatched=false;
    double maxPowerAuto = 0.5;
    double maxTurningPower = 0.3;
    double degreesError =3;
    int goldPosition = 1;
    double goldDegrees = 5;
    double knockOffDistance =24;
    private ElapsedTime runTime = new ElapsedTime();


    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia=null;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod=null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Begin Robot Initialization");
        telemetry.update();
        robot = new RoverRobot(new DriveMecanum());


        robot.init(hardwareMap, telemetry, true);

        telemetry.addData("Status", "Robot Initialized");
        telemetry.update();

        configMode();

        if (isEnableCV) initVision();

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
            // lower the robotc
            robot.roverLift.setTargetPosition(4);
            robot.roverLift.setPower(0.4);

            // Strafe right for 300 milliseconds to unlatch
            robot.drive.driveFRS(0, 0, 1, maxPowerAuto);
            sleep(300);

            // tuck the lift arm down out of the way
            robot.roverLift.setTargetPosition(-4);
            robot.roverLift.setPower(0.4);

            sleep(2000);  // wait for it to tuck down out of the way.

            // don't use any more power on the lift.
            robot.roverLift.setPower(0.0);
        }


        // TODO: We need to check for which spot is the mineral in

    }

    public void doMinerals() {

        if (isEnableCV) doVision(3000);

        if (goldPosition == 1) {  // LEFT of Robot   (as the robot faces forward)
            goldDegrees = -30;
            knockOffDistance = 16;
        } else if (goldPosition == 2) { // middle
            goldDegrees = 0;
            knockOffDistance = 14;
        } else {   // right of the robot
            goldDegrees = 30;
            knockOffDistance = 16;
        }
        robot.drive.setRunModeEncoder(false);

        operation = robot.getOperationRotateToHeading(goldDegrees, maxTurningPower, degreesError, 3000);
        operation.run();
        robot.stop();
        robot.drive.setRunModeEncoder(false);

        operation = robot.getOperationDriveToHeading(5, maxPowerAuto, 0, degreesError, 10000, knockOffDistance);
        operation.run();
        robot.stop();
        sleep(1500);
    }

    public void doDepot() {
        if (!isEnableDepotRun) return;
        /*
            wallRide{
    1. Turn left to face wall
    2. Drive up to wall
    3. Turn left, and use the range sensor to keep a good distance away from the wall
    4. "Wall ride" till the depot and drop off marker
    5. Drive back till the crater and park in the crater


         */
    }

    public void doCrater() {
        if (!isEnableCraterRun) return;

        if (!isEnableDepotRun) {
            if (isStartFacingCrater) {
                // Crater auto
                operation = robot.getOperationRotateToHeading(-goldDegrees, maxTurningPower, degreesError, 3000);
                operation.run();
// rotate in the opposite direction of the 1, 2, 3 turn.

                operation = robot.getOperationDriveToHeading(5, maxPowerAuto, 0, degreesError, 10000, 6);
                operation.run();
// move forwards 6 degrees to the crater
            }
            else {
                // Depot Auto
                operation = robot.getOperationDriveToHeading(5, maxPowerAuto, 0, degreesError, 10000, -10);
                operation.run();
                sleep(1500);
// Back up
                operation = robot.getOperationRotateToHeading(-goldDegrees, maxTurningPower, degreesError, 3000);
                operation.run();
                sleep(1500);
//Rotate so in original lander pos
                operation = robot.getOperationRotateToHeading(-120, maxTurningPower, degreesError, 3000);
                operation.run();
                sleep(1500);
// turn 90 degrees, so will be facing to left of crater.
                operation = robot.getOperationDriveToHeading(5, maxPowerAuto, 0, degreesError, 10000, 12);
                operation.run();
                sleep(1500);
//Move forwards to right next to crater
                operation = robot.getOperationRotateToHeading(40, maxTurningPower, degreesError, 3000);
                operation.run();
                sleep(1500);
// rotate so facing crater
                operation = robot.getOperationDriveToHeading(-5, maxPowerAuto, 0, degreesError, 10000, 6);
                operation.run();
                sleep(1500);
                // move forwards to crater
            }
            robot.roverCollector.setPowerToArmExtender(0.4);
            sleep(1000);
            robot.roverCollector.setPowerToArmExtender(0);
            // no matter which route, end with extending arm at crater.
        }
    }

    public void doVision(int maxMilliseconds) {
        /** Activate Tensor Flow Object Detection. */
        if (tfod == null)  return;

        ElapsedTime runTime = new ElapsedTime();
        runTime.reset();

        int goldMineralX = -1;
        int silverMineral1X = -1;
        int silverMineral2X = -1;

        while (opModeIsActive() && runTime.milliseconds()<maxMilliseconds) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions == null) continue;

            telemetry.addData("# Object Detected", updatedRecognitions.size());
            for (Recognition recognition : updatedRecognitions) {
                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                    goldMineralX = (int) recognition.getLeft();
                    telemetry.addData("Gold", goldMineralX);
                } else if (silverMineral1X == -1) {
                    silverMineral1X = (int) recognition.getLeft();
                    telemetry.addData("Silver1", silverMineral1X);
                } else {
                    silverMineral2X = (int) recognition.getLeft();
                    telemetry.addData("Silver2", silverMineral1X);
                }
            }
            telemetry.update();
            sleep(400);

        }

        tfod.shutdown();

        if (goldMineralX < silverMineral1X || goldMineralX < silverMineral2X) goldPosition=1;
        else if (goldMineralX > silverMineral1X )  goldPosition=2;
        else  goldPosition=3;

        telemetry.addData("Gold Mineral Position", goldPosition);
        telemetry.update();
        sleep(2000);

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


    public void initVision() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }


    }
    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = FaltechUtilities.VUFORIA_KEY;
        boolean useBackCamera=true;
        if (useBackCamera)  {
            parameters.cameraDirection = CameraDirection.BACK;
        } else /*front camera*/ {
            /* lifted this from another example, to go off of the front camera ... or should it just be CameraDirection.FRONT */
            parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        }
        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
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
        } while (!gamepad1.right_bumper);

        telemetry.addData("ConfigMode" , lastModes);
        telemetry.update();

        RobotLog.i("configMode() stop");
    }

}