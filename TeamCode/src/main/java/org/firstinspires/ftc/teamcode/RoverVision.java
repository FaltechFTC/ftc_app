package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.util.List;

public class RoverVision {

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private ElapsedTime runTime = new ElapsedTime();
    private VuforiaLocalizer vuforia=null;  // instance of Vuforia localization engine
    private TFObjectDetector tfod=null;  // Tensor Flow Object Dectection engine
    Telemetry telemetry=null;
    HardwareMap hardwareMap=null;

    public RoverVision (HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap=hardwareMap;
        this.telemetry=telemetry;
    }

    public int doVision(int maxMilliseconds, LinearOpMode opMode) {
        int goldPosition= 0;

        /** Activate Tensor Flow Object Detection. */
        if (tfod == null)  return goldPosition;

        tfod.activate();

        ElapsedTime runTime = new ElapsedTime();
        runTime.reset();

        Recognition bestGold=null; // use this to track the best gold that we find.

        while ((opMode==null || opMode.opModeIsActive()) && runTime.milliseconds()<maxMilliseconds) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            int size=0;
            if (updatedRecognitions==null) {
                ; // DO NOTHING
            } else {
                size=updatedRecognitions.size();
                int objNum=0;
                // loop through all of the found objects
                for (Recognition obj : updatedRecognitions) {

                    // for debugging, output all that we find.
                    telemetry.addData("Vision Obj#"+ objNum++, "label="+obj.getLabel()+
                            " left="+obj.getLeft()+
                            " right="+obj.getRight()+
                            " top="+obj.getTop()+
                            " confidence="+obj.getConfidence());

                    // if we found gold
                    if (obj.getLabel().equals(LABEL_GOLD_MINERAL)) {
                        // if it isn't too high (in crater) and if it has good confidence
                        if (obj.getTop() < 600 && obj.getConfidence() > 50) {
                            // track this one as the best, if we don't have one yet
                            if (bestGold == null)
                                bestGold = obj;
                                // or if the one just found is better than the one we found before
                            else if (obj.getConfidence() > bestGold.getConfidence())
                                bestGold = obj;
                            else {
                                // just going to keep existing bestGold as it has higher confidence
                            }
                        }
                    }

                }
            }
            telemetry.addData("Vision", "objects="+size+"  time elapse="+runTime.seconds());
            if (recognition!=null)
                telemetry.addData("Best Gold",
                        " left="+recognition.getLeft()+
                                " right="+recognition.getRight()+
                                " top="+recognition.getTop()+
                                " confidence="+recognition.getConfidence());
            telemetry.update();
            sleep(400);
        }

        tfod.shutdown();

        // convert our best gold into a position.
        // we aim camera to the right 2 positions, so if nothing found, then we're position 1
        if (bestGold==null) goldPosition=1;
            // if the gold is on the left of the screen, then that is the middle of the three
        else if (bestGold.getLeft()<400) goldPosition=2;
            // else it is on the right
        else goldPosition=3;

        telemetry.addData("Gold Mineral Position", goldPosition);
        telemetry.update();

        return goldPosition;
    }

    public void initVision() {
        telemetry.addData("Status", "Initializing Vision...");
        telemetry.update();

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        telemetry.addData("Status", "Vision Initialized");
        telemetry.update();
    }
    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        telemetry.addData("Status", "Vision Vuforia Initializing...");
        telemetry.update();
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = FaltechUtilities.VUFORIA_KEY;
        boolean useBackCamera=true;
        if (useBackCamera)  {
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        } else /*front camera*/ {
            /* lifted this from another example, to go off of the front camera ... or should it just be CameraDirection.FRONT */
            parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        }
        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        telemetry.addData("Status", "Vision Vuforia Initialized");
        telemetry.update();
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        telemetry.addData("Status", "Vision Tensor Flow Initializing...");
        telemetry.update();

        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);

        telemetry.addData("Status", "Vision Tensor Flow Initialized");
        telemetry.update();
    }


}
