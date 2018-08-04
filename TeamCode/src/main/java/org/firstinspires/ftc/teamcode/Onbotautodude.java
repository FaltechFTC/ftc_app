package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="OnbotautoDUDE", group="FaltechH2O")

public class Onbotautodude extends LinearOpMode {

    /* Declare OpMode members. */
    FaltechH2ORobot robot   = new FaltechH2ORobot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();


    static final double     FORWARD_SPEED = 0.6;
    static final double     TURN_SPEED    = 0.5;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();
        robot.claw.setPosition(1.0);
        robot.clawHinge.setPosition(1.0);
        robot.dude.setPosition(1.0);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        robot.clawHinge.setPosition(0.0);

        robot.leftDrive.setPower(FORWARD_SPEED);
        robot.rightDrive.setPower(FORWARD_SPEED);
        sleep(2600);

        robot.leftDrive.setPower(-FORWARD_SPEED/2);
        robot.rightDrive.setPower(FORWARD_SPEED/2);
        runtime.reset();
        sleep(820);

        robot.leftDrive.setPower(FORWARD_SPEED/2);
        robot.rightDrive.setPower(FORWARD_SPEED/2);
        sleep(1780);

        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);


        robot.clawHinge.setPosition(0.5);
        sleep(100);

        robot.claw.setPosition(0.0); // open claw
        sleep(500);
        robot.clawHinge.setPosition(0.0);  // lift claw
        sleep(1000);

        // doodoo stuff goes here
        robot.leftDrive.setPower(-FORWARD_SPEED/2);
        robot.rightDrive.setPower(-FORWARD_SPEED/2);
        sleep(170);


        robot.leftDrive.setPower(-FORWARD_SPEED/2);
        robot.rightDrive.setPower(FORWARD_SPEED/2);
        sleep(920);

        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);

        robot.dude.setPosition(0.0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}