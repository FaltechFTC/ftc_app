package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="RoverTankAuto", group="7079")
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
     */

    private RoverRobot robot = null;
    RoverRobot.Operation operation= null;

    @Override
    public void runOpMode() {
        telemetry.addData("About to start", "Robot Initialization");
        telemetry.update();
        robot = new RoverRobot(new DriveTank());


        robot.init(hardwareMap, telemetry);
        telemetry.addData("Robot" , "Initialized");
        telemetry.update();

        waitForStart();

        double maxTurningPower=.3;
        double maxDrivePower=.5;
        double degreesError=2.0;
        long timeoutMS=4000;

        robot.drive.driveFRS(0.3,0.0,0.0);
        sleep(2000);
        robot.stop();

        operation = robot.getOperationRotateToHeading(-75, maxTurningPower, degreesError, timeoutMS);
        operation.run();
        robot.stop();

        robot.drive.driveFRS(0.6,0.0,0.0);
        sleep(2000);
        robot.stop();


        operation = robot.getOperationRotateToHeading(-60, maxTurningPower, degreesError, timeoutMS);
        operation.run();
        robot.stop();

        robot.drive.driveFRS(0.6,0.0,0.0);
        sleep(4500);
        robot.stop();

        robot.drive.driveFRS(-0.6,0.0,0.0);
        sleep(5500);
        robot.stop();


    }


}