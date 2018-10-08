/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the FaltechH2ORobot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="PidTeleop", group="7079")

public class FaltechPIDTeleop extends OpMode{

    /* Declare OpMode members. */
    FaltechH2ORobot robot       = new FaltechH2ORobot(); // use the class created to define a Pushbot's hardware
                                                         // could also use FaltechH2ORobotMatrix class.
    double          clawOffset  = 0.0 ;                  // Servo mid position
    final double    CLAW_SPEED  = 0.02 ;                 // sets rate to move servo

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap,telemetry);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Insert Social Security Number to Play Game, **not a scam**");    //
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        processDriving ();
        processPID();
    }
    
    public void processDriving() {
        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        double forward = deadstick (-gamepad1.right_stick_y);
        double side= deadstick (gamepad1.left_stick_x)/2;

        double leftDrive=forward+side;
        double rightDrive=forward-side;

        double maxPower=1.0;
        maxPower=Math.max(maxPower,Math.abs(forward));
        maxPower=Math.max(maxPower,Math.abs(side));

        leftDrive=leftDrive/maxPower;
        rightDrive=rightDrive/maxPower;

        robot.leftDrive.setPower(leftDrive);
        robot.rightDrive.setPower(rightDrive);


        telemetry.addData("angles", "relative="+robot.getRelativeAngle()+" absolute"+robot.getCurrentAbsoluteAngle());
        telemetry.addData("drivejoy",  "%.2f ,%.2f", forward,side);
        telemetry.addData("drivepower",  "%.2f ,%.2f", leftDrive, rightDrive);
        telemetry.update();





    }

    public void processPID() {
        double maxTurningPower=.3;
        double maxDrivePower=.5;
        if (gamepad1.x) robot.rotate2(45,maxTurningPower);
        if (gamepad1.y) robot.rotate2(-45,maxTurningPower);
        if (gamepad1.a) robot.driveToHeading(0,maxDrivePower, maxTurningPower/2, 4000);
        if (gamepad1.b) robot.driveToHeading(15,maxDrivePower, maxTurningPower/2, 4000);
        if (gamepad1.dpad_up) robot.resetRelativeAngleToZero();

    }

    public float deadstick (float value){

        if (value > -0.07 && value < 0.07)
           return 0 ;
        else
            return value;
    }

    public void processDude() {
        double pos=1.0;
        
        if (gamepad1.y)
            pos=0.0;

      //  robot.dude.setPosition(pos);

        // Send telemetry message to signify robot running;
        telemetry.addData("claw", "Offset = %.2f", pos);
        telemetry.update();
    }

    public void processClaw() {
        double posclaw = 0.0;
        double posclawhinge = 0.0;

        if (gamepad1.right_trigger > 0.3) posclaw = 1.0;
        if (gamepad1.left_trigger > 0.3) posclawhinge = 1.0;

//        robot.claw.setPosition(posclaw);
//        robot.clawHinge.setPosition(posclawhinge);
    }
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
    }
}
