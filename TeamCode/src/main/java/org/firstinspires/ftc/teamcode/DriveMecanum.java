/* this class has all the wheel and drive functions defined.  All the scaling functions, and our
main drive FRS function is also part of this class
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/**
 *
 * Borrowed heavily (motion and wheels portions) from original author and source from https://github.com/pmtischler/ftc_app/blob/master/SharedCode/src/main/java/com/github/pmtischler/control/Mecanum.java
 *
 *  Modified by Faltech 7079 ...
 *
 * Mecanum wheel drive calculations.
 * Input controls:
 *   V_d = desired robot speed.
 *   theta_d = desired robot velocity angle.
 *   V_theta = desired robot rotational speed.
 *
 *  Example:
 *    // Convert joysticks to wheel powers.
 *    Mecanum.Wheels wheels = Mecanum.motionToWheels(
 *        Mecanum.joystickToMotion(
 *            gamepad1.left_stick_x, gamepad1.left_stick_y,
 *            gamepad1.right_stick_x, gamepad1.right_stick_y));
 *    // Set power on the motors.
 *    frontLeftMotor.setPower(wheels.frontLeft);
 */
public class DriveMecanum extends IDrive{
    public double speedAdjusters[] = new double[]{0.86,0.75,1.0,0.76};
    public DcMotor mtrFL = null;
    public DcMotor mtrFR = null;
    public DcMotor mtrBL = null;
    public DcMotor mtrBR = null;
    public DcMotor[] motors=null;
    public int gearValue = 1;
    public int debugLevel=1;
    HardwareMap hwMap = null;
    public Telemetry telemetry = null;

    private ElapsedTime runTime = new ElapsedTime();

    public DriveMecanum() {

    }

    public double getMaxPower() { return maxPower;}
    public void setMaxPower(double p) {maxPower=p;}


    public boolean isMecanum() { return true;}


    @Override
    public void init(HardwareMap hwMap, Telemetry telemetry) {
        // Save reference to Hardware map
        this.hwMap = hwMap;
        this.telemetry = telemetry;

        // Define and Initialize Motors
        mtrFL = hwMap.get(DcMotor.class, "mtrFL");
        mtrFR = hwMap.get(DcMotor.class, "mtrFR");
        mtrBL = hwMap.get(DcMotor.class, "mtrBL");
        mtrBR = hwMap.get(DcMotor.class, "mtrBR");

        motors = new DcMotor[] {mtrFL, mtrFR, mtrBL, mtrBR};

        // set motor direction based on installed orientation.
        mtrFL.setDirection(DcMotor.Direction.FORWARD);
        mtrFR.setDirection(DcMotor.Direction.REVERSE); // was FORWARD
        mtrBL.setDirection(DcMotor.Direction.FORWARD); // was REVERSE
        mtrBR.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        setHaltModeCoast(true);
        setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        stop();
    }


    @Override
    public void driveFRS(double forward, double rotate, double sideways, double maxPower) {
        // TODO: maxPower is never used... remove, or impliment version that doesn't ask for it.
        RobotLog.i("driveFRS(F=%3.2f  R=%3.2f  S=%3.2f ", forward, rotate, sideways);
        Motion motion=createMotionFromFRS(sideways,forward,rotate);
        if (debugLevel>1) RobotLog.i("Motion(vD=%3.2f  thetaD=%3.2f  vTheta=%3.2f ", motion.vD, motion.thetaD, motion.vTheta);
        Wheels wheels = motionToWheels(motion);
        setPower(wheels);
    }

    @Override
    public void setHaltModeCoast(boolean coastModeOn) {
        DcMotor.ZeroPowerBehavior behavior;
        if (coastModeOn) behavior=DcMotor.ZeroPowerBehavior.FLOAT;
        else behavior=DcMotor.ZeroPowerBehavior.BRAKE;

        for (DcMotor m : motors) m.setZeroPowerBehavior(behavior);
    }

    @Override
    public void setRunModeEncoder(boolean encoderModeOn) {
        if (encoderModeOn) setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        else setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void setEncoderToRunToPosition() {

         setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setRunMode(DcMotor.RunMode runMode) {
        for (DcMotor m : motors) m.setMode(runMode);
    }

    @Override
    public void stop() {
        for (DcMotor m : motors) m.setPower(0);
    }

    public void setPower(Wheels w) {
        mtrFL.setPower(w.frontLeft*speedAdjusters[0]);
        mtrFR.setPower(w.frontRight*speedAdjusters[1]);
        mtrBL.setPower(w.backLeft*speedAdjusters[2]);
        mtrBR.setPower(w.backRight*speedAdjusters[3]);
        if (debugLevel>1) {
            String s = w.toString();
            telemetry.addData("Powers (pre adj)", s);
            telemetry.addData("Wheel Pos", String.format("FL %5d  FR %5d  BL %5d  BR %5d", mtrFL.getCurrentPosition(), mtrFR.getCurrentPosition(), mtrBL.getCurrentPosition(), mtrBR.getCurrentPosition()));
            RobotLog.i("Powers (pre adj): " + s);
        }
    }


    // encoder clicks
    public double getClicksPerRevolution() {
        return 560;
    }
    public double getGearReduction() {
        return 1.0;
    }
    public double getWheelCircumfrence() {
        return 12.56; // inches
    }

    public void resetEncoders() {
        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public double getEncoderClicksLeft() {
        return (mtrBL.getCurrentPosition() + mtrFL.getCurrentPosition())/2.0;
    }

    public double getEncoderClicksRight() {
        return (mtrBR.getCurrentPosition() + mtrFR.getCurrentPosition())/2.0;
    }

    public double getEncoderClicksLeftAbs() {
        return (Math.abs(mtrBL.getCurrentPosition()) + Math.abs(mtrFL.getCurrentPosition()))/2.0;
    }

    public double getEncoderClicksRightAbs() {
        return (Math.abs(mtrBR.getCurrentPosition()) + Math.abs(mtrFR.getCurrentPosition()))/2.0;
    }

    public double getEncoderClicksFront() {
        return (mtrFL.getCurrentPosition() + mtrFR.getCurrentPosition())/2.0;
    }
    public double getEncoderClicksBack() {
        return (mtrBL.getCurrentPosition() + mtrBR.getCurrentPosition())/2.0;
    }

    /**
     * Mecanum motion vector.
     */
    public static class Motion {
        // Robot speed [-1, 1].
        public final double vD;
        // Robot angle while moving [0, 2pi].
        public final double thetaD;
        // Speed for changing direction [-1, 1].
        public final double vTheta;

        /**
         * Sets the motion to the given values.
         */
        public Motion(double vD, double thetaD, double vTheta) {
            this.vD = vD;
            this.thetaD = thetaD;
            this.vTheta = vTheta;
        }
    }

    /**
     * Gets the motion vector from the joystick values.
     * @param forward example: The left joystick Y.
     * @param sideways example: The left joystick X.
     * @param rotate example: The right joystick X.
     * @return The Mecanum motion vector.
     */
    public static Motion createMotionFromFRS(double forward, double sideways, double rotate) {
        // V_d = desired robot speed.
        // theta_d = desired robot velocity angle.
        // V_theta = desired robot rotational speed.
        double vD = Math.min(Math.sqrt(Math.pow(sideways, 2) + Math.pow(forward, 2)), 1);
        double thetaD = Math.atan2(-sideways, -forward);
        double vTheta = -rotate;
        return new Motion(vD, thetaD, vTheta);
    }

    /**
     * Mecanum wheels, used to get individual motor powers.
     */
    public static class Wheels {
        // The mecanum wheels.
        public final double frontLeft;
        public final double frontRight;
        public final double backLeft;
        public final double backRight;

        /**
         * Sets the wheels to the given values.
         */
        public Wheels(double frontLeft, double frontRight,
                      double backLeft, double backRight) {
            List<Double> powers = Arrays.asList(frontLeft, frontRight,
                                                backLeft, backRight);
            clampPowers(powers);

            this.frontLeft = powers.get(0);
            this.frontRight = powers.get(1);
            this.backLeft = powers.get(2);
            this.backRight = powers.get(3);
        }

        /**
         * Scales the wheel powers by the given factor.
         * @param scalar The wheel power scaling factor.
         */
        public Wheels scaleWheelPower(double scalar) {
            return new Wheels(frontLeft * scalar, frontRight * scalar,
                              backLeft * scalar, backRight * scalar);
        }

        public String toString() {
            return String.format("Wheels(FL=%3.2f FR=%3.2f BL=%3.2f BR=%3.2f)",frontLeft,frontRight,backLeft,backRight);
        }
    }

    /**
     * Gets the wheel powers corresponding to desired motion.
     * @param motion The Mecanum motion vector.
     * @return The wheels with clamped powers. [-1, 1]
     */
    public static Wheels motionToWheels(Motion motion) {
        double vD = motion.vD;
        double thetaD = motion.thetaD;
        double vTheta = motion.vTheta;

        double frontLeft = vD * Math.sin(-thetaD + Math.PI / 4) - vTheta;
        double frontRight  = -vD * Math.cos(-thetaD + Math.PI / 4) + vTheta; // was -vTheta  (and was +vD)
        double backLeft = -vD * Math.cos(-thetaD + Math.PI / 4) - vTheta;  // was +vTheta (and was +vD)
        double backRight = vD * Math.sin(-thetaD + Math.PI / 4) + vTheta;
        return new Wheels(frontLeft, frontRight, backLeft, backRight);
    }

    /**
     * Clamps the motor powers while maintaining power ratios.
     * @param powers The motor powers to clamp.
     */
    private static void clampPowers(List<Double> powers) {
      double minPower = Collections.min(powers);
      double maxPower = Collections.max(powers);
      double maxMag = Math.max(Math.abs(minPower), Math.abs(maxPower));

      if (maxMag > 1.0) {
        for (int i = 0; i < powers.size(); i++) {
          powers.set(i, powers.get(i) / maxMag);
        }
      }
    }

    double motor_count = 1120;
    double gear_reduction = 0.5;
    double wheel_circumference = 12.56;
    double counts_per_inch = (motor_count * gear_reduction / wheel_circumference);

    public void setTargetPosition(int targetPosition){

        mtrFR.setTargetPosition(mtrFR.getCurrentPosition()-targetPosition);
        mtrBR.setTargetPosition(mtrBR.getCurrentPosition()+targetPosition);
        mtrFL.setTargetPosition(mtrFL.getCurrentPosition()+targetPosition);
        mtrBL.setTargetPosition(mtrBL.getCurrentPosition()-targetPosition);
    }
    private ElapsedTime runtime = new ElapsedTime();

    private void timeOutExit(double timeout){

        while ((runtime.seconds() < (timeout))
                && (mtrBL.isBusy() &&mtrBR.isBusy()
                && mtrFL.isBusy() && mtrFR.isBusy())) {

            // Display it for the driver.
            telemetry.addData("Path1",  "Running to target position");
            telemetry.addData("Path2",  "Running at:",
            mtrBL.getCurrentPosition(),
            mtrBR.getCurrentPosition(),
            mtrFL.getCurrentPosition(),
            mtrFR.getCurrentPosition());
            telemetry.update();
        }

    }

}

