package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class IDrive {

    abstract public void init(HardwareMap ahwMap, Telemetry telemetry);

    public void driveFRS(double forward, double rotate, double sideways) {
        driveFRS(forward,rotate, sideways, getMaxPower());
    }
    abstract public void driveFRS(double forward, double rotate, double sideways, double maxPower);
    abstract public void driveToInches(double inches, double power, double timeOut);

    abstract public void setHaltModeCoast(boolean coastModeOn);
    abstract public void setRunModeEncoder(boolean encoderModeOn);

    abstract public void stop();

    public boolean isMecanum() { return false;}

    double maxPower =1.0;

    public double getMaxPower() { return maxPower;}
    public void setMaxPower(double p) {maxPower=p;}


    abstract public double getClicksPerRevolution(); // encoder clicks
    abstract public double getGearReduction(); // ratio
    abstract public double getWheelCircumfrence();  // inches

    abstract public void resetEncoders();
    abstract public double getEncoderClicksLeft();
    abstract public double getEncoderClicksRight();

    public double getEncoderClicks() {
        return (getEncoderClicksLeft()+getEncoderClicksRight())/2.0;
    }

    public double convertInchesToClicks( double inches) {
        return inches* getClicksPerRevolution() * getGearReduction() / getWheelCircumfrence();
    }

    public double convertClicksToInches(double clicks) {
        return clicks*getWheelCircumfrence()/(getClicksPerRevolution()*getGearReduction());
    }



    // todo, adding encoding style functions here?
    // todo, add getters here

    public double clicksPerInch(){

        return(0.0);
    }



}
