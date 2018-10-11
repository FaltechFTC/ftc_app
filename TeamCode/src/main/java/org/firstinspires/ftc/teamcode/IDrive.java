package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class IDrive {

    abstract public void init(HardwareMap ahwMap, Telemetry telemetry);

    public void driveFRS(double forward, double rotate, double sideways) {
        driveFRS(forward,rotate, sideways, getMaxPower());
    }
    abstract public void driveFRS(double forward, double rotate, double sideways, double maxPower);

    abstract public void setHaltModeCoast(boolean coastModeOn);
    abstract public void setRunModeEncoder(boolean encoderModeOn);

    abstract public void stop();

    public boolean isMecanum() { return false;}

    double maxPower =1.0;
    public double getMaxPower() { return maxPower;}
    public void setMaxPower(double p) {maxPower=p;}


    // todo, adding encoding style functions here?
    // todo, add getters here
}
