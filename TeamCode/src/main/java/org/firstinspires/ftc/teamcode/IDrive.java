package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class IDrive {

    abstract public void init(HardwareMap ahwMap, Telemetry telemetry);

    abstract public void driveFRS(double forward, double rotate, double sideways);

    abstract public void setHaltModeCoast(boolean coastModeOn);
    abstract public void setRunModeEncoder(boolean encoderModeOn);

    abstract public void stop();

    public boolean isMecanum() { return false;}

    // todo, adding encoding style functions here?
    // todo, add getters here
}
