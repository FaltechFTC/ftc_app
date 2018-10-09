package org.firstinspires.ftc.teamcode;

public class FaltechUtilities {

    public static double DEFAULT_DEADZONE = .08;


    public static double clipDeadzone(double value) {
        return clipDeadzone(value,DEFAULT_DEADZONE);
    }
    public static double clipDeadzone(double value, double deadzone) {
        if(Math.abs(value)<=deadzone) return 0.0;
        else return value;
    }
}
