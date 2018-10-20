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
    public static double scaleSpeedFunction(double input) {
        double output;
        double direction = 1.0;

        //gear 1 inputs
        double gear1MinInput = 0.1;
        double gear1MaxInput = 0.5;

        //gear 1 outputs
        double gear1MinOutput = 0.05;
        double gear1MaxOutput = 0.25;

        //gear 2 inputs
        double gear2MinInput = 0.5;
        double gear2MaxInput = 1;

        //gear 2 outputs
        double gear2MinOutput = 0.25;
        double gear2MaxOutput = 1;

        if (input < 0) direction = -1;

        input=Math.abs(input);
        if (input < gear1MinInput) {
            output = 0;
        } else if (input < gear1MaxInput) {
            double p = (input - gear1MinInput) / (gear1MaxInput - gear1MinInput);
            output = direction * (gear1MinOutput + p*(gear1MaxOutput - gear1MinOutput));
        } else {
            double p = (input - gear2MinInput) / (gear2MaxInput - gear2MinInput);
            output = direction *( gear2MinOutput + p*(gear2MaxOutput - gear2MinOutput));
        }
        return output;
    }
}

