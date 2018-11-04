package org.firstinspires.ftc.teamcode;

import java.util.HashMap;

public class FaltechUtilities {

    public static double DEFAULT_DEADZONE = .08;

    // This is the Faltech FTC Vuforia Key
    public static final String VUFORIA_KEY = "AY7lK0j/////AAABmffl0hEQlUFfjdc9h8Aw+t5/CrgiSiIgNkZKZcw3qdOlnNEv3HarcW4e1pfYY5Nq+4XVrrnhKKNBeR/S08U41ogd0NpmWwOPgttli7io4p8WtbgWj+c/WL9uDzZK9u03K3Kfx+XFxdk/vy0tnFKCPg5w9M5iy7QQP2SDHFDJuhcAOtsayV8n8hQvB528RDRDykBtXei/V6xhN/qLc+S1Gp7eS0ZzpDFnT+uED0CwYK+oaWKNsPPv+3u9tCwofQ5PaRHlN05kH4V97Nn0N7WquSmDpcCZpAVqI1QnMEi7Fm9rvJgET+4OIlx4ZueF3ZTuXtJJSaEJ8Y6CEy9F7FS0RnlVtt4QlqpQVSmWmJQWYBNu";


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
        double gear1MaxInput = 0.65;

        //gear 1 outputs
        double gear1MinOutput = 0.05;
        double gear1MaxOutput = 0.25;

        //gear 2 inputs
        double gear2MinInput = 0.65;
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

    static HashMap lastValues = new HashMap();
    /** This function helps to find out if the gamepad button value changed or not.
     * This helps to avoid seeing the same button press each loop, so that you only see
     * the transition from on to off, or off to on.
     * Example:
     *  void loop () {
     *      if (isValueChanged("1.x",gamepad1.x)) {
     *          if (gamepad1.x) ... // button just depressed, won't trigger again until released and pressed again
     *
     *      }
     *  }
     * @param key
     * @param newValue
     * @return
     */
    public static boolean isValueChanged(String key, Object newValue) {
        boolean valueChanged=false;
        Object lastValue=lastValues.get(key);
        if (lastValue!=null && lastValue!=newValue) valueChanged=true;
        lastValues.put(key,newValue);  // store new value so that we know if it changed next time.
        return valueChanged;
    }

    /** wrapper function to check if value was changed AND also equal to a given target
     *
     * @param key
     * @param newValue
     * @param targetValue
     * @return
     */
    public static boolean isValueChangedAndEqualTo(String key, Object newValue, Object targetValue) {
        return (isValueChanged(key,newValue) && newValue==targetValue);
    }
}

