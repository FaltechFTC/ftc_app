package com.github.pmtischler.control;

/**
 * https://en.wikipedia.org/wiki/PID_controller
 * originally from pmtischler, but with modifications to change forms
 */
public class Pid {
    private double kp;     // Proportional factor to scale error to output.
    private double ki;    // coefficient  for Integral term
    private double kd;    // coefficient  for Derivative term
    private double integralMin;    // The min of the running integral.
    private double integralMax;    // The max of the running integral.
    private double outputMin;    // The min allowed PID output.
    private double outputMax;    // The max allowed PID output.
    private double previousError;    // The last error value.
    private double runningIntegral;    // The discrete running integral (bounded by integralMax).
    private double lastOutput, lastPTerm, lastITerm, lastDTerm; // last output and components (for debugging)

    /**
     * Creates a PID Controller.
     * @param kp Proportional factor to scale error to output.
     * @param ki Factor of the integral
     * @param kd Factor of the derivative or dampening.
     * @param integralMin The min of the running integral.
     * @param integralMax The max of the running integral.
     * @param outputMin The min of the PID output.
     * @param outputMax The max of the PID output.
     */
    public Pid(double kp, double ki, double kd, double integralMin,
               double integralMax, double outputMin, double outputMax) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.integralMin = integralMin;
        this.integralMax = integralMax;
        this.outputMin = outputMin;
        this.outputMax = outputMax;

        this.previousError = 0;
        this.runningIntegral = 0;
        lastOutput=0.0;
        lastPTerm=0.0;
        lastITerm=0.0;
        lastDTerm=0.0;
    }

    /**
     * Performs a PID update and returns the output control.
     * @param desiredValue The desired state value (e.g. speed).
     * @param actualValue The actual state value (e.g. speed).
     * @param dt The amount of time (sec) elapsed since last update.
     * @return The output which impacts state value (e.g. motor throttle).
     */
    public double update(double desiredValue, double actualValue, double dt) {
        double error = desiredValue - actualValue;
        runningIntegral = clampValue(runningIntegral + error * dt,
                                     integralMin, integralMax);
        double dErr = (error - previousError) / dt;
        lastPTerm=kp*error;
        lastITerm=runningIntegral * ki;
        lastDTerm=kd * dErr;
        double output = lastPTerm + lastITerm + lastDTerm;
        // clamp to expected range
        output = clampValue(output, outputMin, outputMax);
        lastOutput=output;
        previousError = error;
        return output;
    }

    /**
     * Clamps a value to a given range.
     * @param value The value to clamp.
     * @param min The min clamp.
     * @param max The max clamp.
     * @return The clamped value.
     */
    public static double clampValue(double value, double min, double max) {
        return Math.min(max, Math.max(min, value));
    }

    @Override
    public String toString() {
        return "Pid{" +
                "Kp=" + kp +
                ", Ki=" + ki +
                ", Kd=" + kd +
                ", integralMin=" + integralMin +
                ", integralMax=" + integralMax +
                ", outputMin=" + outputMin +
                ", outputMax=" + outputMax +
                ", previousError=" + previousError +
                ", runningIntegral=" + runningIntegral +
                ", lastOutput=" + lastOutput+
                ", lastPTerm=" + lastPTerm +
                ", lastITerm=" + lastITerm+
                ", lastDTerm=" + lastDTerm+
                '}';
    }

}
