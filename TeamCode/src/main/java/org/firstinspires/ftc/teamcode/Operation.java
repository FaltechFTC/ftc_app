package org.firstinspires.ftc.teamcode;

public class Operation {
    protected RoverRobot robot;
    public long timeoutMS=9999;
    public long startMS=System.currentTimeMillis();
    public long curMS=startMS, lastTime=startMS;
    public double deltaTime =0;

    public boolean done=false;
    public boolean timeout=false;
    public long numLoops=0;
    public boolean stopDriveOnDone=true;
    public boolean coastOnStop=false;
    public Operation(RoverRobot robot) {
        this.robot = robot;
    }

    public boolean run() {
        return run(10);
    }
    public boolean run(long timeBetweenLoopsMS) {
        while(loop()) {
            robot.telemetry.update();
            robot.sleep(timeBetweenLoopsMS);
        }
        return !timeout; // true if we ran to completion, else false means we timed out
    }

    public boolean loop() {
        if (done) return false;
        numLoops++;
        curMS=System.currentTimeMillis();
        deltaTime= ((double) (curMS - lastTime)) / 1000.0;
        lastTime = curMS;
        if (curMS-startMS > timeoutMS) { timeout=true; done(); return false;}
        return true;
        // keep looping ...
    }

    public void done() {
        done=true;
        if (stopDriveOnDone) {
            //
            if(!coastOnStop)
                robot.drive.setHaltModeCoast(false);
            robot.drive.stop();
            if(!coastOnStop){
                robot.sleep(500);
                robot.drive.setHaltModeCoast(true);
            }

        }
    }

    public double getRuntime() {
        double runtime=((double) (curMS - startMS)) / 1000.0;
        return runtime; // seconds
    }

    public String getResult() {
        return String.format("Loops=%d TotalTime=%3.1f",numLoops, getRuntime());
    }
}
