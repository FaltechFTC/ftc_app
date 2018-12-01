package org.firstinspires.ftc.teamcode;

public class Operation {
    protected RoverRobot robot;
    public long timeoutMS=9999;
    public long startMS=System.currentTimeMillis();
    public long curMS=0;
    public boolean done=false;
    public boolean timeout=false;
    public long numLoops=0;
    public Operation(RoverRobot robot) {
        this.robot = robot;
    }

    public boolean run() {
        return run(10);
    }
    public boolean run(long timeBetweenLoopsMS) {
        while(loop()) {
            robot.sleep(timeBetweenLoopsMS);
        }
        return !timeout; // true if we ran to completion, else false means we timed out
    }

    public boolean loop() {
        if (done) return false;
        numLoops++;
        curMS=System.currentTimeMillis();
        if (curMS-startMS > timeoutMS) {timeout=true; done(); return false;}
        return true;
        // keep looping ...
    }

    public void done() {
        done=true;
    }
}