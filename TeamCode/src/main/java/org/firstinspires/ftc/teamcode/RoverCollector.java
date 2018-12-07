package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RoverCollector {
    HardwareMap hwMap = null;
    IDrive drive = null;

    public DcMotor mtrLeftCollector = null;
    public DcMotor mtrRightCollector = null;
    public DcMotor mtrArmExtender = null;
    public int zeroOffset = 0;

    public Servo leftClaw = null;
    public Servo rightClaw = null;

    boolean newMode = true;

    private ElapsedTime period = new ElapsedTime();
    public Telemetry telemetry = null;
    public RoverCollector(){
    }

    public void init(HardwareMap hwMap, Telemetry telemetry, boolean isAutonomous) {
        // Save reference to Hardware map
        this.hwMap = hwMap;
        this.telemetry = telemetry;

        mtrArmExtender = hwMap.get(DcMotor.class, "mtrArmExtender");
        mtrArmExtender.setDirection(DcMotor.Direction.FORWARD);
        mtrArmExtender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        mtrLeftCollector = hwMap.get(DcMotor.class, "mtrLeftCollector");
        mtrRightCollector = hwMap.get(DcMotor.class, "mtrRightCollector");

        mtrLeftCollector.setDirection(DcMotor.Direction.FORWARD);
        mtrRightCollector.setDirection(DcMotor.Direction.REVERSE);


            mtrLeftCollector.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mtrRightCollector.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mtrLeftCollector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            mtrRightCollector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            mtrLeftCollector.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mtrRightCollector.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        leftClaw = hwMap.get(Servo.class, "leftClaw");
        rightClaw = hwMap.get(Servo.class, "rightClaw");
        // not initializing them, as we're going to manually have them in closed position
    }


    public void setClawsToCloselPosition(){
        leftClaw.setPosition(0.0);
        rightClaw.setPosition(0.0);
    }
    public void setClawsToOpenPosition(){
        leftClaw.setPosition(1.0);
        rightClaw.setPosition(1.0);
    }
    double armSpeed;
    double encoderClicksPerRevolution=1120.0;
    double motorGearBoxRatio=40.0;
    double armGearRatio=125.0/45.0;   //45 tooth driving 125 tooth gear
    double clicksPerDegree=encoderClicksPerRevolution*motorGearBoxRatio*armGearRatio/360.0;

    public int convertDegreesToEncoder(double inputDegrees){
        int clicks = (int) (inputDegrees*clicksPerDegree);
        return clicks;

    }
    public double currentDegrees(DcMotor mtr){
      double current = mtr.getCurrentPosition();
      return current/clicksPerDegree;
    }

//    public double calculateArmSpeed(double target, DcMotor motor){
//        if (Math.abs((targetDegrees(target) - currentDegrees(motor))) > 15){
//            armSpeed = 0.8;
//        }
//        else if (Math.abs((targetDegrees(target) - currentDegrees(motor))) > 0) {
//            armSpeed = 0.2;
//        }
//        else{
//            armSpeed = 0;
//        }
//        return armSpeed;
//    }

    public void setPositionDegrees(double targetDegrees) {
        setPosition(convertDegreesToEncoder(targetDegrees));
    }

    public void setPosition(int targetEncoder) {
        int curLeft=mtrLeftCollector.getCurrentPosition();
        int curRight=mtrRightCollector.getCurrentPosition();

        mtrLeftCollector.setTargetPosition(targetEncoder);
        mtrRightCollector.setTargetPosition(-targetEncoder);
        telemetry.addData("Current Arm Positions ", "Left="+curLeft+" Right="+(-curRight));
        telemetry.addData("Target Arm Position ", targetEncoder);
    }

    public void setSpeed(double armSpeed) {
        double targetPower = 0;
        if (Math.abs(armSpeed) > 0.5) targetPower = 0.6 ;
        else if (Math.abs(armSpeed) > 0) targetPower = 0.2;
        else targetPower = 0;


        mtrLeftCollector.setPower(targetPower);
        mtrRightCollector.setPower(targetPower);
    }

    public boolean setPositionIncremental(double targetChangeDegrees, double armSpeed, double armLiftHoldPower)
    {
        int curLeft=Math.abs(mtrLeftCollector.getCurrentPosition());
        int curRight=Math.abs(mtrRightCollector.getCurrentPosition());

        int avgPos=(curLeft+curRight)/2;
        int targetEncoder=avgPos+convertDegreesToEncoder(targetChangeDegrees);

        mtrLeftCollector.setTargetPosition(targetEncoder);
        mtrRightCollector.setTargetPosition(-targetEncoder);

        boolean atTarget=false;
        if (Math.abs(curLeft-curRight)<10 && Math.abs(curLeft-targetEncoder)<10) atTarget=true;
        double targetPower;

        if (armSpeed > 0 ) targetPower = armSpeed ;
        else targetPower=armLiftHoldPower;

        mtrLeftCollector.setPower(targetPower);
        mtrRightCollector.setPower(targetPower);

        telemetry.addData("Current Arm Positions ", "Left="+curLeft+" Right="+(-curRight));
        telemetry.addData("Target Arm Position ", targetEncoder);
        telemetry.addData("Arm Power", targetPower);
        return atTarget;
    }

    //old one
    public boolean setPositionIncrementalold(double targetChangeDegrees, double armSpeed, double armLiftHoldPower)
    {
        int curLeft=Math.abs(mtrLeftCollector.getCurrentPosition());
        int curRight=Math.abs(mtrRightCollector.getCurrentPosition());

        int avgPos=(curLeft+curRight)/2;
        int targetEncoder=avgPos+convertDegreesToEncoder(targetChangeDegrees);

        mtrLeftCollector.setTargetPosition(targetEncoder);
        mtrRightCollector.setTargetPosition(-targetEncoder);

        boolean atTarget=false;
        if (Math.abs(curLeft-curRight)<10 && Math.abs(curLeft-targetEncoder)<10) atTarget=true;
        double targetPower;

        if (atTarget) targetPower=armLiftHoldPower;
        else targetPower=armSpeed;

        mtrLeftCollector.setPower(targetPower);
        mtrRightCollector.setPower(targetPower);

        telemetry.addData("Current Arm Positions ", "Left="+curLeft+" Right="+(-curRight));
        telemetry.addData("Target Arm Position ", targetEncoder);
        telemetry.addData("Arm Power", targetPower);
        return atTarget;
    }


    int maxArmEncoder=1355;  //convertDegreesToEncoder(110);

    public void setCollectorToCoast(){
        mtrLeftCollector.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        mtrRightCollector.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }

    public void setPowerToArmExtender(double power){
        double curLeft=mtrLeftCollector.getCurrentPosition();
        double curRight=mtrRightCollector.getCurrentPosition();


        double easeInDistance=150;
        double leftPower=power, rightPower=power;
        if (leftPower>0 && curLeft>maxArmEncoder-easeInDistance)  leftPower*=(maxArmEncoder-curLeft)/easeInDistance;
        else if (leftPower<0 && curLeft<easeInDistance)  leftPower*=curLeft/easeInDistance;

        if (rightPower>0 && curRight>maxArmEncoder-easeInDistance)  rightPower*=(maxArmEncoder-curRight)/easeInDistance;
        else if (rightPower<0 && curRight<easeInDistance)  rightPower*=curRight/easeInDistance;

        mtrLeftCollector.setPower(leftPower);
        mtrRightCollector.setPower(rightPower);

        telemetry.addData("Arm Lift", "Left="+curLeft+" Right="+curRight+" easeIn="+easeInDistance+" maxPos="+maxArmEncoder);
        telemetry.addData("Arm Lift Inputs", "power="+power+" Lpower="+leftPower+" Rpower="+rightPower);
    }

    public void setPowerToArmLift(double power){
        mtrLeftCollector.setPower(power);
        mtrRightCollector.setPower(power);

    }


}
