package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;

public class PIDControl {
    public DcMotor motor;
    public int targetEncoder = 0;
    public double maxPower=0.2;

    private double TICKSPERDEGREE = 8192.0/360.0;
    public double p;
    public double i;
    public double d;

    public PIDControl(DcMotor motorValue, double pValue, double iValue, double dValue){
        this.p=pValue;
        this.d=dValue;
        this.i=iValue;
        this.motor = motorValue;
    }

    public static float clamp(float val, float min, float max) {
        return Math.max(min, Math.min(max, val));
    }
    public static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    public void update(){
     motor.setPower(clamp(p * err(), -1.0, 1.0) / maxPower);
    }

    public double err(){
        return (motor.getCurrentPosition() - targetEncoder) / TICKSPERDEGREE;
    }
    public void setTargetAngle(double targetAngle){
        targetEncoder = (int)(targetAngle * TICKSPERDEGREE);
    }

}
