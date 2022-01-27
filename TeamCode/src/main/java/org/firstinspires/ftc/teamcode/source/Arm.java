package org.firstinspires.ftc.teamcode.source;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This is the setup and control class for the arm motors and servos, and the duck spinner motor *
 */


public class Arm {
    //HardwareMap hardwareMap;
    public DcMotor shoulder = null;
    public DcMotor elbow = null;
    public DcMotor spinner = null;
    public Servo claw = null;
    public Servo magnet = null;

    public int shoulderState = 2;
    int shoulderStartPose;
    double shoulderTicksPerRev = 8192;

    public int elbowState = 2;
    int elbowStartPose;
    double elbowTicksPerRev = 8192;

    public Arm(HardwareMap hardwareMap) {
        //initializing all hardware

        shoulder = hardwareMap.get(DcMotor.class, "shoulder");
        elbow = hardwareMap.get(DcMotor.class, "elbow");
        spinner = hardwareMap.get(DcMotor.class, "spinner");
        claw  = hardwareMap.get(Servo.class, "claw");
        magnet = hardwareMap.get(Servo.class, "mag");

        shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //setting the mode to encoder org.firstinspires.ftc.teamcode.drive and setting the encoder position to 0.  This should only have to be done once
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runShoulder(double shoulderDistance/*this is in revolutions*/, double shoulderSpeed/*this should be a positive number*/) {
        switch (shoulderState) {
            case 0: //setup
                if(Math.abs(shoulderSpeed) > 1) {
                    throw new IllegalArgumentException("DcMotor input exceeds allowed range");
                }
                shoulderState = 1;
                shoulderStartPose = shoulder.getCurrentPosition();
                break;
            case 1: //run mode
                shoulder.setPower(shoulderSpeed * speedMultiplier(shoulderDistance > 0) + 0.0005*Math.cos(shoulder.getCurrentPosition()/11.38 + 135));
                if(     (shoulderDistance > 0 && shoulder.getCurrentPosition()*-1 >= shoulderStartPose + Math.round(shoulderDistance * shoulderTicksPerRev))
                        || (shoulderDistance < 0 && shoulder.getCurrentPosition()*-1 <= shoulderStartPose + Math.round(shoulderDistance * shoulderTicksPerRev))
                ) {
                    shoulderState = 2;
                }
                break;
            case 2: //idle
                break;
        }
    }
    public void runElbow(double elbowDistance/*this is in revolutions*/, double elbowSpeed/*this should be a positive number*/) {
        switch (elbowState) {
            case 0: //setup
                if(elbowSpeed > 1 || elbowSpeed < 0)  {
                    throw new IllegalArgumentException("DcMotor input exceeds allowed range (0-1)");
                }
                elbowState = 1;
                elbowStartPose = elbow.getCurrentPosition();
                break;
            case 1: //run mode
                elbow.setPower(elbowSpeed * speedMultiplier(elbowDistance > 0) + 0.0003*Math.cos(elbow.getCurrentPosition()/11.38 - 10));
                if(     (elbowDistance > 0 && elbow.getCurrentPosition()*-1 >= elbowStartPose + Math.round(elbowDistance * elbowTicksPerRev))
                        || (elbowDistance < 0 && elbow.getCurrentPosition()*-1 <= elbowStartPose + Math.round(elbowDistance * elbowTicksPerRev))
                ) {
                    elbowState = 2;
                }
                break;
            case 2: //idle
                break;
        }
    }
    public void holdShoulder() {
        if (shoulderState == 2) {
            elbow.setPower(-0.0005 * Math.cos(shoulder.getCurrentPosition() / 11.38 - 10)
            + -0.0004*Math.cos(elbow.getCurrentPosition()/11.38 + 160));
        }
    }
    public void holdElbow() {
        if(elbowState == 2) {
            elbow.setPower(-0.0003*Math.cos(elbow.getCurrentPosition()/11.38 - 10));
        }
    }
    public void openClaw() {
        claw.setPosition(0.1);
    }
    public void closeClaw() {
        claw.setPosition(0.1);
    }
    public void pushMagnet() {
        magnet.setPosition(0.3);
    }
    public void retractMagnet() {
        magnet.setPosition(0.3);
    }
    public void resetShoulder() {
        shoulderState = 0;
    }
    public void resetElbow() {
        elbowState = 0;
    }
    int speedMultiplier(boolean positive) {
        if (positive) {
            return 1;
        } else {
            return -1;
        }
    }
}
