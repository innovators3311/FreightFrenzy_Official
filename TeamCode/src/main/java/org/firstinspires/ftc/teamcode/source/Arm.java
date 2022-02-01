package org.firstinspires.ftc.teamcode.source;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This is the setup and control class for the arm motors and servos, and the duck spinner motor
 */


public class Arm {
    public DcMotor shoulder = null;
    public DcMotor elbow = null;
    public DcMotor spinner = null;
    public Servo claw = null;
    public Servo magnet = null;

    private int shoulderDistance = 0;
    private int shoulderState = 0;
    private int shoulderStartPose = 0;
    private double shoulderGravity;
    public boolean shoulderIsBusy = false;

    private int elbowDistance = 0;
    private int elbowState = 0;
    private int elbowStartPose = 0;
    private double elbowGravity;
    public boolean elbowIsBusy = false;

    //both shoulder and elbow encoders have 8192 ticks per revolution

    PID shoulderPID = new PID();
    PID elbowPID = new PID();

    private int encoderMultiplier = -1;

    public Arm(HardwareMap hardwareMap) {
        //initializing all hardware
        shoulderPID.setGains(0.05, 0, 0.000001,-1, 1);
        elbowPID.setGains(0.02, 0, 0.0000005, -1, 1);

        shoulder = hardwareMap.get(DcMotor.class, "shoulder");
        elbow = hardwareMap.get(DcMotor.class, "elbow");
        spinner = hardwareMap.get(DcMotor.class, "spinner");
        claw  = hardwareMap.get(Servo.class, "claw");
        magnet = hardwareMap.get(Servo.class, "mag");

        shoulder.setDirection(DcMotor.Direction.FORWARD);
        elbow.setDirection(DcMotor.Direction.FORWARD);

        shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /*
         * Resetting the encoders.  It has to be set to run without encoder mode or else a qualcomm class appears to put a PID controller on the motor velocity, which will
         * mess with any of your own code.
         */
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void updateShoulder() {
        switch (shoulderState) {
            case 0: //setup
                shoulderState = 1;
                shoulderStartPose = shoulder.getCurrentPosition();
                break;
            case 1: //run mode
                shoulderPID.update(encoderMultiplier * shoulderStartPose / 22.76 + shoulderDistance, encoderMultiplier * shoulder.getCurrentPosition() / 22.76);
                shoulderGravity =  0.05 * Math.cos( Math.toRadians(encoderMultiplier * shoulder.getCurrentPosition() / 22.76 + 150) );
                shoulder.setPower(shoulderPID.output + shoulderGravity); //forward is towards the back of the robot
                if(Math.abs(shoulderPID.Err) < 5) {
                    shoulderIsBusy = false;
                }
        }
    }
    public void updateElbow() {
        switch (elbowState) {
            case 0: //setup
                elbowState = 1;
                elbowStartPose = elbow.getCurrentPosition();
                break;
            case 1: //run mode
                elbowPID.update(encoderMultiplier * elbowStartPose / 22.76 + elbowDistance, encoderMultiplier * elbow.getCurrentPosition() / 22.76);
                elbowGravity =  0.02 * Math.cos(
                        Math.toRadians(encoderMultiplier * shoulder.getCurrentPosition() / 22.76 + encoderMultiplier * elbow.getCurrentPosition() / 22.76 - 20)
                );
                elbow.setPower(elbowPID.output + elbowGravity);
                if(Math.abs(elbowPID.Err) < 5) {
                    elbowIsBusy = false;
                }
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
        magnet.setPosition(0);
    }

    public void runShoulderTo(int shoulderDistance) {
        shoulderState = 0;
        shoulderIsBusy = true;
        this.shoulderDistance = shoulderDistance;
    }
    public void runElbowTo(int elbowDistance) {
        elbowState = 0;
        elbowIsBusy = true;
        this.elbowDistance = elbowDistance;
    }
    public void storeArmPose() {
        PoseStorage.shoulderTicks = shoulder.getCurrentPosition();
        PoseStorage.elbowTicks = elbow.getCurrentPosition();
    }
}
