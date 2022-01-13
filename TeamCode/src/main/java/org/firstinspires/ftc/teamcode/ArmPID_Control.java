package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.PIDControl;


public class ArmPID_Control extends Arm2_Control {

    //create variables for motors/servos
    public DcMotorEx elbow = null;
    public DcMotorEx shoulder = null;

    public PIDControl shoulderPID = null;
    public PIDControl elbowPID = null;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private Object Servo;

    //    private double SHOULDER_GRAVITY_FACTOR = -0.05;
    private final double SHOULDER_GRAVITY_FACTOR = 0;

    @Override
    public void init(HardwareMap ahwMap) {

        // Save reference to Hardware map
        hwMap = ahwMap;

        //declare motors so that they can be used
        elbow = hwMap.get(DcMotorEx.class, "elbow");
        shoulder = hwMap.get(DcMotorEx.class, "shoulder");

        shoulderPID = new PIDControl(shoulder, 1e-3, 0.0, 0.0);
        elbowPID = new PIDControl(elbow, 1e-3, 0.0, 0.0);

        // Set the direction that the motors will turn
        elbow.setDirection(DcMotor.Direction.FORWARD);
        shoulder.setDirection(DcMotor.Direction.FORWARD);
        // Note: you can not set direction on a servo

        //make sure motors are at zero power
        elbow.setPower(0);
        shoulder.setPower(0);
        shoulder.setVelocity(8192);

        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //brake the motors
        elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void elbowDriveAbsolute(double speed,
                                   double elbowAngle) {
        elbowPID.setTargetAngle(elbowAngle);
        elbowPID.maxPower = speed;
    }

    @Override
    public void shoulderDriveAbsolute(double speed,
                                      double shoulderAngle) {
        shoulderPID.setTargetAngle(shoulderAngle);
        shoulderPID.maxPower = speed;
    }

    @Override
    public double getElbowTargetAngle() {
        return elbowPID.getTargetAngle();
    }

    @Override
    public double getShoulderTargetAngle() {
        return shoulderPID.getTargetAngle();
    }

    @Override
    public void update() {
        shoulderPID.update();
        elbowPID.update();
    }

    @Override
    public boolean isBusy() {
        // Might want to have this check the shoulderPID and elbowPID's isBusy functions.
        return shoulder.isBusy() || elbow.isBusy();
    }

    @Override
    public void scaleShoulderPid(double v) {
        shoulderPID.p *= v;
    }
}                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       //Hi. You found me. -SECRET COMMENT