package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.PIDControl;


public class ArmPID_Control extends Arm2_Control {

    public PIDControl shoulderPID = null;
    public PIDControl elbowPID = null;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private Object Servo;

    @Override
    public void init(HardwareMap ahwMap) {

        // Save reference to Hardware map
        hwMap = ahwMap;

        //declare motors so that they can be used
        elbow = hwMap.get(DcMotorEx.class, "elbow");
        shoulder = hwMap.get(DcMotorEx.class, "shoulder");

        shoulderPID = new PIDControl(shoulder, -3e-3, -1e-4, -3e-5);
        elbowPID = new PIDControl(elbow, 3e-3, 0.0, 0.0);

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

        SHOULDER_GRAVITY_FACTOR = 0.0;
    }

    /**
     * Drives the elbow joint to an absolute angle position.
     *
     * @param speed the maximum power to apply.
     * @param elbowAngle The target angle.
     */
    @Override
    public void elbowDriveAbsolute(double speed,
                                   double elbowAngle) {
        elbowPID.setTargetAngle(elbowAngle);
        elbowPID.maxPower = speed;
    }

    /**
     * Drives the shoulder joint to an absolute angle position.
     *
     * @param speed the maximum power to apply.
     * @param shoulderAngle The target angle.
     */
    @Override
    public void shoulderDriveAbsolute(double speed,
                                      double shoulderAngle) {
        shoulderPID.setTargetAngle(shoulderAngle);
        shoulderPID.maxPower = speed;
    }

    /**
     * Gets the current elbow joint target angle.
     *
      * @return angle in degrees
     */
    @Override
    public double getElbowTargetAngle() {
        return elbowPID.getTargetAngle();
    }

    /**
     * Gets the current shoulder joint target angle.
     *
     * @return angle in degrees
     */
    @Override
    public double getShoulderTargetAngle() {
        return shoulderPID.getTargetAngle();
    }

    /**
     * This function updates the PID control based on the new encoder values for each joint by
     * calling the joint's update() command.
     *
     */
    @Override
    public void update() {
        shoulderPID.update();
        elbowPID.update();
    }

    @Override
    public boolean isBusy() {
        // Might want to have this check the shoulderPID and elbowPID's isBusy functions.
        return true; // shoulder.isBusy() || elbow.isBusy();
    }

    @Override
    public void scaleShoulderPid(double v) {
        shoulderPID.p *= v;
        shoulderPID.i *= v;
        shoulderPID.d *= v;
    }

    @Override
    public void showPIDs(Telemetry telemetry) {
        telemetry.addData("shoulderPID motor:", shoulderPID.motor.toString());

        telemetry.addData("shoulder p:", String.format("%.4e", shoulderPID.p));
        telemetry.addData("shoulder i:", String.format("%.4e", shoulderPID.i));
        telemetry.addData("shoulder d:", String.format("%.4e", shoulderPID.d));
        telemetry.addData("shoulder pos p:", String.format("%.4e", shoulderPID.p));
        telemetry.addData("shoulder pos i:", String.format("%.4e", shoulderPID.i));
        telemetry.addData("shoulder pos d:", String.format("%.4e", shoulderPID.d));
        telemetry.addData("shoulder vel:", String.format("%.4e", shoulder.getVelocity()));
    }
}                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       //Hi. You found me. -SECRET COMMENT