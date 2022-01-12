package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.PIDControl;


public class Arm2_Control {

    private ElapsedTime runtime = new ElapsedTime();

    //create variables for motors/servos
    public DcMotor elbow = null;
    public DcMotorEx shoulder = null;
    public Servo cl = null;
    public Servo mag = null;
    public PIDControl sPid = null;

    public PIDFCoefficients shoulderPIDF;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();
    private Object Servo;

    private double ELBOW_COUNTS_PER_DEGREE = 8192.0 / 360.0;
    private double SHOULDER_COUNTS_PER_DEGREE = 8192.0 / 360.0;

//    How much power to add when the shoulder arm is horizontal.

//    private double SHOULDER_GRAVITY_FACTOR = -0.05;
    private double SHOULDER_GRAVITY_FACTOR = 0;

    public void init(HardwareMap ahwMap) {

        // Save reference to Hardware map
        hwMap = ahwMap;

        //declare motors so that they can be used
        elbow = hwMap.get(DcMotor.class, "elbow");
        shoulder = hwMap.get(DcMotorEx.class, "shoulder");
//        sPid = new PIDControl(shoulder, 1.0, 0.0, 0.0);

        shoulderPIDF    = shoulder.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        shoulderPIDF.p  = -1;
        shoulderPIDF.i *= -0.1; // -1e-4;
        shoulderPIDF.d *= -0.1 ; // -1e-4;

        //        shoulder.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, shoulderPIDF);
        shoulder.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shoulderPIDF);

        //  cl = hwMap.get(Servo.class, "cl");
        //  mag = hwMap.get(Servo.class, "mag");


        //set the direction that the motors will turn
        elbow.setDirection(DcMotor.Direction.FORWARD);
        shoulder.setDirection(DcMotor.Direction.FORWARD);
        //you can not set direction on a servo

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

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void armDriveAbsolute(double speed,
                                 double shoulderAngle, double elbowAngle) {
        int newElbowTarget;
        int newShoulderTarget;

        // Determine new target position, and pass to motor controller
        newElbowTarget = (int) (elbowAngle * ELBOW_COUNTS_PER_DEGREE);
        newShoulderTarget = (int) (shoulderAngle * SHOULDER_COUNTS_PER_DEGREE);

        elbow.setTargetPosition(newElbowTarget);
        shoulder.setTargetPosition(newShoulderTarget);

        // Turn On RUN_TO_POSITION
        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        elbow.setPower(Math.abs(speed));
        shoulder.setPower(Math.abs(speed));
    }
    public void elbowArmDriveAbsolute(double speed,
                                 double elbowAngle) {
        int newElbowTarget;
        newElbowTarget = (int) (elbowAngle * ELBOW_COUNTS_PER_DEGREE);
        elbow.setTargetPosition(newElbowTarget);
        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        elbow.setPower(Math.abs(speed));
    }
    public void shoulderArmDriveAbsolute(double speed,
                                      double shoulderAngle) {
        int newShoulderTarget;
        newShoulderTarget = (int) (shoulderAngle * ELBOW_COUNTS_PER_DEGREE);
        shoulder.setTargetPosition(newShoulderTarget);
        shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        shoulder.setPower(Math.abs(speed));
//        int newShoulderTarget;
         newShoulderTarget = (int) (shoulderAngle * ELBOW_COUNTS_PER_DEGREE);
//        sPid.setTargetAngle(shoulderAngle);
    }

    public void update(){
        sPid.update();
    }

    public double getShoulderGravityVector() {
        return Math.cos(getShoulderAngle() * Math.PI / 360.0);
    }

    public double getElbowAngle() {
        return elbow.getCurrentPosition() / ELBOW_COUNTS_PER_DEGREE;
    }
    public double getElbowTargetAngle(){
        return elbow.getTargetPosition()/ELBOW_COUNTS_PER_DEGREE;
    }

    public double getShoulderAngle() {
        return shoulder.getCurrentPosition() / ELBOW_COUNTS_PER_DEGREE;
    }
    public double getShoulderTargetAngle(){
      return shoulder.getTargetPosition()/ELBOW_COUNTS_PER_DEGREE;
    }

    public void elbowMovePower(double power) {
        elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elbow.setPower(power);

// (: (:
    }

    public void shoulderMovePower(double power) {
        shoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        power += getShoulderGravityVector() * SHOULDER_GRAVITY_FACTOR;
        shoulder.setPower(power);

    }

    public void TopTierShippingHub() {
        armDriveAbsolute(.05, -268, 240);

    }

    public void MiddleTierShippingHub() {
        armDriveAbsolute(.25, 0, 0);

    }

    public void BottomTierShippingHub() {
        armDriveAbsolute(.25, 0, 0);
    }

    public void ShippingHubTopper() {
        armDriveAbsolute(.25, 0, 0);
        while (shoulder.isBusy() || elbow.isBusy()) {

        }

        armDriveAbsolute(.25, 0, 0);
    }

    public boolean isBusy() {
        return shoulder.isBusy() || elbow.isBusy();
    }

    public boolean waitForDone(double timeout) {
        runtime.reset();
        while (isBusy()) {
            if (runtime.seconds() > timeout) {
                return false;
            }
        }
        return true;
    }

    public boolean waitForDone() {
        while (isBusy()) {
        }
        return true;
    }

    public void emergencyStop() {
        shoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elbow.setPower(0);
        shoulder.setPower(0);

    }

}                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       //Hi. You found me. -SECRET COMMENT