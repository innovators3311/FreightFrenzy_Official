package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Arm2_Control {

    private ElapsedTime runtime = new ElapsedTime();

    //create variables for motors/servos
    public DcMotor elbow = null;
    public DcMotor shoulder = null;
    public Servo cl = null;
    public Servo mag = null;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();
    private Object Servo;

    private double ELBOW_COUNTS_PER_DEGREE    = 4096.0/360.0;
    private double SHOULDER_COUNTS_PER_DEGREE = 4096.0/360.0;
    public void init(HardwareMap ahwMap) {

        // Save reference to Hardware map
        hwMap = ahwMap;

        //declare motors so that they can be used
        elbow = hwMap.get(DcMotor.class, "fa");
        shoulder = hwMap.get(DcMotor.class, "ba");
        cl = hwMap.get(Servo.class, "cl");
        mag = hwMap.get(Servo.class, "mag");

        //set the direction that the motors will turn
        elbow.setDirection(DcMotor.Direction.FORWARD);
        shoulder.setDirection(DcMotor.Direction.FORWARD);
        //you can not set direction on a servo

        //make sure motors are at zero power
        elbow.setPower(0);
        shoulder.setPower(0);


        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //brake the motors
        elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shoulder.setTargetPosition(0);
        elbow.setTargetPosition(0);
        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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

    public void elbowMoveRelative(double power) {
        elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elbow.setPower(power);

// (:
    }

    public void shoulderMoveRelative(double encoderChange) {
        shoulder.setTargetPosition(shoulder.getCurrentPosition() +
                (int) (encoderChange * SHOULDER_COUNTS_PER_DEGREE));
        shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulder.setPower(.15);

    }

    public void TopTierShippingHub() {
        armDriveAbsolute(.25, 0, 0);

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

    public boolean waitForDone( ) {
        while (isBusy()) { }
        return true;
    }


}                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       //