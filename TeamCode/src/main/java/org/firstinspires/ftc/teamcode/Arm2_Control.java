package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.PIDControl;


public class Arm2_Control {

    //create variables for motors/servos
    public DcMotorEx elbow = null;
    public DcMotorEx shoulder = null;
    public Servo cl = null;
    public Servo mag = null;
    public PIDControl sPid = null;
    public PIDFCoefficients shoulderPIDF;
    protected ElapsedTime period = new ElapsedTime();
    protected Object Servo;
    protected double ELBOW_COUNTS_PER_DEGREE = 8192.0 / 360.0;
    protected double SHOULDER_COUNTS_PER_DEGREE = 8192.0 / 360.0;

    protected double SHOULDER_GRAVITY_FACTOR = -0.1;
    //  0  protected double SHOULDER_GRAVITY_FACTOR = 0;
    ElapsedTime runtime = new ElapsedTime();

    //    How much power to add when the shoulder arm is horizontal.
    /* local OpMode members. */
    HardwareMap hwMap = null;

    /**
     * Initialize hardware and prepare to run TeleOp
     *
     * @param ahwMap a hardware map
     */
    public void init(HardwareMap ahwMap) {

        // Save reference to Hardware map
        hwMap = ahwMap;

        //declare motors so that they can be used
        elbow = hwMap.get(DcMotorEx.class, "elbow");
        shoulder = hwMap.get(DcMotorEx.class, "shoulder");

        shoulderPIDF = shoulder.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        shoulderPIDF.p = -0.0001;
        shoulderPIDF.i *= -0.1; // -1e-4;
        shoulderPIDF.d *= -0.1; // -1e-4;

        //        shoulder.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, shoulderPIDF);
        shoulder.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shoulderPIDF);

        cl = hwMap.get(Servo.class, "claw");
        mag = hwMap.get(Servo.class, "mag");


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
        elbowDriveAbsolute(speed * 1, elbowAngle * 1);
        shoulderDriveAbsolute(speed * 1.2, shoulderAngle * 1);
    }

    public void elbowDriveAbsolute(double speed,
                                   double elbowAngle) {
        int newElbowTarget;
        newElbowTarget = (int) (elbowAngle * ELBOW_COUNTS_PER_DEGREE);

        elbow.setTargetPosition(newElbowTarget);
        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        elbow.setPower(Math.abs(speed));
    }

    public void shoulderDriveAbsolute(double speed,
                                      double shoulderAngle) {
        int newShoulderTarget = (int) (shoulderAngle * ELBOW_COUNTS_PER_DEGREE);
        shoulder.setTargetPosition(newShoulderTarget);
        shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        shoulder.setPower(Math.abs(speed));
    }

    public void update() {
        // For use in the pid controllers.
    }

    public double getShoulderGravityVector() {
        return Math.cos(getShoulderAngle() * Math.PI / 180.0);
    }

    public double getElbowAngle() {
        return elbow.getCurrentPosition() / ELBOW_COUNTS_PER_DEGREE;
    }

    public double getElbowTargetAngle() {
        return elbow.getTargetPosition() / ELBOW_COUNTS_PER_DEGREE;
    }

    public double getShoulderAngle() {
        return shoulder.getCurrentPosition() / SHOULDER_COUNTS_PER_DEGREE;
    }

    public double getShoulderTargetAngle() {
        return shoulder.getTargetPosition() / SHOULDER_COUNTS_PER_DEGREE;
    }

    public void elbowMovePower(double power) {
//        elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elbow.setPower(power);

// (: (:
    }

    public void shoulderMovePower(double power) {
//        shoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

    public void showPIDs(Telemetry telemetry) {
        PIDFCoefficients shoulderEncoderPIDF = shoulder.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("shoulder p:", String.format("%.4e", shoulderEncoderPIDF.p));
        telemetry.addData("shoulder i:", String.format("%.4e", shoulderEncoderPIDF.i));
        telemetry.addData("shoulder d:", String.format("%.4e", shoulderEncoderPIDF.d));
        PIDFCoefficients shoulderPositionPIDF = shoulder.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData("shoulder pos p:", String.format("%.4e", shoulderPositionPIDF.p));
        telemetry.addData("shoulder pos i:", String.format("%.4e", shoulderPositionPIDF.i));
        telemetry.addData("shoulder pos d:", String.format("%.4e", shoulderPositionPIDF.d));
        telemetry.addData("shoulder vel:", String.format("%.4e", shoulder.getVelocity()));
    }

    public void scaleShoulderPid(double v) {
        PIDFCoefficients shoulderEncoderPIDF = shoulder.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        shoulderEncoderPIDF.p *= v;
        shoulderEncoderPIDF.d *= v;
        shoulderEncoderPIDF.i *= v;
        shoulder.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shoulderEncoderPIDF);
    }
}                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       //Hi. You found me. -SECRET COMMENT