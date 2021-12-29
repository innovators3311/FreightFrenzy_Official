package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Arm2_Setup {
    //create variables for motors/servos
    public DcMotor fa = null;
    public DcMotor ba = null;
    public Servo cl = null;
    public Servo mag = null;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();
    private Object Servo;

    /* Constructor */
    public Arm2_Setup() {
    }

    /* Initialize standard Hardware interfaces */
    public void init (HardwareMap ahwMap){
        // Save reference to Hardware map
        hwMap = ahwMap;

        //declare motors so that they can be used
        fa = hwMap.get(DcMotor.class, "fa");
        ba = hwMap.get(DcMotor.class, "ba");
        cl = hwMap.get(Servo.class, "cl");
        mag = hwMap.get(Servo.class, "mag");

        //set the direction that the motors will turn
        fa.setDirection(DcMotor.Direction.FORWARD);
        ba.setDirection(DcMotor.Direction.FORWARD);
        //you can not set direction on a servo

        //make sure motors are at zero power
        fa.setPower(0);
        ba.setPower(0);

        //make sure that it will run without encoders so that they will operate correctly
        fa.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ba.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fa.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ba.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //brake the motors
        fa.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ba.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}