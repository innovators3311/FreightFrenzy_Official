package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Arm_Setup {

    public DcMotor fa = null;
    public DcMotor ba = null;
    public DcMotor cl = null;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();
    /* Constructor */
    public Arm_Setup() {
    }

        /* Initialize standard Hardware interfaces */
        public void init (HardwareMap ahwMap){
            // Save reference to Hardware map
            hwMap = ahwMap;

            // Define and Initialize Motors
            fa = hwMap.get(DcMotor.class, "fa");
            ba = hwMap.get(DcMotor.class, "ba");
            cl = hwMap.get(DcMotor.class, "cl");

            // set motor direction
            fa.setDirection(DcMotor.Direction.FORWARD);
            ba.setDirection(DcMotor.Direction.FORWARD);
            cl.setDirection(DcMotor.Direction.FORWARD);

            // Set all motors to zero power
            fa.setPower(0);
            ba.setPower(0);
            cl.setPower(0);


            // Set all motors to run without encoders.
            // May want to use RUN_USING_ENCODERS if encoders are installed.
            fa.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ba.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            cl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            //brake all of the motors
            fa.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            ba.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            cl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        }
}