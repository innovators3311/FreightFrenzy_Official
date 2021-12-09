package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

public class HardwareInnoBot2
{
    public DcMotor lf = null;
    public DcMotor rf = null;
    public DcMotor rb = null;
    public DcMotor lb = null;

    public CompassSensor compass = null;
    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public HardwareInnoBot2() {

    }
        /* Initialize standard Hardware interfaces */
        public void init (HardwareMap ahwMap){
            // Save reference to Hardware map
            hwMap = ahwMap;
            compass = hwMap.get(CompassSensor.class, "imu");

            // Define and Initialize Motors
            lf = hwMap.get(DcMotor.class, "lf");
            rf = hwMap.get(DcMotor.class, "rf");
            rb = hwMap.get(DcMotor.class, "rb");
            lb = hwMap.get(DcMotor.class, "lb");

            // set motor direction
            lf.setDirection(DcMotor.Direction.REVERSE);
            rf.setDirection(DcMotor.Direction.FORWARD);
            rb.setDirection(DcMotor.Direction.FORWARD);
            lb.setDirection(DcMotor.Direction.REVERSE);

            // Set all motors to zero power
            lf.setPower(0);
            rf.setPower(0);
            rb.setPower(0);
            lb.setPower(0);

            // Set all motors to run without encoders.
            // May want to use RUN_USING_ENCODERS if encoders are installed.
            lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            //brake all of the motors
            lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        }
}