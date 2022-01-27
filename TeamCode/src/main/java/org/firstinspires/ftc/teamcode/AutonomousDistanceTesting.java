/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="AutonomousDistanceTesting", group="3311")
public class AutonomousDistanceTesting extends OpMode
{
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    public DcMotor leftDriveFront = null;
    public DcMotor rightDriveFront = null;
    public DcMotor leftDriveBack = null;
    public DcMotor rightDriveBack = null;
    /* Setup a variable for each org.firstinspires.ftc.teamcode.drive wheel to save power level for telemetry */
    public double leftPowerFront    = 1.0;
    public double rightPowerFront   = 1.0;
    public double rightPowerBack    = 1.0;
    public double leftPowerBack     = 1.0;
    public double drive             = 0.0;
    public double turn              = 0.0;
    public double strafe            = 0.0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        leftDriveFront  = hardwareMap.get(DcMotor.class, "lf");
        rightDriveFront = hardwareMap.get(DcMotor.class, "rf");
        leftDriveBack  = hardwareMap.get(DcMotor.class, "lb");
        rightDriveBack = hardwareMap.get(DcMotor.class, "rb");



        // Set Motor Direction
        leftDriveFront.setDirection(DcMotor.Direction.FORWARD);
        rightDriveFront.setDirection(DcMotor.Direction.REVERSE);
        leftDriveBack.setDirection(DcMotor.Direction.FORWARD);
        rightDriveBack.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        leftDriveFront.setPower(1);
        rightDriveFront.setPower(1);
        leftDriveBack.setPower(1);
        rightDriveBack.setPower(1);

        leftDriveBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Run Without Encoders
        leftDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Brake when power set to Zero
        leftDriveFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDriveBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDriveFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDriveBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

        runtime.reset();


    }
    // *
    //
    //
    //
    //
    //
    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {

        /*
         Choose to org.firstinspires.ftc.teamcode.drive using either Tank Mode, or POV Mode
         Comment out the method that's not used.  The default below is POV.
         POV Mode uses left stick to go forward, and right stick to turn.
         - This uses basic math to combine motions and is easier to org.firstinspires.ftc.teamcode.drive straight.
        */
        // Send calculated power to wheels

        // The code below allows you to

        // Get the gamepad control values for this loop iteration

        drive = -gamepad1.left_stick_y;
        turn = gamepad1.left_stick_x;
        strafe = gamepad1.right_stick_x;

        // Calculate the powers for each of the wheels
        leftPowerFront = (drive + turn + strafe);
        rightPowerFront = (drive - turn - strafe);
        leftPowerBack = (drive + turn - strafe);
        rightPowerBack = (drive - turn + strafe);
        // Send calculated power to wheels


        if (gamepad1.a) {
            leftPowerFront = leftPowerFront * 0.5;
            rightPowerFront = rightPowerFront * 0.5;
            leftPowerBack = leftPowerBack * 0.5;
            rightPowerBack = rightPowerBack * 0.5;


            telemetry.addData("ChillMode", "activated");
        } else {
            telemetry.addData("ChillMode", "deactivated");

        }
        //send power to wheels
        leftDriveFront.setPower(leftPowerFront);
        rightDriveFront.setPower(rightPowerFront);
        leftDriveBack.setPower(leftPowerBack);
        rightDriveBack.setPower(rightPowerBack);

        if (gamepad1.x)
            telemetry.addData("Encoder_value:","Running to %7d :%7d %7d :%7d",
                    leftDriveFront.getCurrentPosition(),
                    rightDriveFront.getCurrentPosition(),
                    leftDriveBack.getCurrentPosition(),
                    rightDriveBack.getCurrentPosition());
    }


    /*
     * Code to run ONCE after the driver hits STOP
     */

    @Override
    public void stop() {
    }

}
