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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Test OpMode_other", group="Linear Opmode")
public class BasicTeleop extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor leftDriveBack = null;
    private DcMotor rightDriveBack = null;
    private DcMotor mainArm = null;
    private DcMotor secondaryArm = null;
    private DcMotor spinner = null;
    private Servo claw = null;
    private Servo release = null;

    double max;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "lf");
        rightDrive = hardwareMap.get(DcMotor.class, "rf");
        leftDriveBack  = hardwareMap.get(DcMotor.class, "lb");
        rightDriveBack = hardwareMap.get(DcMotor.class, "rb");

        mainArm = hardwareMap.get(DcMotor.class, "shoulder");
        secondaryArm = hardwareMap.get(DcMotor.class, "elbow");
        spinner = hardwareMap.get(DcMotor.class, "spinner");

        claw  = hardwareMap.get(Servo.class, "claw");
        release = hardwareMap.get(Servo.class, "mag");

        // Most robots need the motor on one side to be reversed to org.firstinspires.ftc.teamcode.drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDriveBack.setDirection(DcMotor.Direction.FORWARD);
        rightDriveBack.setDirection(DcMotor.Direction.REVERSE);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDriveBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDriveBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        mainArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        secondaryArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each org.firstinspires.ftc.teamcode.drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;
            double leftBackPower;
            double rightBackPower;

            double armPower = gamepad2.left_stick_y;
            double secondaryArmPower = gamepad2.right_stick_y;

            // Choose to org.firstinspires.ftc.teamcode.drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to org.firstinspires.ftc.teamcode.drive straight.

            double turn = gamepad1.left_stick_x;
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.right_stick_x;

            if (Math.abs(armPower) < 0.05) {
                armPower = 0;
            }
            if (Math.abs(secondaryArmPower) < 0.05) {
                secondaryArmPower = 0;
            }

            //claw code
            if (gamepad2.a) {
                claw.setPosition(0);
            }
            if (gamepad2.b) {
                claw.setPosition(1);
            }
            if (gamepad2.x) {
                release.setPosition(1);
            }
            if (gamepad2.y) {
                release.setPosition(0);
            }
            if (gamepad2.right_bumper) {
                spinner.setPower(1);
            } else {
                spinner.setPower(0);
            }

            leftPower = drive + turn + strafe;
            rightPower = drive - turn - strafe;
            leftBackPower = drive - turn + strafe;
            rightBackPower = drive + turn - strafe;

            if (Math.abs(leftPower) > 1 || Math.abs(leftBackPower) > 1 || Math.abs(rightPower) > 1 || Math.abs(rightBackPower) > 1) {
                max = Math.max(Math.abs(leftPower), Math.abs(leftBackPower));
                max = Math.max(Math.abs(rightPower), max);
                max = Math.max(Math.abs(rightBackPower), max);

                //divide everything by highest power to keep proper ratios of strafe, org.firstinspires.ftc.teamcode.drive, and turn
                leftPower /= max;
                rightPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // Send calculated power to wheels
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);
            leftDriveBack.setPower(leftBackPower);
            rightDriveBack.setPower(rightBackPower);
            mainArm.setPower(armPower);
            secondaryArm.setPower(secondaryArmPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "arm1 (%.2f), arm2 (%.2f)", armPower, secondaryArmPower);
            telemetry.update();
        }
    }
}
