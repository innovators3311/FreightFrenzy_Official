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

package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Arm2_Control;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "TeleOpFreightFrenzyArmTest", group = "3311")
public class TeleOpFreightFrenzyArmTest extends OpMode {
    //Bring in code to setup arm.
    Arm2_Control arm = new Arm2_Control();
    boolean stickControl = true;
    double elbowSpeed    = 0.0;
    double shoulderSpeed = 0.0;
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    PIDControl shoulder = null;
    PIDControl elbow    = null;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        arm.init(hardwareMap);
        this.shoulder = new PIDControl(arm.shoulder, .003, 0, 0 );
        this.elbow = new PIDControl(arm.elbow, .003, 0, 0 );
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

    private void handleClaw() {
        if (gamepad2.a) {
            arm.cl.setPosition(1);
        }
        if (gamepad2.b) {
            arm.cl.setPosition(0);
        }
        if (gamepad2.x) {
            arm.mag.setPosition(1);
        }
        if (gamepad2.y) {
            arm.mag.setPosition(0);
        }
    }


    private void handleArm() {
        // If driver moves the stick more than 10% or we're already in run without encoder mode ...
        if ((Math.abs(gamepad2.left_stick_y) > .10) || (stickControl)) {
            arm.elbowMovePower(gamepad2.left_stick_y * elbowSpeed);
            stickControl = true;
        }

        if ((Math.abs(gamepad2.right_stick_y) > .10) || (stickControl)) {
            arm.shoulderMovePower(gamepad2.right_stick_y * shoulderSpeed);
            stickControl = true;
        }
        if (gamepad2.right_bumper){
            elbowSpeed    = 0.5;
            shoulderSpeed = 0.7;
        }
        else {
            elbowSpeed    = 0.2;
            shoulderSpeed = 0.3;
        }


        telemetry.addData("shoulder current angle:", String.format("%.2f", arm.getShoulderAngle()));
        telemetry.addData("shoulder angle target:", String.format("%.2f", arm.getShoulderTargetAngle()));

        PIDFCoefficients shoulderEncoderPIDF    = arm.shoulder.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("shoulder p:",String.format("%.4e",  shoulderEncoderPIDF.p));
        telemetry.addData("shoulder i:", String.format("%.4e", shoulderEncoderPIDF.i));
        telemetry.addData("shoulder d:",String.format("%.4e",  shoulderEncoderPIDF.d));
        PIDFCoefficients shoulderPositionPIDF    = arm.shoulder.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData("shoulder pos p:",String.format("%.4e",  shoulderPositionPIDF.p));
        telemetry.addData("shoulder pos i:", String.format("%.4e", shoulderPositionPIDF.i));
        telemetry.addData("shoulder pos d:",String.format("%.4e",  shoulderPositionPIDF.d));
        telemetry.addData("shoulder vel:",String.format("%.4e",  arm.shoulder.getVelocity()));

        //        telemetry.addData("shoulder p:", shoulder.p);
        // Publish Elbow values
        telemetry.addData("elbow Current angle:", String.format("%.2f", arm.getElbowAngle()));
        telemetry.addData("elbow angle target:", String.format("%.2f", arm.getElbowTargetAngle()));
        telemetry.addData("Commanded shoulder Power", String.format("%.2f", arm.shoulder.getPower()));

        telemetry.addData("is busy?", arm.isBusy());

        while (gamepad2.right_stick_button || gamepad2.left_stick_button) {
            arm.emergencyStop();
        }

        // puts the arm back to its beginning position
        if(gamepad2.dpad_up){
//            shoulder.setTargetAngle(360);
//            elbow.setTargetAngle(0);
            arm.shoulderArmDriveAbsolute( .25, 360);
            stickControl = false;

        }
        // puts the arm to position 1
        if(gamepad2.dpad_right){
//            shoulder.p *= 1.2;
            shoulderEncoderPIDF    = arm.shoulder.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            shoulderEncoderPIDF.p*= 1.2;
            shoulderEncoderPIDF.d *= 1.2;
            shoulderEncoderPIDF.i *= 1.2;
            arm.shoulder.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shoulderEncoderPIDF);
            stickControl = false;
        }
        // puts the arm back to its position 2
        if(gamepad2.dpad_left){
            shoulderEncoderPIDF    = arm.shoulder.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            shoulderEncoderPIDF.p /= 1.2;
            shoulderEncoderPIDF.d /= 1.2;
            shoulderEncoderPIDF.i /= 1.2;
            arm.shoulder.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shoulderEncoderPIDF);
            stickControl = false;
        }
        // puts the arm back to its position 3
        if(gamepad2.dpad_down){
            arm.shoulderArmDriveAbsolute( .25, -360);

//            shoulder.setTargetAngle(0);
//            elbow.setTargetAngle(0);
            stickControl = false;
        }


//        shoulder.update();
    }

    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        handleArm();
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString()); }

    /*
      Code to run ONCE after the driver hits STOP
     Code to run ONCE after the driver hits STOP
     */

    @Override
    public void stop() {
    }

}
//If you found this then you are a big brain gamer