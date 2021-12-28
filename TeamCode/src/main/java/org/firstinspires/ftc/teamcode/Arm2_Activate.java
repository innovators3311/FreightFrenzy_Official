package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Arm2_Activate", group="OpMode")
public class Arm2_Activate extends OpMode {
        private ElapsedTime runtime = new ElapsedTime();
        Arm2_Setup Arm = new Arm2_Setup();
        //Initialize everything necessary for TeleOp
        public void init () {

            Arm.init(hardwareMap);
        }

        public void init_loop() {
            runtime.reset();
        }

        public void start() {

        }

        public void loop() {
            //Setup and reset variables for the arm
            double frontPower = 0;
            double backPower = 0;

            //Motors
            if(gamepad2.left_stick_y > 0.8) {
                frontPower = 0.4;
            }
            if(gamepad2.left_stick_y < -0.8) {
                frontPower = -0.3;
            }
            if(gamepad2.right_stick_y > 0.8) {
                backPower = -0.4;
            }
            if(gamepad2.right_stick_y < -0.8) {
                backPower = 0.3;
            }

            //Servos
            if(gamepad2.a){
                Arm.cl.setPosition(1);
            }
            if(gamepad2.b){
                Arm.cl.setPosition(0);
            }
            if(gamepad2.x){
                Arm.mag.setPosition(1);
            }
            if(gamepad2.y){
                Arm.mag.setPosition(0);
            }

            //give the power from the variables through the motors
            Arm.fa.setPower(frontPower);
            Arm.ba.setPower(backPower);

            //tell the driver how fast the motor is traveling
            telemetry.addData("back arm power", backPower);
            telemetry.addData("front arm power", frontPower);
        }

        public void stop() {
        }
    }
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            //hi. you found me. -SECRET COMMENT