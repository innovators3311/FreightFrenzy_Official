package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

public class Arm_Activate {
// Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    Arm_Setup Arm = new Arm_Setup();
    public void init () {

        Arm.init(hardwareMap);
    }

    public void init_loop() {
        double frontPower = 0;
        double backPower = 0;
        double clawPower = 0;

        frontPower = gamepad2.right_stick_y;
        backPower = gamepad2.left_stick_y;

        if(gamepad2.a){
            clawPower = 1;
        }
        if(gamepad2.b){
            clawPower = -1;
        }

        Arm.fa.setPower(frontPower);
        Arm.ba.setPower(backPower);
        Arm.cl.setPower(clawPower);
    }

    public void start() {
        runtime.reset();
    }

    public void loop() {

    }

    public void stop() {
    }
}
                                                                                                                                                                                                                                                                                                                                                                                    //Hi. You found me. -SECRET COMMENT