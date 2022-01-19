package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOpFreightFrenzyPID", group = "3311")
public class TeleOpFreightFrenzyPID extends TeleOpFreightFrenzy {
    // Arm setup
    public boolean PIDenabled = true;

    // These values affect joystick sensitivity of the arm.
    public double ELBOW_MANUAL_FACTOR=2.0;
    public double SHOULDER_MANUAL_FACTOR=2.0;
    public int armLevel = 1;
    public boolean armLevelDebounceUp = false;
    public boolean armLevelDebounceDown = false;

    public double[][] armPos = {
            {200,-173}, // position 0
            {30,-30}, // position 1
            {210,0}, // position 2
            {0,0}, // position 3
    };

    ArmPID_Control arm = new ArmPID_Control();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        super.init();
        this.arm.init(hardwareMap);
    }

    @Override
    protected void handleArm() {
        double elbowTargetDegrees   = 0.0;
        double shoulderTargetDegrees = 0.0;

        if (Math.abs(gamepad2.left_stick_y) > .05 || Math.abs(gamepad2.right_stick_y) > .05
                || Math.abs(gamepad2.left_stick_x) > .05 || Math.abs(gamepad2.right_stick_x) > .05) {

            // If driver moves the stick more than 10% or we're already in run without encoder mode ...
            elbowTargetDegrees += gamepad2.left_stick_y * ELBOW_MANUAL_FACTOR;
            shoulderTargetDegrees += gamepad2.right_stick_y * SHOULDER_MANUAL_FACTOR;
            elbowTargetDegrees -= -gamepad2.right_stick_x * ELBOW_MANUAL_FACTOR;
            shoulderTargetDegrees += -gamepad2.right_stick_x * SHOULDER_MANUAL_FACTOR;

            arm.shoulderDriveAbsolute(1, arm.getShoulderTargetAngle() + shoulderTargetDegrees);
            arm.elbowDriveAbsolute(1, arm.getElbowTargetAngle() + elbowTargetDegrees);
            PIDenabled = true;
        }

        //set arm to shared hub level
        if (gamepad2.dpad_right || gamepad2.dpad_left) {
            arm.shoulderDriveAbsolute(0.7, 162.8);
            arm.elbowDriveAbsolute(0.7, -144.3);
            PIDenabled = true;
        }
        //

        telemetry.addData("shoulder current angle:", arm.getShoulderAngle());
        telemetry.addData("shoulder angle target:", arm.getShoulderTargetAngle());
        telemetry.addData("gravity vector", arm.getShoulderGravityVector());
//         Publish Elbow values
        telemetry.addData("elbow Current angle:", arm.getElbowAngle());
        telemetry.addData("elbow angle target:", arm.getElbowTargetAngle());
        telemetry.addData("is busy?", arm.isBusy());

        while (gamepad2.right_stick_button || gamepad2.left_stick_button) {
            arm.emergencyStop();
            PIDenabled = false;
        }
        if (PIDenabled) {
            arm.update();
        }
    }
    public void handleFixedPos(){
        if(armLevel < 0){
            armLevel = 0;
        }

        if(armLevel > 3){
            armLevel = 3;
        }

        if (gamepad2.dpad_up) {

            if (!armLevelDebounceUp) {
                armLevel = armLevel + 1;
            }
            armLevelDebounceUp = true;

        }    else{
            armLevelDebounceUp = false;
        }
        if (gamepad2.dpad_down) {
            if (!armLevelDebounceDown) {
                armLevel = armLevel - 1;
            }
            armLevelDebounceDown = true;

        }    else{
            armLevelDebounceDown = false;
        }
        if(gamepad2.dpad_down || gamepad2.dpad_up){
            arm.shoulderDriveAbsolute(1, armPos[armLevel][0]);
            arm.elbowDriveAbsolute(1, armPos[armLevel][1]);

        }
    }
      // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        this.handleClaw();
        this.handleSpinner();
        this.handleDriving();
        this.handleFixedPos();
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        this.handleArm();
    }


}