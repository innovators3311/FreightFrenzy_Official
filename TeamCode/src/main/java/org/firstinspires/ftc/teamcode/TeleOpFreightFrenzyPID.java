package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "TeleOpFreightFrenzyPID", group = "3311")
public class TeleOpFreightFrenzyPID extends TeleOpFreightFrenzy {
    // Arm setup
    public boolean PIDEnabled = true;
    protected boolean gamepadInput = false;

    // These values affect joystick sensitivity of the arm.
    public double ELBOW_MANUAL_FACTOR    =3.0;
    public double SHOULDER_MANUAL_FACTOR =3.0;
    public boolean armLevelDebounceUp = false;
    public boolean armLevelDebounceDown = false;
    public double shoulderTargetDegrees = 0.0;

    // This is a Java Array. It's stored as (shoulder, elbow) values for each position.
    public double[][] ARM_POSITIONS = {
            {0,0}, // position 0: Hard reset
            {30,-30}, // position 1: Carrying state
            {221,-180}, // position 2: Ground pickup
            {218,-231}, //position 3: Middle tier
            {125,-110}, // position 4: Top (needs tuning. bit to low)
            {79,137},// position 5: Top-er
            {79,-205},//position 7; cap
            {-114,113}
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
    public void init_loop(){
        telemetry.addData("Shoulder Button Pressed", arm.shoulderLimitSwitch.isPressed());
        telemetry.addData("Arm Initialized", arm.armInitalized);
        telemetry.addData("Arm State", arm.armState);
        telemetry.addData("shoulderPID target:", arm.shoulderPID.getTargetAngle());
        telemetry.addData("shoulderPID angle:", arm.getShoulderAngle());
        telemetry.addData("elbowPID target:", arm.elbowPID.getTargetAngle());
//        telemetry.addData("shoulder Power:", arm.shoulder.getPower());
//        telemetry.addData("elbow Power:", arm.elbow.getPower());
        arm.arm_reset();
        if (arm.armInitalized){
            gamepad1.rumble(1000);
            gamepad2.rumble(1000);
        }

    }

    @Override
    protected void handleArm() {
        double elbowTargetDegrees   = 0.0;
        shoulderTargetDegrees = 0.0;
        double distanceFactor = 1;
        double backup = 0;
        if(armLevel == 6){
            if(distanceSensor.getDistance(DistanceUnit.CM) < 4){
                if(distanceSensor.getDistance(DistanceUnit.CM) < 1.5){
                    distanceFactor = 0;
                    backup = -0.25 * distanceSensor.getDistance(DistanceUnit.CM);
                }else{
                    distanceFactor = 0.2;
                }
            }
        }

        if (Math.abs(gamepad2.left_stick_y) > .05 || Math.abs(gamepad2.right_stick_y) > .05
                || Math.abs(gamepad2.left_stick_x) > .05 || Math.abs(gamepad2.right_stick_x) > .05) {
        gamepadInput = true;

            // If driver moves the stick more than 10% or we're already in run without encoder mode ...
            elbowTargetDegrees += gamepad2.left_stick_y * ELBOW_MANUAL_FACTOR;
            shoulderTargetDegrees += gamepad2.right_stick_y * SHOULDER_MANUAL_FACTOR;
            elbowTargetDegrees -= -gamepad2.right_stick_x * ELBOW_MANUAL_FACTOR;
            shoulderTargetDegrees += -gamepad2.right_stick_x * SHOULDER_MANUAL_FACTOR;

            arm.shoulderDriveAbsolute(1, arm.getShoulderTargetAngle() + shoulderTargetDegrees);
            arm.elbowDriveAbsolute(1, arm.getElbowTargetAngle() + elbowTargetDegrees);
            PIDEnabled = true;
        }

        telemetry.addData("shoulder current angle:", arm.getShoulderAngle());
        telemetry.addData("shoulder angle target:", arm.getShoulderTargetAngle());
        telemetry.addData("gravity vector", arm.getShoulderGravityVector());
//         Publish Elbow values
        telemetry.addData("elbow Current angle:", arm.getElbowAngle());
        telemetry.addData("elbow angle target:", arm.getElbowTargetAngle());
//        telemetry.addData("is busy?", arm.isBusy());
        telemetry.addData("Arm Level", armLevel);
        telemetry.addData("blue",colorSensor2.blue());
        telemetry.addData("green",colorSensor2.green());
        telemetry.addData("red",colorSensor2.red());
        colorSensor2.enableLed(false);

        arm.shoulderPID.chillFactor = -0.75 * Math.max(gamepad2.left_trigger , gamepad2.right_trigger) + 1;
        arm.elbowPID.chillFactor    = -0.9 * Math.max(gamepad2.left_trigger , gamepad2.right_trigger) + 1;

        if (PIDEnabled) {
            arm.update();
            gamepadInput = true;
        }
        if(arm.getShoulderAngle() < 5 && arm.getElbowAngle() < 5 && !gamepadInput) {
            arm.shoulder.setPower(0);
            arm.elbow.setPower(0);
        }
    }
    public void handleFixedPos(){
        if(gamepad2.right_bumper = true) {
            armLevel = 7;
        }
        if(gamepad2.y){
            armLevel = 6;
        }
        if(gamepad2.b) {
            armLevel = 2;
        }
        if (gamepad2.dpad_up) {

            if (!armLevelDebounceUp) {
                armLevel = armLevel + 1;
                gamepadInput = true;
            }
            armLevelDebounceUp = true;

        }    else{
            armLevelDebounceUp = false;
            gamepadInput = false;
        }
        if (gamepad2.dpad_down) {
            if (!armLevelDebounceDown) {
                armLevel = armLevel - 1;
                gamepadInput = true;
            }
            armLevelDebounceDown = true;
            gamepadInput = true;

        }    else{
            gamepadInput = false;
            armLevelDebounceDown = false;
        }

        if(armLevel < 0){
            armLevel = 0;
        }
        if(armLevel > 6){
            armLevel = 6;
        }

        int armPosLen = ARM_POSITIONS.length;
        if(armLevel > armPosLen-1){
            armLevel = armPosLen-1;
        }

        if(gamepad2.y || gamepad2.b || gamepad2.right_bumper) {
            arm.shoulderDriveAbsolute(1, ARM_POSITIONS[armLevel][0]);
            arm.elbowDriveAbsolute(1, ARM_POSITIONS[armLevel][1]);

        }
    }
      // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        gamepadInput = false;
        this.handleArm();
        this.handleIntake();
        this.handleSpinner();
        this.handleDriving();
        this.handleFixedPos();

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());

    }
}