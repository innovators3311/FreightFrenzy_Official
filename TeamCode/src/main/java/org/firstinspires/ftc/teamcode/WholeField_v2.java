package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.SwitchableCamera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import org.firstinspires.ftc.teamcode.source.Arm;

@Disabled
@Autonomous(name = "Whole Field_v2", group = "Whole Field")
public class WholeField_v2 extends LinearOpMode {

    /* Note: This sample uses the all-objects Tensor Flow model (FreightFrenzy_BCDM.tflite), which contains
     * the following 4 detectable objects
     *  0: Ball,
     *  1: Cube,
     *  2: Duck,
     *  3: Marker (duck location tape marker)
     *
     *  Two additional model assets are available which only contain a subset of the objects:
     *  FreightFrenzy_BC.tflite  0: Ball,  1: Cube
     *  FreightFrenzy_DM.tflite  0: Duck,  1: Marker
     */
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };

    private static final String VUFORIA_KEY =
            "ATCNswP/////AAABmboo62E3M0RLvUoBrala8GQowW4hvn2lz0v4xIUqDcerBojdZbFDT7KxueF7R6JgJY9tQ+gV9sHXv6aOcnznTsupwlzsqujeV1pIN0j5/uZNqLkxZCORToVMVD/kd8XY5y58Pnml+lS3pqkZee6pSUTNWfmWgJAu/oKPGVrOm5GwCPObOM9Mx3NSbWeRVSiKcaN9o6QyqV+Knuf2xYpF87rKiH0pbWGRIFSy8JgVQ6dabuIoDCKbXpDeTwK3PJ2VtgON+8PA2TIIn95Yq8UmBYJRJc6kDyvCDyCnKJ63oPRfzth3P8DM4IchQd69ccU6vqeto4JNQbPZh5JB5KRXFS8CcmQJLkSRcHDIP92eIhv/";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * Variables used for switching cameras.
     */
    private WebcamName webcam1, webcam2;
    private SwitchableCamera switchableCamera;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    //defining program states
    enum mainState {
        TRAJECTORY_1,   // Head to the duck carousel
        TRAJECTORY_2,   // Creep forward several inches to position accurately
        SPIN,           // Spin the duck
        TRAJECTORY_3,   // Go to the shipping hub
        TRAJECTORY_4,   // Drive forward slowly a few inches to the hub
        DROP_1,         // Drop block
        WAIT_1,
        TRAJECTORY_5,   // Position to get some freight
        TRAJECTORY_6,
        TRAJECTORY_7,   // Get the freight
        TRAJECTORY_8,   // Get out of the warehouse
        TRAJECTORY_9,   // Go to shipping hub and place the freight
        DROP_2,         // Dropping freight in shipping hub for second time
        TRAJECTORY_10,   // Park in warehouse
        IDLE            // Enter IDLE state when complete
    }
    enum armState {
        TIER_1,
        TIER_2,
        TIER_3,
        PICKUP,
        ARM_RESET,
        IDLE
    }

    //defining the main state
    mainState currentState = mainState.IDLE;

    //defining the arm state
    armState currentArmState = armState.IDLE;

    //defining start pose
    Pose2d startPose = new Pose2d(30, -64, Math.toRadians(0));

    //initializing timer
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1, 16.0/9.0);
        }

        int Duck = 0;
        timer.reset();
        while(Duck == 0 && timer.seconds() < 0) {
            if (tfod != null) {
                List<Recognition> recognitions = tfod.getRecognitions();
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : recognitions) {
                    if(recognition.getLabel() == "Duck") {
                        Duck = DuckSpot(recognition.getRight()-((recognition.getRight()-recognition.getLeft())/2));
                    }
                    i++;
                }
            }
        }
        if(Duck == 0) { //If it took too long to find the duck then it defaults to spot 3
            Duck = 3;
        }
        telemetry.addData("Duck Spot", Duck);
        telemetry.update();

        //Initializing our other robot hardware
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Arm arm = new Arm(hardwareMap); //Shoulder start angle is 150 degrees, elbow is 20 relative to the shoulder

        //Setting initial position estimate
        drive.setPoseEstimate(startPose);

        //Defining trajectories
        Trajectory trajectory1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(49, -61), Math.toRadians(10)) //to duck carousel
                .build();
        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end()) //creeping up to carousel
                .splineTo(new Vector2d(57, -61), Math.toRadians(10))
                .build();
        Trajectory trajectory3_1 = drive.trajectoryBuilder(trajectory2.end()) //turning back of robot to face shipping hub
                .lineToLinearHeading(new Pose2d(52, -55, Math.toRadians(-35)))
                .build();
        Trajectory trajectory3_2 = drive.trajectoryBuilder(trajectory2.end()) //turning front of robot to face shipping hub
                .lineToLinearHeading(new Pose2d(52, -55, Math.toRadians(135)))
                .build();
        Trajectory trajectory4_1 = drive.trajectoryBuilder(trajectory3_1.end()) //*duck spot 1* positions for bottom tier
                .back(38)
                .build();
        Trajectory trajectory4_2 = drive.trajectoryBuilder(trajectory3_1.end()) //*duck spot 2* positions for middle tier
                .back(36)
                .build();
        Trajectory trajectory4_3 = drive.trajectoryBuilder(trajectory3_2.end()) //*duck spot 3* positions for top tier
                .forward(30)
                .build();
        //*trajectory5* position to get some freight
        Trajectory trajectory5_1 = drive.trajectoryBuilder(trajectory4_1.end())
                .splineTo(new Vector2d(20.87, -67), Math.toRadians(180))
                .build();
        Trajectory trajectory5_2 = drive.trajectoryBuilder(trajectory4_2.end())
                .splineTo(new Vector2d(-12, -67), Math.toRadians(180))
                .build();
        Trajectory trajectory5_3 = drive.trajectoryBuilder(trajectory4_3.end())
                .lineToLinearHeading(new Pose2d(20.87, -33.2, Math.toRadians(180)))
                .build();
        Trajectory trajectory6_1 = drive.trajectoryBuilder(trajectory5_3.end())
                .strafeLeft(37)
                .build();
        Trajectory trajectory6_2 = drive.trajectoryBuilder(trajectory5_3.end())
                .strafeLeft(37)
                .build();
        Trajectory trajectory6_3 = drive.trajectoryBuilder(trajectory5_3.end())
                .strafeLeft(37)
                .build();
        Trajectory trajectory7 = drive.trajectoryBuilder(trajectory6_3.end()) //going to get freight from warehouse
                .forward(40)
                .build();
        Trajectory trajectory8 = drive.trajectoryBuilder(trajectory7.end()) //positioning to drive to shipping hub
                .back(40)
                .build();
        Trajectory trajectory9 = drive.trajectoryBuilder(trajectory8.end()) //driving to shipping hub
                .splineTo(new Vector2d(-12, 45), Math.toRadians(45))
                .build();
        Trajectory trajectory10 = drive.trajectoryBuilder(trajectory9.end()) //parking in warehouse
                .splineTo(new Vector2d(-60, -40), Math.toRadians(180))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        currentState = mainState.TRAJECTORY_1;
        drive.followTrajectoryAsync(trajectory1);

        while (opModeIsActive() && !isStopRequested()) {
            //You can have multiple switch statements running together for multiple state machines
            //in parallel. This is the basic idea for subsystems and commands.

            //Essentially defining the flow of the state machine through this switch statement
            switch (currentState) {
                case TRAJECTORY_1: //driving to the duck carousel
                    if (!drive.isBusy()) {
                        currentState = mainState.TRAJECTORY_2;
                        drive.followTrajectoryAsync(trajectory2);
                    }
                    break;
                case TRAJECTORY_2: //going forward a little bit to mash duck wheel against carousel
                    if (!drive.isBusy()) {
                        timer.reset();
                        currentState = mainState.SPIN;
                    }
                    break;
                case SPIN: //kind of obvious
                    arm.spinner.setPower(1);
                    if (timer.milliseconds() > 2500) {
                        arm.spinner.setPower(0);
                        currentState = mainState.TRAJECTORY_3;
                        if(Duck == 3) {
                            drive.followTrajectoryAsync(trajectory3_2);
                        } else {
                            drive.followTrajectoryAsync(trajectory3_1);
                        }
                    }
                    break;
                case TRAJECTORY_3: //turning to go to shipping hub
                    if (!drive.isBusy()) {
                        currentState = mainState.TRAJECTORY_4;
                        if(Duck == 3) {
                            drive.followTrajectoryAsync(trajectory4_3);
                        } else {
                            drive.followTrajectoryAsync(trajectory4_1);
                        }
                        switch(Duck) {
                            case 1:
                                drive.followTrajectoryAsync(trajectory4_1);
                                currentArmState = armState.TIER_1;
                            case 2:
                                drive.followTrajectoryAsync(trajectory4_2);
                                currentArmState = armState.TIER_2;
                            case 3:
                                drive.followTrajectoryAsync(trajectory4_3);
                                currentArmState = armState.TIER_3;
                        }
                    }
                    break;
                case TRAJECTORY_4: //going to the shipping hub and dropping freight
                    if (!drive.isBusy() && !arm.shoulderIsBusy && !arm.elbowIsBusy) {
                        //arm.openClaw(750, 1);
                        //arm.retractMagnet(750, 1);
                        currentState = mainState.WAIT_1;
                        timer.reset();
                    }
                    break;
                case WAIT_1:
                    if(timer.milliseconds() > 800) {
                        currentState = mainState.TRAJECTORY_5;
                        switch(Duck) {
                            case 1:
                                drive.followTrajectoryAsync(trajectory5_1);
                            case 2:
                                drive.followTrajectoryAsync(trajectory5_2);
                            case 3:
                                drive.followTrajectoryAsync(trajectory5_3);
                        }
                    }
                case TRAJECTORY_5: //turning to strafe to wall
                    if (!drive.isBusy()) {
                        currentState = mainState.TRAJECTORY_6;
                        switch(Duck) {
                            case 1:
                                drive.followTrajectoryAsync(trajectory6_1);
                            case 2:
                                drive.followTrajectoryAsync(trajectory6_2);
                            case 3:
                                drive.followTrajectoryAsync(trajectory6_3);
                        }
                        currentArmState = armState.PICKUP;
                    }
                    break;
                case TRAJECTORY_6: //strafing to wall and lowering arm
                    if (!drive.isBusy() && !arm.shoulderIsBusy && !arm.elbowIsBusy) {
                        currentState = mainState.TRAJECTORY_7;
                        drive.followTrajectoryAsync(trajectory7);
                    }
                    break;
                case TRAJECTORY_7: //driving forward to get the freight
                    if (!drive.isBusy()) {
                        currentArmState = armState.PICKUP;
                        currentState = mainState.TRAJECTORY_8;
                        drive.followTrajectoryAsync(trajectory8);
                    }
                    break;
                case TRAJECTORY_8: //going out of the warehouse
                    if (!drive.isBusy()) {
                        currentArmState = armState.TIER_3;
                        currentState = mainState.TRAJECTORY_9;
                        drive.followTrajectoryAsync(trajectory9);
                    }
                    break;
                case TRAJECTORY_9: //driving to shipping hub
                    if(!drive.isBusy()) {
                        timer.reset();
                        currentState = mainState.DROP_2;
                    }
                case DROP_2: //dropping another block
                    if(timer.milliseconds() > 2000) {
                        currentArmState = armState.ARM_RESET;
                        currentState = mainState.IDLE;
                        drive.followTrajectoryAsync(trajectory10);
                    }
                case TRAJECTORY_10: //parking in warehouse
                    if(!drive.isBusy()) {
                        currentState = mainState.IDLE;
                    }
                case IDLE:
                    arm.storeArmPose();
                    break;
            }

            switch(currentArmState) {
                case TIER_1:
                    arm.runShoulderTo(153);
                    arm.runElbowTo(270);
                    currentArmState = armState.IDLE;
                    break;
                case TIER_2:
                    arm.runShoulderTo(120);
                    arm.runElbowTo(285);
                    currentArmState = armState.IDLE;
                    break;
                case TIER_3:
                    arm.runShoulderTo(50);
                    arm.runElbowTo(175);
                    currentArmState = armState.IDLE;
                    break;
                case PICKUP:
                    arm.runShoulderTo(-25);
                    arm.runElbowTo(205);
                    currentArmState = armState.IDLE;
                    break;
                case ARM_RESET:
                    arm.runShoulderTo(150);
                    arm.runElbowTo(20);
                    currentArmState = armState.IDLE;
                    break;
                case IDLE:
                    break;
            }

            //Updating all arm components continuously in the background, regardless of state
            arm.updateShoulder();
            arm.updateElbow();
            //arm.updateClaw();
            //arm.updateMagnet();

            //Updating drive continuously in the background, regardless of state
            drive.update();

            //Printing data to telemetry
            telemetry.addData("current state", currentState);
            telemetry.addData("Shoulder error", arm.shoulderErr);
            telemetry.addData("Elbow error", arm.elbowErr);
            telemetry.addData("1", trajectory4_1.end());
            telemetry.addData("2", trajectory4_1.end());
            telemetry.addData("3", trajectory4_1.end());
            telemetry.update();
        }
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        // Indicate that we wish to be able to switch cameras.
        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcam2 = hardwareMap.get(WebcamName.class, "Webcam 2");
        parameters.cameraName = ClassFactory.getInstance().getCameraManager().nameForSwitchableCamera(webcam1, webcam2);

        // Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Set the active camera to Webcam 1.
        switchableCamera = (SwitchableCamera) vuforia.getCamera();
        switchableCamera.setActiveCamera(webcam1);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.7f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
    private int DuckSpot(double DuckX) {
        if(DuckX < 233) {
            return 1;
        } else if(DuckX < 467) {
            return 2;
        } else {
            return 3;
        }
    }
}