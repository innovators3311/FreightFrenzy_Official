package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.SwitchableCamera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

import org.firstinspires.ftc.teamcode.source.Arm;

@Autonomous(name = "Whole Field_FINAL", group = "Whole Field")
public class WholeField_FINAL extends LinearOpMode {

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

    //SETTING UP VUFORIA IMAGE TRACKING

    // IMPORTANT:  For Phone Camera, set 1) the camera source and 2) the orientation, based on how your phone is mounted:
    // 1) Camera Source.  Valid choices are:  BACK (behind screen) or FRONT (selfie side)
    // 2) Phone Orientation. Choices are: PHONE_IS_PORTRAIT = true (portrait) or PHONE_IS_PORTRAIT = false (landscape)

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */


    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here.  These are useful for the Freight Frenzy field.
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = 6 * mmPerInch;          // the height of the center of the target image above the floor
    private static final float halfField        = 72 * mmPerInch;
    private static final float halfTile         = 12 * mmPerInch;
    private static final float oneAndHalfTile   = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaTrackables targets = null ;

    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    //defining program states
    enum mainState {
        TRAJECTORY_1,   // Head to the duck carousel
        TRAJECTORY_2,   // Creep forward several inches to position accurately
        SPIN,           // Spin the duck
        TRAJECTORY_3,   // Go to the shipping hub
        TRAJECTORY_4,   // Drive forward slowly a few inches to the hub
        TRAJECTORY_4_2, // Creep up to shipping hub if necessary
        WAIT_1,         // Pause for robot to decelerate
        DROP_1,         // Drop block
        WAIT_2,         // Wait for block to drop
        TRAJECTORY_5,   // Position to get some freight
        TRAJECTORY_6,   // Continuation of trajectory 5
        TRAJECTORY_7,   // Get the freight
        IDLE            // Enter IDLE state when complete
    }
    enum armState {
        TIER_1,
        TIER_2,
        TIER_3,
        PICKUP_1,
        PICKUP_2,
        ARM_RESET,
        IDLE
    }
    enum imageTrackerState { //so as not to severely slow the loop down by running the tracker when it's not needed
        TRACK,
        IDLE
    }

    //defining the main state
    mainState currentState = mainState.IDLE;

    //defining the arm state
    armState currentArmState = armState.IDLE;

    //defining the tracker state
    imageTrackerState currentTrackerState = imageTrackerState.IDLE;

    //defining start pose
    Pose2d startPose = new Pose2d(30, -64, Math.toRadians(0));

    //initializing timer
    ElapsedTime timer = new ElapsedTime();
    int i = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing...");
        telemetry.update();
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection   = CAMERA_CHOICE;

        // Indicate that we wish to be able to switch cameras.
        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcam2 = hardwareMap.get(WebcamName.class, "Webcam 2");
        parameters.cameraName = ClassFactory.getInstance().getCameraManager().nameForSwitchableCamera(webcam1, webcam2);

        // Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Set the active camera to Webcam 1.
        switchableCamera = (SwitchableCamera) vuforia.getCamera();
        switchableCamera.setActiveCamera(webcam1);

        // Turn off Extended tracking.  Set this true if you want Vuforia to track beyond the target.
        parameters.useExtendedTracking = false;

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targets = this.vuforia.loadTrackablesFromAsset("FreightFrenzy");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targets);

        identifyTarget(0, "Blue Storage",       -halfField,  oneAndHalfTile, mmTargetHeight, 90, 0, 90);
        identifyTarget(1, "Blue Alliance Wall",  halfTile,   halfField,      mmTargetHeight, 90, 0, 0);
        identifyTarget(2, "Red Storage",        -halfField, -oneAndHalfTile, mmTargetHeight, 90, 0, 90);
        identifyTarget(3, "Red Alliance Wall",   halfTile,  -halfField,      mmTargetHeight, 90, 0, 180);

        /*
         * Create a transformation matrix describing where the phone is on the robot.
         *
         * Info:  The coordinate frame for the robot looks the same as the field.
         * The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
         * Z is UP on the robot.  This equates to a heading angle of Zero degrees.
         *
         * The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
         * pointing to the LEFT side of the Robot.
         * The two examples below assume that the camera is facing forward out the front of the robot.
         */

        // We need to rotate the camera around its long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -110; //-90 normally but in this case the camera is tilted slightly down
        } else {
            phoneYRotate = 90;
        }
        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }
        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered on the robot (left-to-right and front-to-back), and 6 inches above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = -1.625f * mmPerInch;   // eg: Enter the forward distance from the center of the robot to the camera lens
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.5f * mmPerInch;   // eg: Camera is 6 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 6.375f * mmPerInch;   // eg: Enter the left distance from the center of the robot to the camera lens

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1, 16.0/9.0);
        }

        telemetry.clear();
        telemetry.addLine("Scanning for duck...");
        telemetry.update();
        int Duck = 0;
        timer.reset();
        while(Duck == 0 && timer.milliseconds() < 3000) {
            if (tfod != null) {
                List<Recognition> recognitions = tfod.getRecognitions();
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : recognitions) {
                    if(recognition.getLabel() == "Duck" || recognition.getLabel() == "Cube" || recognition.getLabel() == "Ball") {
                        Duck = DuckSpot(recognition.getRight()-((recognition.getRight()-recognition.getLeft())/2));
                    }
                    i++;
                }
            }
        }
        if(Duck == 0) { //If it took too long to find the duck then it defaults to spot 3
            Duck = 1;
        }
        telemetry.clear();
        telemetry.addData("Duck Spot", Duck);
        telemetry.addLine("Building trajectories...");
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
                .lineToLinearHeading(new Pose2d(52, -55, Math.toRadians(137)))
                .build();
        Trajectory trajectory4_1 = drive.trajectoryBuilder(trajectory3_1.end()) //*duck spot 1* positions for bottom tier
                .back(24)
                .build();
        Trajectory trajectory4_1_1 = drive.trajectoryBuilder(trajectory4_1.end()) //*duck spot 3* positions for middle tier
                .back(8.5)
                .build();
        Trajectory trajectory4_2 = drive.trajectoryBuilder(trajectory3_1.end()) //*duck spot 2* positions for middle tier
                .back(27)
                .build();
        Trajectory trajectory4_2_2 = drive.trajectoryBuilder(trajectory4_2.end()) //*duck spot 2* positions for middle tier
                .back(7)
                .build();
        Trajectory trajectory4_3 = drive.trajectoryBuilder(trajectory3_2.end()) //*duck spot 3* positions for top tier
                .forward(29)
                .build();
        //*trajectory5* position to get some freight
        Trajectory trajectory5_1 = drive.trajectoryBuilder(trajectory4_1.end())
                .splineTo(new Vector2d(20.87, -67), Math.toRadians(180))
                .build();
        Trajectory trajectory5_2 = drive.trajectoryBuilder(trajectory4_2.end())
                .splineTo(new Vector2d(-12, -67), Math.toRadians(180))
                .build();
        Trajectory trajectory5_3 = drive.trajectoryBuilder(trajectory4_3.end())
                .lineToLinearHeading(new Pose2d(30, -33.2, Math.toRadians(180)))
                .build();
        Trajectory trajectory6_1 = drive.trajectoryBuilder(trajectory5_1.end())
                .lineToLinearHeading(new Pose2d(30, -58, Math.toRadians(170)))
                .build();
        Trajectory trajectory6_2 = drive.trajectoryBuilder(trajectory5_2.end())
                .lineToLinearHeading(new Pose2d(30, -58, Math.toRadians(170)))
                .build();
        Trajectory trajectory6_3 = drive.trajectoryBuilder(trajectory5_3.end())
                .lineToLinearHeading(new Pose2d(30, -58, Math.toRadians(170))) //empty comment so i can push again
                .build();
        Trajectory trajectory7 = drive.trajectoryBuilder(trajectory6_3.end()) //going to get freight from warehouse
                .forward(100)
                .build();
        telemetry.clear();
        telemetry.addLine("Fully loaded!  Smash that play button!");
        telemetry.update();
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
                    if (timer.milliseconds() > 3000) {
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
                        switch(Duck) {
                            case 1:
                                drive.followTrajectoryAsync(trajectory4_1);
                                currentArmState = armState.TIER_1;
                                break;
                            case 2:
                                drive.followTrajectoryAsync(trajectory4_2);
                                currentArmState = armState.TIER_2;
                                break;
                            case 3:
                                drive.followTrajectoryAsync(trajectory4_3);
                                currentArmState = armState.TIER_3;
                                break;
                        }
                        targets.activate(); //activating vuforia tracking
                        currentTrackerState = imageTrackerState.TRACK;
                    }
                    break;
                case TRAJECTORY_4: //going to the shipping hub and dropping freight
                    if (!drive.isBusy() && !arm.shoulderIsBusy && !arm.elbowIsBusy) {
                        if(Duck < 3) {
                            timer.reset();
                            currentState = mainState.WAIT_1;
                        } else {
                            arm.openClaw();
                            arm.retractMagnet();
                            currentState = mainState.WAIT_2;
                            timer.reset();
                        }
                    }
                    break;
                case WAIT_1:
                    if(timer.milliseconds() > 250) {
                        currentState = mainState.TRAJECTORY_4_2;
                        if(Duck == 1) {
                            drive.followTrajectoryAsync(trajectory4_1_1);
                        } else {
                            drive.followTrajectoryAsync(trajectory4_2_2);
                        }
                    }
                    break;
                case TRAJECTORY_4_2: //going to the shipping hub and dropping freight
                    if (!drive.isBusy()) {
                        arm.openClaw();
                        arm.retractMagnet();
                        currentState = mainState.WAIT_2;
                        timer.reset();
                    }
                    break;
                case WAIT_2:
                    arm.openClaw();
                    if(timer.milliseconds() > 1500) {
                        arm.openClaw();
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
                    break;
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
                    }
                    break;
                case TRAJECTORY_6: //strafing to wall and lowering arm
                    if (!drive.isBusy() && !arm.shoulderIsBusy && !arm.elbowIsBusy) {
                        currentState = mainState.TRAJECTORY_7;
                        drive.followTrajectoryAsync(trajectory7);
                        currentArmState = armState.ARM_RESET;
                    }
                    break;
                case TRAJECTORY_7: //driving forward to get the freight
                    if (!drive.isBusy()) {
                        targets.deactivate();
                        currentState = mainState.IDLE;
                    }
                    break;
                case IDLE:
                    arm.storeArmPose();
                    break;
            }

            switch(currentArmState) {
                case TIER_1:
                    arm.runShoulderTo(155);
                    arm.runElbowTo(270);
                    currentArmState = armState.IDLE;
                    break;
                case TIER_2:
                    arm.runShoulderTo(128);
                    arm.runElbowTo(285);
                    currentArmState = armState.IDLE;
                    break;
                case TIER_3:
                    arm.runShoulderTo(50);
                    arm.runElbowTo(175);
                    currentArmState = armState.IDLE;
                    break;
                case PICKUP_1:
                    arm.runShoulderTo(-25);
                    arm.runElbowTo(215);
                    currentArmState = armState.IDLE;
                    break;
                case PICKUP_2:
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
            switch (currentTrackerState) {
                case TRACK:
                    // check all the trackable targets to see which one (if any) is visible.
                    targetVisible = false;
                    for (VuforiaTrackable trackable : allTrackables) {
                        if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                            telemetry.addData("Visible Target", trackable.getName());
                            targetVisible = true;

                            // getUpdatedRobotLocation() will return null if no new information is available since
                            // the last time that call was made, or if the trackable is not currently visible.
                            OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                            if (robotLocationTransform != null) {
                                lastLocation = robotLocationTransform;
                            }
                            break;
                        }
                    }
                    if (targetVisible) {
                        // express position (translation) of robot in inches.
                        VectorF translation = lastLocation.getTranslation();
                        telemetry.addData("Pos (inches)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                                translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                        // express the rotation of the robot in degrees.
                        Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                        telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                    }
                    else {
                        telemetry.addData("Visible Target", "none");
                    }
                    break;
                case IDLE:
                    break;
            }

            //Updating all arm components continuously in the background, regardless of state
            arm.updateShoulder();
            arm.updateElbow();

            //Updating drive continuously in the background, regardless of state
            drive.update();

            //Printing data to telemetry
            telemetry.addData("current state", currentState);
            telemetry.addData("Shoulder error", arm.shoulderErr);
            telemetry.addData("Elbow error", arm.elbowErr);
            telemetry.addData("Tracker state", currentTrackerState);
            telemetry.addData("1", trajectory4_1.end());
            telemetry.addData("2", trajectory4_2.end());
            telemetry.addData("3", trajectory4_3.end());
            telemetry.addData("Duck", Duck);
            telemetry.update();
        }
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
    void identifyTarget(int targetIndex, String targetName, float dx, float dy, float dz, float rx, float ry, float rz) {
        VuforiaTrackable aTarget = targets.get(targetIndex);
        aTarget.setName(targetName);
        aTarget.setLocation(OpenGLMatrix.translation(dx, dy, dz)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, rx, ry, rz)));
    }
    private int DuckSpot(double DuckX) {
        if(DuckX < 233) {
            return 3;
        } else if(DuckX < 467) {
            return 2;
        } else {
            return 1;
        }
    }
}