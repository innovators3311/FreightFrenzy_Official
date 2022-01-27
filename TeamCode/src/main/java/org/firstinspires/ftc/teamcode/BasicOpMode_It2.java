package org.firstinspires.ftc.teamcode;

    import com.qualcomm.robotcore.eventloop.opmode.Disabled;
    import com.qualcomm.robotcore.eventloop.opmode.OpMode;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
    import com.qualcomm.robotcore.hardware.DcMotor;
    import com.qualcomm.robotcore.hardware.HardwareMap;
    import com.qualcomm.robotcore.util.ElapsedTime;
    import com.qualcomm.robotcore.util.Range;

    import org.checkerframework.common.util.report.qual.ReportOverride;

@TeleOp(name="STRAFE/ARM.test", group = "Inno")

public class BasicOpMode_It2 extends OpMode
{    // Declare OpMode members.

    HardwareInnoBot2 Inno = new HardwareInnoBot2();
    private final ElapsedTime runtime = new ElapsedTime();

    //HardwareMap hardwareMap = ;
    //public void runOpMode() throws InterruptedException{
    //}

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        Inno.init(hardwareMap);

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

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each org.firstinspires.ftc.teamcode.drive wheel to save power level for telemetry
        double leftPower = 0;
        double rightPower = 0;
        double clawPosition = 0;
        double magPosition = 0;

        leftPower = -gamepad1.left_stick_y;
        rightPower = -gamepad1.right_stick_y;

        if(gamepad1.right_bumper){
            clawPosition = 1.0;
        }
        if(gamepad1.left_bumper){
            magPosition = 1.0;
        }
        if (gamepad1.dpad_right) {
            Inno.lf.setPower(-1);
            Inno.rf.setPower(1);
            Inno.lb.setPower(1);
            Inno.rb.setPower(-1);
        }
        //if (gamepad1.b) {
        //    Inno.ax.setPower(0.5);
        //}
        //if (gamepad1.x) {
        //    Inno.ax.setPower(-0.5);
        //}
        //if (gamepad1.y) {
        //    Inno.ay.setPower(0.5);
        //}
        //if (gamepad1.a) {
         //   Inno.ay.setPower(-0.4);
       // }

        //Inno.claw.setPosition(clawPosition);
        //Inno.mag.setPosition(magPosition);
        Inno.lf.setPower(leftPower);
        Inno.rf.setPower(rightPower);
        Inno.lb.setPower(leftPower);
        Inno.rb.setPower(rightPower);


        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            //hi. you found me. -SECRET COMMENT