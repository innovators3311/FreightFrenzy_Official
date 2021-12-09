package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

public class CompassCalibration (CompassSensor compass){


    double HOLD_TIME_MS = 1000;

    HardwareInnoBot2 Inno = new HardwareInnoBot2();

        Inno.init(hardwareMap ahwMap);

        // get a reference to our Compass Sensor object.


        // Set the compass to calibration mode
        compass.setMode(CompassSensor.CompassMode.CALIBRATION_MODE);

        sleep(HOLD_TIME_MS);  // Just do a sleep while we switch modes

        compass.setMode(CompassSensor.CompassMode.MEASUREMENT_MODE);


        sleep(HOLD_TIME_MS);  // Just do a sleep while we switch modes

}