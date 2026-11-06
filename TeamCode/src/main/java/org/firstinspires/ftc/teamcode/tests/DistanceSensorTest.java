package org.firstinspires.ftc.teamcode.tests;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@Config
@TeleOp(name = "DistanceSensorTest", group = "Test")
public class DistanceSensorTest extends OpMode {
    private DistanceSensor distanceSensor;
    public static String NAME = "distanceSensor";

    @Override
    public void init() {
        distanceSensor = hardwareMap.get(DistanceSensor.class, NAME);
    }

    @Override
    public void loop() {
        telemetry.addData("Distance: ", distanceSensor.getDistance(DistanceUnit.INCH));
    }
}