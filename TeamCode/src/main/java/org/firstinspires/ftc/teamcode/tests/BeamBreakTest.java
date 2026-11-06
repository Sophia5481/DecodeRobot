package org.firstinspires.ftc.teamcode.tests;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.util.BeamBreakSensor;

@Config
@TeleOp(name = "ArtifactDetectionTest", group = "Test")
public class BeamBreakTest extends OpMode {
    private static final double MAX_VOLTS = 3.3;
    private static final double MAX_DISTANCE_MM = 1000.0;
    private AnalogInput firstBallDetector;
    private AnalogInput secondBallDetector;
    private AnalogInput secondBallDetector2;
    private AnalogInput thirdBallDetector;
    public static String NAME = "beamBreak";

    @Override
    public void init() {
        firstBallDetector = hardwareMap.get(AnalogInput.class, "firstAnalogSensor");
        secondBallDetector = hardwareMap.get(AnalogInput.class, "secondAnalogSensor");
        secondBallDetector2 = hardwareMap.get(AnalogInput.class, "secondAnalogSensor2");
        thirdBallDetector = hardwareMap.get(AnalogInput.class, "thirdAnalogSensor");
    }

    @Override
    public void loop() {
        telemetry.addData("FirstArtifactPresent: ", hasFirstArtifact());
        telemetry.addData("SecondArtifactPresent: ", hasSecondArtifact());
        telemetry.addData("ThirdArtifactPresent: ", hasThirdArtifact());
    }

    public boolean hasFirstArtifact() {
        double volts = firstBallDetector.getVoltage();

        // Convert voltage to distance in millimeters (linear mapping)
        double distanceMM = (volts / MAX_VOLTS) * MAX_DISTANCE_MM;
        Log.d("Distance sensor 1st ball: ", "distance: " + distanceMM);
        return distanceMM < 90.0;
    }

    public boolean hasSecondArtifact() {
        double volts = secondBallDetector.getVoltage();

        // Convert voltage to distance in millimeters (linear mapping)
        double distanceMM = (volts / MAX_VOLTS) * MAX_DISTANCE_MM;
        Log.d("Distance sensor 2nd ball: ", "distance: " + distanceMM);

        double volts2 = secondBallDetector2.getVoltage();

        // Convert voltage to distance in millimeters (linear mapping)
        double distanceMM2 = (volts2 / MAX_VOLTS) * MAX_DISTANCE_MM;
        Log.d("Distance sensor 2nd ball 2: ", "distance: " + distanceMM2);
        return (distanceMM < 70.0) /*|| (distanceMM2 < 70.0)*/;
    }

    public boolean hasThirdArtifact() {
        double volts = thirdBallDetector.getVoltage();

        // Convert voltage to distance in millimeters (linear mapping)
        double distanceMM = (volts / MAX_VOLTS) * MAX_DISTANCE_MM;
        Log.d("Distance sensor 3nd ball: ", "distance: " + distanceMM);
        return distanceMM < 70.0;
    }
}
