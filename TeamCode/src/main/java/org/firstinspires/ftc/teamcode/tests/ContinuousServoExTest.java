package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.hardware.motors.CRServo;

@Config
@TeleOp(name = "ServoExTest", group = "Test")
public class ContinuousServoExTest extends OpMode {
    private CRServo servo;
    public static String NAME = "linkage";
    public static double power = 0.0;
    public static boolean INVERTED = false;

    @Override
    public void init() {
        servo = new CRServo(hardwareMap, NAME);
    }

    @Override
    public void loop() {
        servo.set(power);
    }
}
