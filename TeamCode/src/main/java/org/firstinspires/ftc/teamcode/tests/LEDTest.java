package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "goBildaLEDTest")
public class LEDTest extends OpMode {
    private Servo led;

    public void init() {
        led = hardwareMap.get(Servo.class, "led");
    }

//    public void displayColor(double color) {
//        if (Math.abs(led.getPosition() - color) > 0.01) {
//            led.setPosition(color);
//        }
//    }

    // see https://www.gobilda.com/rgb-indicator-light-pwm-controlled/ for the full color range

    public void loop() {
//        displayColor(0.5);
        led.setPosition(0.5);
    }
}
