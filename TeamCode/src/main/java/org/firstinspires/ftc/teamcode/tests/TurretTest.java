package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Turret;

@Config
@TeleOp(name = "TurretTest", group = "Test")
public class TurretTest extends OpMode {
    private ServoEx servo;

    public Turret turret;
    // centered position is 157.5
    public static double TARGET_DEGREE;

    @Override
    public void init() {
        turret = new Turret(hardwareMap);
    }

    @Override
    public void loop() {
        turret.setTargetDegree(TARGET_DEGREE);
    }
}
