package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.TurretWithEncoder;

@Config
@TeleOp(name = "EncoderTest", group = "Test")
public class EncoderTest extends OpMode {
    private ServoEx servo;

    public TurretWithEncoder turret;
    // centered position is 157.5
    public static double TARGET_DEGREE;

    @Override
    public void init() {
        turret = new TurretWithEncoder(hardwareMap);
        turret.resetEncoder();
        telemetry = new MultipleTelemetry(
            telemetry,
            FtcDashboard.getInstance().getTelemetry()
        );
    }

    @Override
    public void loop() {
        double angle = turret.getTurretAngle();
        telemetry.addData("Turret angle: ", "" + angle);
    }
}
