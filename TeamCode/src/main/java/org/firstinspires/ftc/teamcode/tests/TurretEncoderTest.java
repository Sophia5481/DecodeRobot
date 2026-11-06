package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.TurretWithEncoder;

@Config
@TeleOp(name = "TurretEncoderTest", group = "Test")
public class TurretEncoderTest extends OpMode {
    private ServoEx servo;

    public TurretWithEncoder turret;
    // centered position is 157.5
    public static double TARGET_DEGREE;

    @Override
    public void init() {
        turret = new TurretWithEncoder(hardwareMap);
        turret.init(false);
        telemetry = new MultipleTelemetry(
            telemetry,
            FtcDashboard.getInstance().getTelemetry()
        );
    }

    @Override
    public void loop() {
        telemetry.addData("Turret Angle: ", "" + turret.getTurretAngle());
        telemetry.addData("Turret Correction: ", "" + turret.getCorrection());
        telemetry.addData("Turret Angle with Offset: ", "" + turret.getNewAngle());
        telemetry.update();
        turret.update(TARGET_DEGREE);
    }
}
