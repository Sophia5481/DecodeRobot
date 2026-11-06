package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import java.util.List;

@Config
@TeleOp(name = "ShooterPIDTest", group = "Test")
public class Shooter2PIDTest extends OpMode {
    private Shooter shooter;

    // Target velocity in ticks/sec (for 6000 RPM motor ~ 2800 ticks/sec)
    public static double targetVelocity = 1500.0;
    // target velocity 1500 almost got perfect result

    List<LynxModule> hubs;
    @Override
    public void init() {
        // Initialize dashboard telemetry
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        telemetry.addData("init", "init");
        telemetry.update();

        // Enable bulk caching for faster loop times
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Initialize motor
        shooter = new Shooter(hardwareMap);
        shooter.init();
    }

    @Override
    public void loop() {
        // Set target velocity
        shooter.setVelocity(targetVelocity);
        // Get actual velocity
        double actualVelocity = shooter.getVelocity();

        telemetry.addData("Actual velocity", actualVelocity);
        telemetry.addData("Target velocity ", targetVelocity);
        telemetry.update();
    }

    @Override
    public void stop() {
        shooter.shooterOff();
    }
}
