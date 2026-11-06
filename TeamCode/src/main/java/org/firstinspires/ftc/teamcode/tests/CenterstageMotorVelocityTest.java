package org.firstinspires.ftc.teamcode.tests;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.util.MotorWithVelocityPID;
import org.firstinspires.ftc.teamcode.util.control.PIDCoefficients;

import java.util.List;

@Config
@TeleOp(name = "CenterstageMotorVelocityTest", group = "Test")
public class CenterstageMotorVelocityTest extends OpMode {
    // 6000 RPM motor has around 2800 ticks per second
    private DcMotorEx shooter;
    private MotorWithVelocityPID shooterController;
    public static PIDCoefficients pidCoefficients = new PIDCoefficients(0.05, 0.01, 0.31);
    //    public static double motor_power = 0.5;
    public static int target_velo = 500;
    public static double motor_velocity;
    List<LynxModule> hubs;

    @Override
    public void init() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        shooterController = new MotorWithVelocityPID(shooter, pidCoefficients);
        shooterController.setTargetVelocity(target_velo);
    }

    @Override
    public void loop() {
        shooterController.update();
        if (shooterController.getVelocity() == target_velo) {
            telemetry.addData("At target velocity: ", "true");
        }
//        shooter.set(motor_power);

        motor_velocity = shooterController.getVelocity();
        telemetry.addData("target velo: ", target_velo);
        telemetry.addData("motor_velocity ", motor_velocity);
        telemetry.update();
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
    }
}
