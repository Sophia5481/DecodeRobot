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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

import java.util.List;

@Config
@TeleOp(name = "SDKMotorVelocityTest", group = "Test")
public class SDKMotorVelocityTest extends OpMode {
    // 6000 RPM motor has around 2800 ticks per second
    private DcMotorEx shooter;
    public static double kP = 0.05;
    public static double kI = 0.01;
    public static double kD = 0.31;
    public static double kF = 0.0;
    public static boolean shooterReversed = true;
    public static boolean encoderReversed = true;

    public static double target_velo = 500.0;
    public static double motor_velocity;
    public static double motor_acceleration;
    List<LynxModule> hubs;

    @Override
    public void init() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setDirection(shooterReversed ? DcMotorEx.Direction.REVERSE : DcMotorEx.Direction.FORWARD);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter.setVelocityPIDFCoefficients(kP, kI, kD, kF);
    }

    @Override
    public void loop() {
        if (shooter.getVelocity() == target_velo) {
            telemetry.addData("At target velocity: ", "true");
        }
//        shooter.set(motor_power);
        shooter.setVelocity(target_velo);
        motor_velocity = shooter.getVelocity();
        telemetry.addData("motor kP value: ", kP);
        telemetry.addData("pos ", shooter.getCurrentPosition());
        telemetry.addData("target velo: ", target_velo);
        telemetry.addData("motor_velocity ", motor_velocity);
        telemetry.update();
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
    }
}
