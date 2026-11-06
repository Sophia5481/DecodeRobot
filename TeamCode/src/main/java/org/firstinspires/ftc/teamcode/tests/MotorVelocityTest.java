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
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import java.util.List;

@Config
@TeleOp(name = "MotorVelocityTest", group = "Test")
public class MotorVelocityTest extends OpMode {
    // 6000 RPM motor has around 2800 ticks per second
    private MotorEx shooter;
    private SimpleServo lid;
    private Turret turret;
    public static double kP = 0.05;
    public static double kI = 0.01;
    public static double kD = 0.31;
    public static double kS = 0.0;
    public static double kV = 0.0;
    public static double kA = 0.0;
    //    public static double motor_power = 0.5;
    public static double target_velo = 500.0;
    public static double motor_velocity;
    public static double motor_acceleration;
    public static double lidAngle;
    public static double turretAngle = 0.0;
    List<LynxModule> hubs;

    @Override
    public void init() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        turret = new Turret(hardwareMap);
        hubs = hardwareMap.getAll(LynxModule.class);
        lid = new SimpleServo(hardwareMap, "hoodServo", 0, 90);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        shooter = new MotorEx(hardwareMap, "shooter");
        lid = new SimpleServo(hardwareMap, "adjustableLid", 0, 90);
        shooter.setInverted(false);
        shooter.encoder.setDirection(MotorEx.Direction.REVERSE);
        shooter.setRunMode(MotorEx.RunMode.VelocityControl);
//        shooter.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        shooter.setVeloCoefficients(kP, kI, kD);
        shooter.setFeedforwardCoefficients(kS, kV, kA);
        shooter.resetEncoder();
    }

    @Override
    public void loop() {
        if (shooter.getVelocity() == target_velo) {
            telemetry.addData("At target velocity: ", "true");
        }
//        shooter.set(motor_power);
        turret.setTargetDegree(turretAngle);
        shooter.setVelocity(target_velo);
        lid.turnToAngle(lidAngle);
        motor_velocity = shooter.getVelocity();
        motor_acceleration = shooter.getAcceleration();
        telemetry.addData("motor kP value: ", kP);
        telemetry.addData("pos ", shooter.getCurrentPosition());
        telemetry.addData("motor velo coefficients: ", shooter.getVeloCoefficients());
        telemetry.addData("target velo: ", target_velo);
        telemetry.addData("motor_velocity ", motor_velocity);
        telemetry.addData("Motor acceleration: ", motor_acceleration);
        telemetry.update();
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
    }
}