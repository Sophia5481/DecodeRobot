package org.firstinspires.ftc.teamcode.tests;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import java.util.List;

@Config
@TeleOp(name = "ShooterIntakeTest", group = "Test")
public class ShooterIntakeTest extends OpMode {
    // 6000 RPM motor has around 2800 ticks per second
    private MotorEx shooter;
    private MotorEx shooterTop;
    private MotorEx intakeRight;
    private MotorEx intakeLeft;
    private SimpleServo lid;
    private SimpleServo latch;
    private SimpleServo turret;
    private Shooter shooterSubsystem;
    public static double kP = 0.05;
    public static double kI = 0.01;
    public static double kD = 0.31;
    public static double kS = 0.0;
    public static double kV = 0.0;
    public static double kA = 0.0;
    //    public static double motor_power = 0.5;
    public static double target_velo = 500;
    public static double motor_velocity;
    public static double motor_acceleration;

    public static double intakePower = 0.0;
    public static double lidAngle = 0.0;
    public static double turretAngle = 0.0;
    public static double latchAngle = 360;
    List<LynxModule> hubs;

    @Override
    public void init() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
//        lid = new SimpleServo(hardwareMap, "hoodServo", 0, 280);
        latch = new SimpleServo(hardwareMap, "latch", 300, 360);
        turret = new SimpleServo(hardwareMap, "turretRight", 0, 360);
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        shooter = new MotorEx(hardwareMap, "shooter");
        shooter.setInverted(true);
        shooter.encoder.setDirection(MotorEx.Direction.REVERSE);
        shooter.setRunMode(MotorEx.RunMode.RawPower);
        shooterTop = new MotorEx(hardwareMap, "motorTop");
        shooterTop.setInverted(false);
        shooterTop.setRunMode(Motor.RunMode.RawPower);
//        shooter.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        shooter.resetEncoder();

        intakeRight = new MotorEx(hardwareMap, "intakeRight");
        intakeRight.setRunMode(Motor.RunMode.RawPower);
        intakeLeft = new MotorEx(hardwareMap, "intakeLeft");
        intakeLeft.setRunMode(Motor.RunMode.RawPower);
        intakeLeft.setInverted(true);

        shooterSubsystem = new Shooter(hardwareMap);
    }

    @Override
    public void loop() {
        Log.d("Test:TargetVelocity: ", " " + target_velo);
        Log.d("Test:TargetVelocity: ", " " + target_velo);
        if (shooterSubsystem.getVelocity() == target_velo) {
            telemetry.addData("At target velocity: ", "true");
        }
        shooterSubsystem.setVelocity(target_velo);
//        shooter.set(motor_power);
        intakeRight.set(intakePower);
        intakeLeft.set(intakePower);
        //turret.turnToAngle(turretAngle);
        latch.turnToAngle(latchAngle);
        shooter.setVelocity(target_velo);
        shooterTop.setVelocity(target_velo);
//        lid.turnToAngle(lidAngle);
        motor_velocity = shooterSubsystem.getVelocity();
        telemetry.addData("motor kP value: ", kP);
        telemetry.addData("pos ", shooter.getCurrentPosition());
        telemetry.addData("motor velo coefficients: ", shooter.getVeloCoefficients());
        telemetry.addData("target velo: ", target_velo);
        telemetry.addData("motor_velocity ", motor_velocity);
        telemetry.addData("turret angle:", turretAngle);
//        telemetry.addData("lid angle: ", lidAngle);
        telemetry.addData("latch angle: ", latchAngle);
        telemetry.update();
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
    }
}