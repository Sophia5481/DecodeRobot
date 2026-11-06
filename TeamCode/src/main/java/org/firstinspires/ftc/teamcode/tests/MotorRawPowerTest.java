package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

@Config
@TeleOp(name = "MotorRawPowerTest", group = "Test")
public class MotorRawPowerTest extends OpMode {
    // 6000 RPM motor has around 2800 ticks per second
    private MotorEx motor;
    public static String NAME = "intakeRight";
    public static double kS = 0.92;
    public static double motor_velocity;
    public static double motor_acceleration;

    @Override
    public void init() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        motor = new MotorEx(hardwareMap, NAME);
        motor.setRunMode(MotorEx.RunMode.RawPower);
        motor.resetEncoder();
    }

    @Override
    public void loop() {
        motor.set(kS);
        motor_velocity = motor.getVelocity();
        motor_acceleration = motor.getAcceleration();
        telemetry.addData("motor_velocity", motor_velocity);
        telemetry.addData("Motor acceleration: ", motor_acceleration);
        telemetry.update();
    }
}
