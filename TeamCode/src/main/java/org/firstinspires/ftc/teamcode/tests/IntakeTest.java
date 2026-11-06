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

import org.firstinspires.ftc.teamcode.subsystems.Turret;

@Config
@TeleOp(name = "IntakeTest", group = "Test")
public class IntakeTest extends OpMode {
    private MotorEx intakeRight;
    private MotorEx intakeLeft;
    public static double intakeVelocity = 1.0;
    @Override
    public void init() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        intakeRight = new MotorEx(hardwareMap, "intakeRight");
        intakeRight.setRunMode(Motor.RunMode.RawPower);
        intakeLeft = new MotorEx(hardwareMap, "intakeLeft");
        intakeLeft.setRunMode(Motor.RunMode.RawPower);
        intakeLeft.setInverted(true);
    }
    @Override
    public void loop(){
//        intakeRight.set(intakeVelocity);
        intakeLeft.set(intakeVelocity);
        telemetry.addData("velocity", intakeVelocity);
    }

}