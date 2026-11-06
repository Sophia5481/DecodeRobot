package org.firstinspires.ftc.teamcode.pedroPathing.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

@Config
@TeleOp(name = "Motor Direction Tuner", group = "Teleop")
public class MotorDirectionDebugger extends OpMode {
    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;
    public static boolean LEFT_FRONT_REVERSED = true;
    public static boolean RIGHT_FRONT_REVERSED = false;
    public static boolean LEFT_REAR_REVERSED = true;
    public static boolean RIGHT_REAR_REVERSED = false;
    private Gamepad g1;
    public static double MOTOR_POWER = 1.0;

    @Override
    public void init() {
        g1 = new Gamepad();
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        leftFront.setDirection(LEFT_FRONT_REVERSED ? DcMotorEx.Direction.REVERSE : DcMotorEx.Direction.FORWARD);
        leftRear.setDirection(LEFT_REAR_REVERSED ? DcMotorEx.Direction.REVERSE : DcMotorEx.Direction.FORWARD);
        rightFront.setDirection(RIGHT_FRONT_REVERSED ? DcMotorEx.Direction.REVERSE : DcMotorEx.Direction.FORWARD);
        rightRear.setDirection(RIGHT_REAR_REVERSED ? DcMotorEx.Direction.REVERSE : DcMotorEx.Direction.FORWARD);
    }

    @Override
    public void loop() {
        if (gamepad1.x) {
            telemetry.addLine("x");
            leftFront.setPower(MOTOR_POWER);
        }
        if (gamepad1.y) {
            telemetry.addLine("y");
            rightFront.setPower(MOTOR_POWER);
        }
        if (gamepad1.a) {
            telemetry.addLine("a");
            leftRear.setPower(MOTOR_POWER);
        }
        if (gamepad1.b) {
            telemetry.addLine("b");
            rightRear.setPower(MOTOR_POWER);
        }
        telemetry.update();
    }
}
