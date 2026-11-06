package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;
import java.util.Timer;

@Config
@TeleOp(name = "ShooterFeedForwardTest", group = "Test")
public class ShooterFeedForwardTest extends OpMode {
    // 6000 RPM motor has around 2800 ticks per second
    private MotorEx shooter;
    private MotorEx shooter2;
    public static String NAME = "shooter";
    public static String NAME2 = "motorTop";
    public static double power = 0.2;
    public static long time = 0;

    //V3 prototype
    /**
     * power = 0.2, velocity = 400
     * power = 0.4, velocity = 1000
     * power = 0.6, velocity = 1520
     * power = 0.8, velocity = 2060
     * power = 1.0, velocity = 2520
     *
     * m = 0.000376634, b = 0.0350483
     */

    //V3 new shooter 3/15/26
    /**
     * power = 0.2, velocity = 320
     * power = 0.4, velocity = 860
     * power = 0.6, velocity = 1380
     * power = 0.8, velocity = 1920
     * power = 1.0, velocity = 2360
     *
     * m = 0.000388617, b = 0.0683723
     */

    //V3 new shooter AND Mega heavy flywheels 3/15/26
    /**
     * power = 0.2, velocity = 360
     * power = 0.4, velocity = 940
     * power = 0.6, velocity = 1460
     * power = 0.8, velocity = 2000
     * power = 1.0, velocity = 2500
     *
     * m=0.000374306, b=0.0565076
     */

    //V3 1:2 sprocket ratio testing
    /**
     * power = 0.2, velocity = 440
     * power = 0.4, velocity = 1000
     * power = 0.6, velocity = 1500
     * power = 0.8, velocity = 2040
     * power = 1.0, velocity = 2500
     *
     * m=0.000374306, b=0.0565076
     */

    //V3 even more weights
    /**
     * power = 0.2, velocity = 440
     * power = 0.4, velocity = 960
     * power = 0.6, velocity = 1420
     * power = 0.8, velocity = 1920
     * power = 1.0, velocity = 2340
     *
     * m=0.000419694, b=0.00571348
     */
    //V3 3/20
    /**
     * power = 0.2, velocity = 360
     * power = 0.4, velocity = 860
     * power = 0.6, velocity = 1320
     * power = 0.8, velocity = 1740
     * power = 1.0, velocity = 2200
     * y=0.000453234x+0.0198599
     * m = 0.00453234, b = 0.0198599
     * Note: DO NOT USE (FAILED)
     */

    /**
     * power = 0.2, velocity = 420
     * power = 0.4, velocity = 960
     * power = 0.6, velocity = 1440
     * power = 0.8, velocity = 1920
     * power = 1.0, velocity = 2360
     *
     * m=0.000412688 b=0.0139836
     *
     */

    // New gobilda flywheels with higher rotational inertia
    /**
     * power = 0.2, velocity = 440
     * power = 0.4, velocity = 1000
     * power = 0.6, velocity = 1500
     * power = 0.8, velocity = 2000
     * power = 1.0, velocity = 2460 (or 2480)
     *
     * m=0.000412688 b=0.0139836
     *
     */

    // Single belt drive???
    /**
     * power = 0.2, velocity = 340
     * power = 0.4, velocity = 900
     * power = 0.5, velocity = 1140
     * power = 0.6, velocity = 1420
     * power = 0.7, velocity = 1700
     * power = 0.8, velocity = 1960
     *
     *
     * m=0.000370647 b=0.072496
     *
     */

    // Single belt drive but looser
    /**
     * power = 0.2, velocity = 320
     * power = 0.4, velocity = 860
     * power = 0.5, velocity = 1140
     * power = 0.6, velocity = 1380
     * power = 0.7, velocity = 1660
     * power = 0.8, velocity = 1900
     *
     *
     * y=0.000378611x+0.0752141
     *
     */

    private double motor_velocity;
    private double motor_acceleration;
    List<LynxModule> hubs;

    @Override
    public void init() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        shooter = new MotorEx(hardwareMap, NAME);
        shooter.setRunMode(MotorEx.RunMode.RawPower);
//        shooter.setInverted(true);
//        shooter.encoder.setDirection(MotorEx.Direction.REVERSE);

        shooter2 = new MotorEx(hardwareMap, NAME2);
        shooter2.setRunMode(MotorEx.RunMode.RawPower);
        shooter.setInverted(false);
//        shooter2.encoder.setDirection(MotorEx.Direction.REVERSE);
        shooter.resetEncoder();
//        shooter.set(power);
//        shooter2.set(power);
//        time = System.currentTimeMillis();
    }

    @Override
    public void loop() {
//        if (System.currentTimeMillis() - time > 5000) {
//            shooter.set(0);
//            shooter2.set(0);
//        }
        shooter.set(power);
        shooter2.set(power);
        motor_velocity = shooter.getVelocity();
        motor_acceleration = shooter.getAcceleration();
        telemetry.addData("pos ", shooter.getCurrentPosition());
        telemetry.addData("motor_velocity ", motor_velocity);
        telemetry.addData("Motor acceleration: ", motor_acceleration);
        telemetry.update();
    }

    @Override
    public void stop() {
        shooter.set(0);
    }
}