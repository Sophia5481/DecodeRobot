package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.VoltageSensing;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LimelightVision;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Vision2;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
//import org.firstinspires.ftc.teamcode.subsystems.BlobVision;

import java.util.List;

public class Robot extends com.arcrobotics.ftclib.command.Robot {
    private Intake intake;
    private Shooter shooter;
    private Turret turret;
//    private Vision2 vision;
    public Telemetry telem;
    private LimelightVision limelight;

    public static boolean RED_ALLIANCE = true;
    public static boolean FAR_AUTO = false;
    public static boolean AUTO_NOSORTING = false;
    public static boolean LIMELIGHT_FAILED = false;
    public static int targetTeleTag = 20;

    List<LynxModule> hubs;

    public Robot(boolean auto, final HardwareMap hMap) {
        shooter = new Shooter(hMap);
        intake = new Intake(hMap);
        turret = new Turret(hMap);
        limelight = new LimelightVision(hMap);
//        vision = new Vision2(hMap);
        register(shooter);
        register(intake);
//        register(vision);
        VoltageSensing.initialize(hMap);

        hubs = hMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        if (auto) {
            initAuto(hMap);
        } else {
            initTele(hMap);
        }
    }

    public void initTele(final HardwareMap hMap) {
// TODO: Add memory logic
//        feeder.init(false);
//        shooter.init();
//        transfer.init();
//        intake.init(false);
//        vision.init(true, RED_ALLIANCE);
        if (RED_ALLIANCE) {
            targetTeleTag = 24;
        } else {
            targetTeleTag = 20;
        }
    }

    public void startTele() {
        shooter.init();
        intake.init();
        turret.init(false);
//        vision.init(true, RED_ALLIANCE);
    }

    public void initAuto(final HardwareMap hMap) {
        intake.init();
        shooter.init();
        turret.init(true);
//        vision.init(false, RED_ALLIANCE);
    }

    public void resetCache() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
    }

    public void allianceConfig(Telemetry telemetry, Gamepad gamepad1) {
        telemetry.addData("Alliance", RED_ALLIANCE ? "Red" : "Blue");
        telemetry.addLine("Use driver 1's " + (RED_ALLIANCE ? "Right" : "Left") + " bumper to switch alliances");
        telemetry.update();
        if (gamepad1.left_bumper) {
            RED_ALLIANCE = true;
        } else if (gamepad1.right_bumper) {
            RED_ALLIANCE = false;
        }
    }


    public Shooter getShooter() {
        return shooter;
    }
    public Turret getTurret(){return turret;}

    public Intake getIntake(){return intake;}

    public LimelightVision getLimelight() {
        return limelight;
    }

//    public Vision2 getVision() {
////        return vision;
//        return null;
//    }
}
