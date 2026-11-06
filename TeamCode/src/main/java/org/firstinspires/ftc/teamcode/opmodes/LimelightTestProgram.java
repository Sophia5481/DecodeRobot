package org.firstinspires.ftc.teamcode.opmode;

import android.util.Log;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.VoltageSensing;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.subsystems.LimelightVision;

import org.firstinspires.ftc.teamcode.Robot;

import java.util.List;

@TeleOp(name = "LimelightTestProgram", group = "TeleOp")
public class LimelightTestProgram extends OpMode {
    private LimelightVision limelight;
    private LLResult result = null;
    private Follower follower;
    private Robot robot;


    @Override
    public void init() {
        follower = new Follower(hardwareMap);
        limelight = new LimelightVision(hardwareMap);
        robot = new Robot(false, hardwareMap);
    }

    @Override
    public void start() {
        Log.d("ll test start", "batter voltage: " + VoltageSensing.readVoltage());
    }


    @Override
    public void loop() {
        telemetry.addLine("Initializing...");
        telemetry.update();

        Log.d("ll test", "entered");

        List<LLResultTypes.FiducialResult> fiducials = limelight.getFiducials();
        if (fiducials == null || fiducials.isEmpty()) {
            Log.d("ll test limelight", "fiducials empty");
            return;
        }

        LLResultTypes.FiducialResult target = null;
        for (LLResultTypes.FiducialResult f : fiducials) {
            if (f.getFiducialId() == 20) {
                target = f;
                break;
            }
        }
        if (target == null) {
            Log.d("ll test limelight", "target not seen");
            return;
        }

        double yawToTagDegrees = limelight.getYawToFiducial(target);
        Log.d("ll test heading", "yawToTagDegrees: " + yawToTagDegrees);

        Pose currentPose = follower.getPose();
        double currentHeading = currentPose.getHeading();
        Log.d("ll test heading", "current heading: " + Math.toDegrees(currentHeading));


        double targetHeadingDegrees = Math.toDegrees(currentHeading) - yawToTagDegrees;
        Log.d("ll test heading", "target heading: " + targetHeadingDegrees);

        Log.d("ll test voltage", "battery voltage: " + VoltageSensing.readVoltage());
    }

    @Override
    public void stop() {
        Log.d("ll test start", "battery voltage: " + VoltageSensing.readVoltage());
    }
}
