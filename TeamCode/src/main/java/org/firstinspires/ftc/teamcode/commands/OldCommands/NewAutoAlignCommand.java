package org.firstinspires.ftc.teamcode.commands.OldCommands;

import android.util.Log;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.subsystems.LimelightVision;

import java.util.List;
import java.util.function.Supplier;

public class NewAutoAlignCommand extends SequentialCommandGroup {
    public NewAutoAlignCommand(Follower follower, LimelightVision limelight, Supplier<Integer> tagSupplier) {
        addRequirements(limelight);

        addCommands(
            new InstantCommand(() -> {
                Robot.LIMELIGHT_FAILED = false;
                Log.d("limelight", "entered");
                Log.d("New Auto ALign Command: ", "targetID: " + tagSupplier.get());
            }),
            // Wait until LL sees the tag
            new WaitUntilCommand(() -> {
                List<LLResultTypes.FiducialResult> fid = limelight.getFiducials();
                if (fid == null || fid.isEmpty()) {
                    Log.d("limelight", "fiducials empty");
                    Robot.LIMELIGHT_FAILED = true;
                    Log.d("limelight", "limelight failed: " + Robot.LIMELIGHT_FAILED);
                    return false;
                }
                return fid.stream().anyMatch(f -> f.getFiducialId() == tagSupplier.get());
            }),

            // Now safely read fresh data
            new InstantCommand(() -> {
                Log.d("limelight", "after if");
                LLResultTypes.FiducialResult target =
                    limelight.getFiducials().stream()
                        .filter(f -> f.getFiducialId() == tagSupplier.get())
                        .findFirst().orElse(null);

                if (target.getTargetArea() < 0.001) {
                    Log.d("target", "target area invalid");
                    Robot.LIMELIGHT_FAILED = true;
                    return;
                }

                double yawToTagDegrees = limelight.getYawToFiducial(target);
                Log.d("heading", "yawToTagDegrees: " + yawToTagDegrees);

                // check if limelight's reusing the same stale value
                if (Math.abs(yawToTagDegrees - limelight.getLastDetectedAngle()) < 0.001) {
                    Log.d("heading", "saw stale value");
                    Robot.LIMELIGHT_FAILED = true;
                }

                limelight.setLastDetectedAngle(yawToTagDegrees);

                Pose currentPose = follower.getPose();
                double currentHeading = currentPose.getHeading();
                Log.d("heading", "current heading: " + Math.toDegrees(currentHeading));


                double targetHeadingDegrees = Math.toDegrees(currentHeading) - yawToTagDegrees - Math.signum(yawToTagDegrees) * 1;
                Log.d("heading", "target heading: " + targetHeadingDegrees);
                Log.d("heading", "target position: " + currentPose.getX());
                Log.d("heading", "target position: " + currentPose.getY());

                follower.holdPoint(
                    new BezierPoint(
                        new Point(currentPose.getX(), currentPose.getY(), Point.CARTESIAN)),
                    Math.toRadians(targetHeadingDegrees));
            })
        );
    }

}