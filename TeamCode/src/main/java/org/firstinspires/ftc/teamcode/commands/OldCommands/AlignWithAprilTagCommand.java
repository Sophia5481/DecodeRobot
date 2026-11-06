package org.firstinspires.ftc.teamcode.commands.OldCommands;

import android.util.Log;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.teamcode.commands.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.subsystems.LimelightVision;

import java.util.List;

public class AlignWithAprilTagCommand extends SequentialCommandGroup {
    public AlignWithAprilTagCommand(Follower follower, LimelightVision limelight, int targetID) {
        addRequirements(limelight);

        addCommands(
                new InstantCommand(() -> {
//                    limelight.setPollRate(100);
                    Log.d("limelight", "entered");

                    List<LLResultTypes.FiducialResult> fiducials = limelight.getFiducials();
                    if (fiducials == null || fiducials.isEmpty())  {
                        Log.d("limelight", "fiducials empty");
                        return;
                    }

                    Log.d("limelight", "after if");

                    LLResultTypes.FiducialResult target = null;
                    for (LLResultTypes.FiducialResult f : fiducials) {
                        if (f.getFiducialId() == targetID) { target = f; break; }
                    }
                    if (target == null) {
                        Log.d("target", "not seen");
                        return;
                    }

                    double yawToTagDegrees = limelight.getYawToFiducial(target);
                    Log.d("heading", "yawToTagDegrees: " + yawToTagDegrees);

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

//                    limelight.setPollRate(10);
                }
        ));
    }
}