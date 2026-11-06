package org.firstinspires.ftc.teamcode.commands.OldCommands;

import android.util.Log;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.commands.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.subsystems.LimelightVision;

public class PinpointLimelightAlignWithAprilTagCommand extends SequentialCommandGroup {
    public PinpointLimelightAlignWithAprilTagCommand(Follower follower, LimelightVision limelight, double targetAngle) {
        addRequirements(limelight);

        addCommands(
                new InstantCommand(() -> {
//                    limelight.setPollRate(100);
                    Log.d("limelight", "entered");



                    Pose currentPose = follower.getPose();
                    double currentHeading = currentPose.getHeading();
                    double targetHeadingDegrees = Math.toDegrees(targetAngle);
                    Log.d("pinpoint", "target angle radians " + targetAngle);
                    Log.d("pinpoint", "current heading " + currentHeading);
                    Log.d("pinpoint", "targetHeadingDegrees " + targetHeadingDegrees);
                    follower.holdPoint(
                            new BezierPoint(
                                    new Point(currentPose.getX(), currentPose.getY(), Point.CARTESIAN)),
                            Math.toRadians(targetHeadingDegrees));

//                    limelight.setPollRate(10);
                }
        ));
    }
}