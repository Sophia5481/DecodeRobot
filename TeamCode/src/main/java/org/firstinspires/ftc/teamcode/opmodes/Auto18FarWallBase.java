package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;

public abstract class Auto18FarWallBase extends AutoFarBase {

    @Override
    protected void setupAllianceSpecifics() {
        super.setupAllianceSpecifics();
    }

    @Override
    protected Command scheduleSequence() {
        return new SequentialCommandGroup(
                scorePreload(),
                scoreWall(1),
                scoreWall(2),
                scoreWall(3),
                scoreWall(4),
                scoreWall(5),
                park()
        );
    }

    protected Command retryIntake(int cycle) {
        Path backOutPath = new Path(new BezierLine(
            wallIntakeFinal,
                wallIntakeBack
        ));
        backOutPath.setConstantHeadingInterpolation(intakeHeading);

        Path toWallZone = new Path(new BezierLine(
                wallIntakeBack,
            wallIntakeFinal
        ));
        toWallZone.setConstantHeadingInterpolation(intakeHeading);

        PathChain retryPath = new PathChain(
                backOutPath,
                toWallZone
        );

        return new FollowPathCommand(
                follower,
                retryPath
        );
    }
}