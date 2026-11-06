package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

public abstract class Auto21FarBase extends AutoFarBase {

    @Override
    protected void setupAllianceSpecifics() {
        super.setupAllianceSpecifics();
    }

    @Override
    protected Command scheduleSequence() {
            return new SequentialCommandGroup(
                scorePreload(),
                scoreWallInitial(),
                scoreLoadingZone(),
                scoreWall(2),
                scoreWall(3),
                scoreWall(4),
                scoreWall(5),
                park()
            );
    }
}