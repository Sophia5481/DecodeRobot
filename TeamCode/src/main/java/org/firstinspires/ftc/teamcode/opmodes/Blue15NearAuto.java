package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.Command;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.commands.SequentialCommandGroup;

@Disabled
@Autonomous(name = "Blue Near 15 Ball Auto", group = "Auto")
public class Blue15NearAuto extends Auto15BallNearBase {

    @Override
    protected boolean isRedAlliance() {
        return false;
    }

    @Override
    protected Command scheduleSequence(){
        return new SequentialCommandGroup(
                scorePreload(),
                intakeAndScore(1),
                intakeAndScore(2),
                gateCycle(),
                gateCycle(),
                park()
        );
    }
}