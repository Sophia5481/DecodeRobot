package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.Command;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.SequentialCommandGroup;

@Autonomous(name = "Red Near 18 Ball Auto", group = "Auto")
public class Red18NearAuto extends Auto18BallNearBase {

    @Override
    protected boolean isRedAlliance() {
        return true;
    }

    @Override
    protected Command scheduleSequence() {
        return new SequentialCommandGroup(
                scorePreload(),
                intakeAndScore(1),
                gateCycle(),
                gateCycle(),
                gateCycle(),
                intakeAndScore(2),
                park()
        );
    }
}