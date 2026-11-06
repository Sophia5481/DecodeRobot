package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.InstantCommand;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

public class PinpointAlignmentCommand extends SequentialCommandGroup {
    public PinpointAlignmentCommand (Turret turret, double targetAngle) {
        addRequirements(turret);

        addCommands(
                new InstantCommand(()-> {
                    double targetHeadingDegrees = Math.toDegrees(targetAngle);
                    Log.d("pinpoint", "target angle radians " + targetAngle);
                    Log.d("pinpoint", "targetHeadingDegrees " + targetHeadingDegrees);
                    turret.setTargetDegree(targetHeadingDegrees);
                })
        );
    }
}
