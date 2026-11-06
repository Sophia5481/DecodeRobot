package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.subsystems.Shooter.LID_FAR;
import static org.firstinspires.ftc.teamcode.subsystems.Shooter.LID_NEAR;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;

// turns on the shooter and sets the lid angle without any control of the feeder system
public class StartShooterCommand extends InstantCommand {
    public StartShooterCommand(Shooter shooter, boolean far) {
        super(() -> {
            shooter.setStopShooting(false);
            shooter.setShooterEnabled(true);
            shooter.setShooterFar(far);
        });
    }
}
