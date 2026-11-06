package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;

// turns on the shooter and sets the lid angle without any control of the feeder system
public class DynamicStartShooterCommand extends InstantCommand {
    public DynamicStartShooterCommand(Shooter shooter) {
        super(() -> {
            shooter.setStopShooting(false);
            shooter.setShooterEnabled(true);
            shooter.enableDynamic(true);
        });
    }
}
