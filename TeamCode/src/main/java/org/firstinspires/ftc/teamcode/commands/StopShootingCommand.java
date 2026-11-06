package org.firstinspires.ftc.teamcode.commands;

import static java.lang.Math.abs;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;

// turns on the shooter and sets the lid angle without any control of the feeder system
public class StopShootingCommand extends InstantCommand {
    public StopShootingCommand(Shooter shooter) {
        super(shooter::shooterOff, shooter);
    }
}
