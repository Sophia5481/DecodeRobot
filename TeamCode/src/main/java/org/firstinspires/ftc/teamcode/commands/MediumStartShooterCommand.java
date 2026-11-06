package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;

// turns on the shooter and sets the lid angle without any control of the feeder system

// TODO: If this works well, integrate it into StartShooterCommand.
// Currently StartShooterCommand takes in a boolean whether it's near or far
// so changing it to take in 3 options is too much work for now.
public class MediumStartShooterCommand extends CommandBase {
    private Shooter shooter;
    private boolean far;

    public MediumStartShooterCommand(Shooter shooter) {
        addRequirements(shooter);
        this.shooter = shooter;
        this.far = far;
    }

    @Override
    public void initialize() {
        shooter.setStopShooting(false);
        shooter.lidMid();
    }

    @Override
    public void execute() {
        shooter.shootArtifactMediumPower();
    }

    @Override
    public boolean isFinished() {
        Log.d("StartShooterCommand::isFinished() ", "" + shooter.getStopShooting());
        return shooter.getStopShooting();
    }

    @Override
    public void end(boolean interrupted) {
//        shooter.shooterOff();
    }
}
