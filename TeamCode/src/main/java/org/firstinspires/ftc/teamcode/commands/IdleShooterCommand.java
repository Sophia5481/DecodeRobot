package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;

// turns on the shooter and sets the lid angle without any control of the feeder system
public class IdleShooterCommand extends CommandBase {
    private Shooter shooter;
    private boolean far;

    public IdleShooterCommand(Shooter shooter, boolean far) {
        addRequirements(shooter);
        this.shooter = shooter;
        this.far = far;
    }

    @Override
    public void execute() {
        Log.d("IdleShooterCommand::execute() ", "far: " + far);
        if (far) {
            shooter.shooterIdleFarVelocity();
        } else {
            shooter.shooterIdleNearVelocity();
        }
    }
}
