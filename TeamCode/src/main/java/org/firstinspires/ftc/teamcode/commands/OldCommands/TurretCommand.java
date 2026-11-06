package org.firstinspires.ftc.teamcode.commands.OldCommands;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Turret;

public class TurretCommand extends CommandBase {
    private Turret turret;
    private final double turnAmount = 10.0;
    private boolean right;

    public TurretCommand(Turret turret, boolean right) {
        addRequirements(turret);
        this.turret = turret;
        this.right = right;
    }

    @Override
    public void initialize() {
        turret.disable();
        if(!right){
            turret.turn(turnAmount);
        }else{
            turret.turn(-turnAmount);
        }
    }

    @Override
    public void execute() {
        Log.d("TurretCommand::execute:", "turned" + turnAmount + "to the" + (right ? "right" : "left"));
        turret.holdCurrentPosition();
    }
    @Override
    public boolean isFinished() {
        return !turret.isEnabled();
    }

    @Override
    public void end(boolean interrupted) {
        Log.d("TurretCommand::interrupted", "interrupted");
        turret.disable();
    }
}