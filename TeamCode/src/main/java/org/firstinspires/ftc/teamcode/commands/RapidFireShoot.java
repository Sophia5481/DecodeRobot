package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

public class RapidFireShoot extends SequentialCommandGroup {
    private boolean far;

    public RapidFireShoot(Intake intake, Shooter shooter, boolean far) {
        addRequirements(shooter);
        addRequirements(intake);
        this.far = far;
        Log.d("RapidFireShoot: ", "far: " + this.far);

        addCommands(
            new InstantCommand(() -> {
                shooter.setStopShooting(false);
            }),
            // TODO: Verify if these new intake code changes work
//            new IntakeCommand(intake, false, false).interruptOn(shooter::getStopShooting),
            new StartShooterCommand(shooter, this.far),
            new WaitCommand(50),
            new WaitUntilCommand(shooter::isShooterReady).withTimeout(500),
            new ParallelCommandGroup(
                // TODO: Determine if far side needs slower intake speed
                new IntakeCommand(intake, false, true).interruptOn(shooter::getStopShooting),
                new SequentialCommandGroup(
                    new InstantCommand(() -> {
                        Log.d("Shooter::RapidFireShoot: ", "shooter ready 1");
                    }),
                    new InstantCommand(intake::openLatch),
                    new InstantCommand(() -> {
                        Log.d("Shooter::RapidFireShoot ", "CheckpointEvent: latchOpen");
                    }),
                    new WaitCommand(300),
                    new InstantCommand(() -> {
                        shooter.setStopShooting(true);
                    }),
                    new TeleopStopShootingCommand(shooter),
                    new InstantCommand(intake::stopIntake)
                )
            ),
            new InstantCommand(() -> {
                Log.d("RapidFireShoot: ", "finished");
                Log.d("Shooter::setVelocity", "CheckpointEvent: RapidFireFinished");
            })
        );
    }
}
