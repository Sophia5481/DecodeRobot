package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

public class DynamicRapidFireShoot extends SequentialCommandGroup {
    private boolean far;

    public DynamicRapidFireShoot(Intake intake, Shooter shooter) {
        addRequirements(shooter);
        addRequirements(intake);

        addCommands(
            new InstantCommand(() -> {
                shooter.setStopShooting(false);
            }),
            // TODO: Verify if these new intake code changes work
//                        new IntakeCommand(intake, false, false).interruptOn(shooter::getStopShooting),
            new DynamicStartShooterCommand(shooter),
            new WaitCommand(50),
            // TODO: Warning! Used to be 200! If shooting is less accurate then change it back!
            new WaitUntilCommand(shooter::isShooterReady).withTimeout(50/*200*/),
            new ParallelCommandGroup(
                // for testing purposes, make transfer speed 0.7
                new IntakeCommand(intake, false, true).interruptOn(shooter::getStopShooting),
                new SequentialCommandGroup(
                    new InstantCommand(() -> {
                        Log.d("Shooter::RapidFireShoot: ", "shooter ready 1");
                    }),
                    new InstantCommand(intake::openLatch),
                    new InstantCommand(() -> {
                        Log.d("Shooter::RapidFireShoot: ", "CheckpointEvent: latchOpen");
                    }),
                    // TODO: Now it's changed to 400. Verify if this really works.
                    new WaitCommand(550), //latch is dying
                    new InstantCommand(() -> {
                        Log.d("Shooter::RapidFireShoot: ", "shooterVelocity = " + shooter.getVelocity());
                        Log.d("Shooter::RapidFireShoot: ", "lidAngle = " + shooter.getLidAngle());
                    }),
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
