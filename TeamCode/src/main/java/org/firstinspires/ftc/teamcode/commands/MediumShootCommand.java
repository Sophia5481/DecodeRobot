package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

public class MediumShootCommand extends SequentialCommandGroup {
    public MediumShootCommand(Shooter shooter, Intake intake) {
        addRequirements(shooter);
        addRequirements(intake);

        addCommands(
                new ParallelCommandGroup(
                        new InstantCommand(() -> {
                            shooter.setStopShooting(false);
                        }),
                        new MediumStartShooterCommand(shooter),
                        new IntakeCommand(intake, false, false).interruptOn(shooter::getStopShooting),
                        new SequentialCommandGroup(
                                new WaitCommand(100),
                                new WaitUntilCommand(shooter::isShooterReady),
                                new InstantCommand(() -> {
                                    Log.d("RapidFireShoot: ", "shooter ready 1");}),
                                new InstantCommand(intake::openLatch),
                                // original value was 1500, but changed it to 1200 to speed up autos
                                new WaitCommand(1200),
                                new InstantCommand(() -> {
                                    shooter.setStopShooting(true);
                                }),
                                new InstantCommand(shooter::shooterOffTele),
                                new InstantCommand(intake::stopIntake)
                        )
                ),
                new InstantCommand(() -> {Log.d("RapidFireShoot: ", "finished");})
        );
    }
}
