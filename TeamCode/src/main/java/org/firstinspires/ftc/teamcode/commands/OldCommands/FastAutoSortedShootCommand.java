//package org.firstinspires.ftc.teamcode.commands;
//
//import android.util.Log;
//
//import com.arcrobotics.ftclib.command.ConditionalCommand;
//import com.arcrobotics.ftclib.command.InstantCommand;
//import com.arcrobotics.ftclib.command.ParallelCommandGroup;
//import com.arcrobotics.ftclib.command.ParallelRaceGroup;
//import com.arcrobotics.ftclib.command.WaitCommand;
//import com.arcrobotics.ftclib.command.WaitUntilCommand;
//
//import org.firstinspires.ftc.teamcode.subsystems.Feeder;
//import org.firstinspires.ftc.teamcode.subsystems.Intake;
//import org.firstinspires.ftc.teamcode.subsystems.Shooter;
//import org.firstinspires.ftc.teamcode.subsystems.Transfer;
//
//// the same as SortingShootCommand, but assumes that there is already a ball of the correct color in the feeder
//public class FastAutoSortedShootCommand extends SequentialCommandGroup {
//    public FastAutoSortedShootCommand(Feeder feeder, Shooter shooter, Intake intake, Transfer transfer, String motif, boolean far) {
//        addRequirements(feeder);
//        addRequirements(shooter);
//        addRequirements(intake);
//        addRequirements(transfer);
//
//        addCommands(
//            new ParallelCommandGroup(
//                new InstantCommand(() -> {
//                    shooter.setStopShooting(false);
//                }),
//                new StartShooterCommand(shooter, far),
//                new TransferCommand(transfer).interruptOn(shooter::getStopShooting),
//                new SequentialCommandGroup(
//                    // FIRST CYCLE
//                    new WaitUntilCommand(shooter::isShooterReady).withTimeout(500),
//                    new InstantCommand(() -> {
//                        Log.d("motor velocity: ", "shooter ready 1");
//                    }),
//                    new InstantCommand(feeder::shovelRapidFire),
//                    new InstantCommand(() -> {
//                        Log.d("motor velocity: ", "shoot 1!");
//                    }),
//
//                    new WaitCommand(300),
//
//                    // SECOND CYCLE
//                    new InstantCommand(() -> {
//                        if (motif.charAt(1) == 'G') {
//                            feeder.feedGreenArtifact();
//                            feeder.setShovelDown();
//                        } else {
//                            feeder.feedPurpleArtifact();
//                        }
//                    }),
//
//                    new WaitCommand(motif.charAt(1) == 'G' ? 500 : 300),
//
//                    // THIRD CYCLE
//                    new InstantCommand(() -> {
//                        if (motif.charAt(2) == 'G') {
//                            feeder.closePurpleFeeder();
//                            feeder.feedGreenArtifact();
//                            feeder.setShovelDown();
//                        } else {
//                            feeder.shovelRapidFire();
//                            feeder.closeGreenFeeder();
//                            feeder.feedPurpleArtifact();
//                        }
//                    }),
//                    new WaitCommand(600),
//
//                    new InstantCommand(feeder::shovelRapidFire),
//
//                    new ConditionalCommand(
//                        new WaitCommand(500),
//                        new InstantCommand(),
//                        () -> motif.charAt(2) == 'G'
//                    ),
//
//                    new InstantCommand(() -> {
//                        shooter.setStopShooting(true);
//                    }),
//                    new InstantCommand(shooter::shooterOffTele),
//                    new InstantCommand(feeder::setShovelDown)
//                )
//            )
//        );
//    }
//}
