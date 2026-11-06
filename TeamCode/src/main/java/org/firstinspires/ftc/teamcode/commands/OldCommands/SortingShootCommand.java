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
//public class SortingShootCommand extends SequentialCommandGroup {
//    public SortingShootCommand(Feeder feeder, Shooter shooter, Intake intake, Transfer transfer, String motif, boolean far) {
//        // todo: THIS IS A HACKY FIX THAT LETS US SHOVEL AND OPEN LATCH AT THE SAME TIME
//        // REWRITE THE CODE AND SPLIT UP THE FEEDRE SUBSYTSEM IF POSSIBLE
////        addRequirements(feeder);
//        addRequirements(shooter);
//        addRequirements(intake);
//        addRequirements(transfer);
//
//        addCommands(
//            new InstantCommand(feeder::setShovelDown),
//            new ParallelCommandGroup(
//                new InstantCommand(() -> {
//                    shooter.setStopShooting(false);
//                }),
//                new StartShooterCommand(shooter, far),
//                new SequentialCommandGroup(
//                    // FIRST CYCLE
//                    new InstantCommand(() -> {
//                        Log.d("SortingShootCommand motif: ", motif);
//                    }),
//                    new ParallelRaceGroup(
//                        new FeedOneCommand(feeder, motif.charAt(0) == 'G'),
////                        new TransferCommand(transfer)
//                        new IntakeCommand(intake, transfer)
//                    ),
//
//                    // Failsafe if not sorted correctly
//                    new ConditionalCommand(
//                        // if the shooter is full, continue.
//                        new InstantCommand(() -> {Log.d("ShootThreeCommand: ", "feeder detected it"); }),
//                        // else, then feed the ball on the opposite side
//                        new ParallelRaceGroup(
//                            new FeedOneCommand(feeder, motif.charAt(0) != 'G'),
//                            new ParallelCommandGroup(
//                                new InstantCommand(() -> {
//                                    Log.d("ShootThreeCommand: ", "feeder didn't detect" + feeder.shooterFull());
//                                }),
//                                new IntakeCommand(intake, transfer)
//                            )
//                        ),
//                        feeder::shooterFull
//                    ),
//                    new InstantCommand(() -> {
//                        Log.d("ShootThreeCommand: ", "feeder ready 1");
//                    }),
//                    new WaitUntilCommand(shooter::isShooterReady).withTimeout(1000),
//                    new InstantCommand(() -> {
//                        Log.d("motor velocity: ", "shooter ready 1");
//                    }),
//                    new ParallelCommandGroup(
//                        new ShovelOnlyCommand(feeder),
//                        new InstantCommand(() -> {
//                            Log.d("motor velocity: ", "shoot 1!");
//                        }),
//
//                        // SECOND CYCLE
//                        new ParallelRaceGroup(
//                            // need to wait for shovel to go down before detecting if there's a ball
//                            new SequentialCommandGroup(
//                                new WaitCommand(400),
//                                new FeedOneCommand(feeder, motif.charAt(1) == 'G')
//                            ),
////                        new TransferCommand(transfer)
//                            new IntakeCommand(intake, transfer)
//                        )
//                    ),
//
//                    // Failsafe if not sorted correctly
//                    new ConditionalCommand(
//                        // if the shooter is full, continue.
//                        new InstantCommand(() -> {Log.d("ShootThreeCommand: ", "feeder detected it"); }),
//                        // else, then feed the ball on the opposite side
//                        new ParallelRaceGroup(
//                            new FeedOneCommand(feeder, motif.charAt(1) != 'G'),
//                            new ParallelCommandGroup(
//                                new InstantCommand(() -> {
//                                    Log.d("ShootThreeCommand: ", "feeder didn't detect" + feeder.shooterFull());
//                                }),
//                                new IntakeCommand(intake, transfer)
//                            )
//                        ),
//                        feeder::shooterFull
//                    ),
//
//                    new InstantCommand(() -> {
//                        Log.d("ShootThreeCommand: ", "feeder ready 2");
//                    }),
//
//                    new ParallelCommandGroup(
//                        new ShovelOnlyCommand(feeder),
//                        new InstantCommand(() -> {
//                            Log.d("motor velocity: ", "shoot 2!");
//                        }),
//
//                        // THIRD CYCLE
//                        new ParallelRaceGroup(
//                            // need to wait for shovel to go down before detecting if there's a ball
//                            new SequentialCommandGroup(
//                                new WaitCommand(400),
//                                new FeedOneCommand(feeder, motif.charAt(2) == 'G')
//                            ),
////                        new TransferCommand(transfer)
//                            new IntakeCommand(intake, transfer)
//                        )
//                    ),
//                    // Failsafe if not sorted correctly
//                    new ConditionalCommand(
//                        // if the shooter is full, continue.
//                        new InstantCommand(() -> {Log.d("ShootThreeCommand: ", "feeder detected it"); }),
//                        // else, then feed the ball on the opposite side
//                        new ParallelRaceGroup(
//                            new FeedOneCommand(feeder, motif.charAt(2) != 'G'),
//                            new ParallelCommandGroup(
//                                new InstantCommand(() -> {
//                                    Log.d("ShootThreeCommand: ", "feeder didn't detect" + feeder.shooterFull());
//                                }),
//                                new IntakeCommand(intake, transfer)
//                            )
//                        ),
//                        feeder::shooterFull
//                    ),
//                    new InstantCommand(() -> {
//                        Log.d("ShootThreeCommand: ", "feeder ready 3");
//                    }),
//
//                    new ShovelOnlyCommand(feeder),
//                    new InstantCommand(() -> {
//                        Log.d("motor velocity: ", "shoot 3!");
//                    }),
//                    new WaitCommand(500),
//                    new InstantCommand(() -> {
//                        shooter.setStopShooting(true);
//                    })
//                )
//            )
//        );
//    }
//}
