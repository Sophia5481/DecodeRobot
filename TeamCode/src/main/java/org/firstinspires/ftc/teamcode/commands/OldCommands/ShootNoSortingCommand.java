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
//public class ShootNoSortingCommand extends SequentialCommandGroup {
//    public ShootNoSortingCommand(Feeder feeder, Shooter shooter, Intake intake, Transfer transfer, boolean far) {
//        addRequirements(feeder);
//        addRequirements(shooter);
//        addRequirements(intake);
//        addRequirements(transfer);
//
//        addCommands(
//                new InstantCommand(() -> {
//                    feeder.setShovelDown();
//                }),
//                new ParallelCommandGroup(
//                        new InstantCommand(() -> {
//                            shooter.setStopShooting(false);
//                        }),
//                        new StartShooterCommand(shooter, far),
//                        new SequentialCommandGroup(
//                                // FIRST CYCLE
//                                new ParallelRaceGroup(
//                                        new FeedOneCommand(feeder, false),
//                                        new IntakeCommand(intake, transfer)
//                                ),
//                                new InstantCommand(() -> {
//                                    Log.d("ShootThreeCommand: ", "feeder ready 1");
//                                }),
//
//                                new WaitUntilCommand(shooter::isShooterReady).withTimeout(1000),
//
//                                new InstantCommand(() -> {
//                                    Log.d("motor velocity: ", "shooter ready 1");
//                                }),
//                                new ShovelOnlyCommand(feeder),
//                                new InstantCommand(() -> {
//                                    Log.d("motor velocity: ", "shoot 1!");
//                                }),
//
//                                // SECOND CYCLE
//                                new ParallelRaceGroup(
//                                        new FeedOneCommand(feeder, false),
//                                        new IntakeCommand(intake, transfer)
//                                ),
//
//                                new InstantCommand(() -> {
//                                    Log.d("ShootThreeCommand: ", "feeder ready 2");
//                                }),
//                                new WaitUntilCommand(shooter::isShooterReady).withTimeout(1000),
//                                new InstantCommand(() -> {
//                                    Log.d("motor velocity: ", "shooter ready 2");
//                                }),
//                                new ShovelOnlyCommand(feeder),
//                                new InstantCommand(() -> {
//                                    Log.d("motor velocity: ", "shoot 2!");
//                                }),
//
//                                // THIRD CYCLE
//                                new ParallelRaceGroup(
//                                        new FeedOneCommand(feeder, false),
//                                        new IntakeCommand(intake, transfer)
//                                ),
//
//                                new InstantCommand(() -> {
//                                    Log.d("ShootThreeCommand: ", "feeder ready 2");
//                                }),
//                                new WaitUntilCommand(shooter::isShooterReady).withTimeout(1000),
//                                new InstantCommand(() -> {
//                                    Log.d("motor velocity: ", "shooter ready 3");
//                                }),
//                                new ShovelOnlyCommand(feeder),
//                                new InstantCommand(() -> {
//                                    Log.d("motor velocity: ", "shoot 3!");
//                                }),
//                                new WaitCommand(500),
//                                new InstantCommand(() -> {
//                                    shooter.setStopShooting(true);
//                                })
//                        )
//                )
//        );
//    }
//}
