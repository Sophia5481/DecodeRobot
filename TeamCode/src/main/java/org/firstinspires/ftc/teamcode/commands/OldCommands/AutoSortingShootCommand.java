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
//public class AutoSortingShootCommand extends SequentialCommandGroup {
//    public AutoSortingShootCommand(Feeder feeder, Shooter shooter, Intake intake, Transfer transfer, String motif, boolean far) {
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
//                new SequentialCommandGroup(
//                    // FIRST CYCLE
//                    // Failsafe if somehow the ball is not in the shovel
//                    new ConditionalCommand(
//                        // if the shooter is full, continue.
//                        new InstantCommand(),
//                        // else, then feed the ball on the opposite side
//                        new ParallelRaceGroup(
//                            new FeedOneCommand(feeder, motif.charAt(0) != 'G'),
//                            new IntakeCommand(intake, transfer)
//                        ),
//                        feeder::shooterFull
//                    ),
//                    new InstantCommand(() -> {
//                        Log.d("ShootThreeCommand: ", "feeder ready 1");
//                    }),
//                    new WaitUntilCommand(shooter::isShooterReady).withTimeout(500),
//                    new InstantCommand(() -> {
//                        Log.d("motor velocity: ", "shooter ready 1");
//                    }),
//                    new ShovelOnlyCommand(feeder),
//                    new InstantCommand(() -> {
//                        Log.d("motor velocity: ", "shoot 1!");
//                    }),
//
//                    // SECOND CYCLE
//                    new ParallelRaceGroup(
//                        new FeedOneCommand(feeder, motif.charAt(1) == 'G'),
////                        new TransferCommand(transfer)
//                        new IntakeCommand(intake, transfer)
//                    ),
//
//                    // Failsafe if not sorted correctly
//                    new ConditionalCommand(
//                        // if the shooter is full, continue.
//                        new InstantCommand(),
//                        // else, then feed the ball on the opposite side
//                        new ParallelRaceGroup(
//                            new FeedOneCommand(feeder, motif.charAt(1) != 'G'),
//                            new IntakeCommand(intake, transfer)
//                        ),
//                        feeder::shooterFull
//                    ),
//
//                    new InstantCommand(() -> {
//                        Log.d("ShootThreeCommand: ", "feeder ready 2");
//                    }),
//                    new WaitUntilCommand(shooter::isShooterReady).withTimeout(500),
//                    new InstantCommand(() -> {
//                        Log.d("motor velocity: ", "shooter ready 2");
//                    }),
//                    new ShovelOnlyCommand(feeder),
//                    new InstantCommand(() -> {
//                        Log.d("motor velocity: ", "shoot 2!");
//                    }),
//
//                    // THIRD CYCLE
//                    new ParallelRaceGroup(
//                        new FeedOneCommand(feeder, motif.charAt(2) == 'G'),
////                        new TransferCommand(transfer)
//                        new IntakeCommand(intake, transfer)
//                    ),
//
//                    // Failsafe if not sorted correctly
//                    new ConditionalCommand(
//                        // if the shooter is full, continue.
//                        new InstantCommand(),
//                        // else, then feed the ball on the opposite side
//                        new ParallelRaceGroup(
//                            new FeedOneCommand(feeder, motif.charAt(2) != 'G'),
//                            new IntakeCommand(intake, transfer)
//                        ),
//                        feeder::shooterFull
//                    ),
//                    new InstantCommand(() -> {
//                        Log.d("ShootThreeCommand: ", "feeder ready 2");
//                    }),
//                    new WaitUntilCommand(shooter::isShooterReady).withTimeout(500),
//                    new InstantCommand(() -> {
//                        Log.d("motor velocity: ", "shooter ready 3");
//                    }),
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
