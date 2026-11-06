//package org.firstinspires.ftc.teamcode.commands;
//
//import android.util.Log;
//
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
//public class ShootThreeCommand extends SequentialCommandGroup {
//    private boolean far;
//
//    public ShootThreeCommand(Feeder feeder, Shooter shooter, Transfer transfer, boolean far) {
//        addRequirements(feeder);
//        addRequirements(shooter);
//        addRequirements(transfer);
//        this.far = far;
//        Log.d("ShootThreeCommand: ", "far: " + this.far);
//
//        addCommands(
//            new InstantCommand(() -> {
//                feeder.setShovelDown();
//            }),
//            new ParallelCommandGroup(
//                new InstantCommand(() -> {
//                    shooter.setStopShooting(false);
//                }),
//                new StartShooterCommand(shooter, this.far),
//                new TransferCommand(transfer).interruptOn(shooter::getStopShooting),
//                new SequentialCommandGroup(
//                    new WaitCommand(200),
//                    new WaitUntilCommand(shooter::isShooterReady),
//                    new InstantCommand(() -> {Log.d("ShootThreeCommand: ", "shooter ready 1");}),
//                    new ShovelOnlyCommand(feeder),
//
//                    // prevent the feeder sensor from seeing the shovel and thinking it has a ball
//                    new WaitCommand(200),
//                    new WaitUntilCommand(feeder::shooterFull).withTimeout(200),
//
//                    new InstantCommand(() -> {Log.d("ShootThreeCommand: ", "feeder ready 2");}),
//                    new WaitUntilCommand(shooter::isShooterReady),
//                    new InstantCommand(() -> {Log.d("ShootThreeCommand: ", "shooter ready 2");}),
//                    new ShovelOnlyCommand(feeder),
//
//                    // prevent the feeder sensor from seeing the shovel and thinking it has a ball
//                    new WaitCommand(200),
//                    new WaitUntilCommand(feeder::shooterFull).withTimeout(200),
//                    new InstantCommand(() -> {Log.d("ShootThreeCommand: ", "feeder ready 3");}),
//
//                    new WaitUntilCommand(shooter::isShooterReady),
//                    new InstantCommand(() -> {Log.d("ShootThreeCommand: ", "shooter ready 3");}),
//                    new ShovelOnlyCommand(feeder),
//                    new WaitCommand(500),
//                    new InstantCommand(() -> {
//                        shooter.setStopShooting(true);
//                    })
//                )
//            ),
//            new InstantCommand(() -> {Log.d("ShootThreeCommand: ", "finished");})
//        );
//    }
//}
