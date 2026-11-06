//package org.firstinspires.ftc.teamcode.commands;
//
//import android.util.Log;
//
//import com.arcrobotics.ftclib.command.InstantCommand;
//import com.arcrobotics.ftclib.command.ParallelCommandGroup;
//import com.arcrobotics.ftclib.command.WaitCommand;
//import com.arcrobotics.ftclib.command.WaitUntilCommand;
//
//import org.firstinspires.ftc.teamcode.subsystems.Feeder;
//import org.firstinspires.ftc.teamcode.subsystems.Shooter;
//import org.firstinspires.ftc.teamcode.subsystems.Transfer;
//
//// for shooting at the farther side of the near zone, in case we are being pushed and defensed.
//public class MediumShootCommand extends SequentialCommandGroup {
//
//    public MediumShootCommand(Feeder feeder, Shooter shooter, Transfer transfer) {
//        addRequirements(feeder);
//        addRequirements(shooter);
//        addRequirements(transfer);
//
//        addCommands(
//            new InstantCommand(() -> {
//                feeder.setShovelDown();
//            }),
//            new ParallelCommandGroup(
//                new InstantCommand(() -> {
//                    shooter.setStopShooting(false);
//                }),
//                new MediumStartShooterCommand(shooter),
//                new TransferCommand(transfer).interruptOn(shooter::getStopShooting),
//                new SequentialCommandGroup(
//                    new WaitCommand(100),
//                    new WaitUntilCommand(shooter::isShooterReady),
//                    new InstantCommand(() -> {Log.d("RapidFireShoot: ", "shooter ready 1");}),
//                    new InstantCommand(feeder::shovelRapidFire),
//                    // original value was 1500, but changed it to 1200 to speed up autos
//                    new WaitCommand(1200),
//                    new InstantCommand(() -> {
//                        shooter.setStopShooting(true);
//                    }),
//                    new InstantCommand(shooter::shooterOffTele),
//                    new InstantCommand(feeder::setShovelDown)
//                )
//            ),
//            new InstantCommand(() -> {Log.d("RapidFireShoot: ", "finished");})
//        );
//    }
//}
