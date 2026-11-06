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
//import org.firstinspires.ftc.teamcode.subsystems.Intake;
//import org.firstinspires.ftc.teamcode.subsystems.Shooter;
//import org.firstinspires.ftc.teamcode.subsystems.Transfer;
//
//public class RapidFireShoot extends SequentialCommandGroup {
//    private boolean far;
//
//    public RapidFireShoot(Feeder feeder, Shooter shooter, Transfer transfer, boolean far) {
//        addRequirements(feeder);
//        addRequirements(shooter);
//        addRequirements(transfer);
//        this.far = far;
//        Log.d("RapidFireShoot: ", "far: " + this.far);
//
//        addCommands(
//            new InstantCommand(() -> {
//                feeder.setShovelDown();
//                feeder.feedPurpleArtifact();
//            }),
//            new ParallelCommandGroup(
//                new InstantCommand(() -> {
//                    shooter.setStopShooting(false);
//                }),
//                new StartShooterCommand(shooter, this.far),
//                new TransferCommand(transfer).interruptOn(shooter::getStopShooting),
//                new SequentialCommandGroup(
//                    new WaitCommand(50),
//                    new WaitUntilCommand(shooter::isShooterReady).withTimeout(500),
//                    new InstantCommand(() -> {Log.d("ManualDrive::DPAD RapidFireShoot: ", "shooter ready 1");}),
//                    new InstantCommand(feeder::shovelRapidFire),
//                    //new InstantCommand(() -> {Log.d("CheckpointEvent", "shovelRapidFireFinished");}),
//                    // original value was 1500, but changed it to 1200 to speed up autos
//                    // TODO: Now it's changed to 800. Verify if this really works.
//                    new WaitCommand(1200 /*800*/),
//                    new InstantCommand(() -> {
//                        shooter.setStopShooting(true);
//                    }),
//                    new InstantCommand(shooter::shooterOffTele),
//                    new InstantCommand(feeder::setShovelDown)
//                )
//            ),
//            new InstantCommand(() -> {
//                Log.d("RapidFireShoot: ", "finished");
//                //Log.d("CheckpointEvent: ", "RapidFireFinished");
//            })
//        );
//    }
//
////    // use this command if you want to set a custom speed for the shooter
////    public RapidFireShoot(Feeder feeder, Shooter shooter, Transfer transfer, int speed) {
////        addRequirements(feeder);
////        addRequirements(shooter);
////        addRequirements(transfer);
////        this.far = far;
////        Log.d("RapidFireShoot: ", "far: " + this.far);
////
////        addCommands(
////            new InstantCommand(() -> {
////                feeder.setShovelDown();
////            }),
////            new ParallelCommandGroup(
////                new InstantCommand(() -> {
////                    shooter.setStopShooting(false);
////                }),
////                new StartShooterCommand(shooter, speed),
////                new TransferCommand(transfer).interruptOn(shooter::getStopShooting),
////                new SequentialCommandGroup(
////                    new WaitCommand(100),
////                    new WaitUntilCommand(shooter::isShooterReady),
////                    new InstantCommand(() -> {Log.d("ManualDrive::DPAD RapidFireShoot: ", "shooter ready 1");}),
////                    new InstantCommand(feeder::shovelRapidFire),
////                    //new InstantCommand(() -> {Log.d("CheckpointEvent", "shovelRapidFireFinished");}),
////                    // original value was 1500, but changed it to 1200 to speed up autos
////                    // TODO: Now it's changed to 800. Verify if this really works.
////                    new WaitCommand(1200 /*800*/),
////                    new InstantCommand(() -> {
////                        shooter.setStopShooting(true);
////                    }),
////                    new InstantCommand(shooter::shooterOffTele),
////                    new InstantCommand(feeder::setShovelDown)
////                )
////            ),
////            new InstantCommand(() -> {
////                Log.d("RapidFireShoot: ", "finished");
////                //Log.d("CheckpointEvent: ", "RapidFireFinished");
////            })
////        );
////    }
//}
