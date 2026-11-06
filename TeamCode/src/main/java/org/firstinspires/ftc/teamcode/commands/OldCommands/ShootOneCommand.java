//package org.firstinspires.ftc.teamcode.commands;
//
//import com.arcrobotics.ftclib.command.InstantCommand;
//import com.arcrobotics.ftclib.command.ParallelCommandGroup;
//import com.arcrobotics.ftclib.command.WaitCommand;
//import com.arcrobotics.ftclib.command.WaitUntilCommand;
//
//import org.firstinspires.ftc.teamcode.subsystems.Feeder;
//import org.firstinspires.ftc.teamcode.subsystems.Shooter;
//
//public class ShootOneCommand extends SequentialCommandGroup {
//    public ShootOneCommand(Shooter shooter, Feeder feeder, boolean far) {
//        addRequirements(shooter);
//        addRequirements(feeder);
//
//        addCommands(
//            new InstantCommand(() -> {
//                feeder.setShovelDown();
//            }),
//            new ParallelCommandGroup(
//                new StartShooterCommand(shooter, far),
//                new SequentialCommandGroup(
//                    new WaitUntilCommand(shooter::isShooterReady),
//                    new InstantCommand(feeder::setShovelUp),
//                    new WaitCommand(300),
//                    new InstantCommand(feeder::setShovelDown)
//                )
//            )
//        );
//    }
//}
