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
//public class FeedOneCommand extends SequentialCommandGroup {
//    public FeedOneCommand(Feeder feeder, boolean green) {
////        addRequirements(feeder);
//
//        addCommands(
//            new SequentialCommandGroup(
//                green ? new InstantCommand(feeder::feedGreenArtifact) : new InstantCommand(feeder::feedPurpleArtifact),
//                new WaitUntilCommand(feeder::shooterFull).withTimeout(750),
//                green ? new InstantCommand(feeder::closeGreenFeeder) : new InstantCommand(feeder::closePurpleFeeder)
//            )
//        );
//    }
//}
