//package org.firstinspires.ftc.teamcode.commands;
//
//import com.arcrobotics.ftclib.command.InstantCommand;
//import com.arcrobotics.ftclib.command.WaitCommand;
//
//import org.firstinspires.ftc.teamcode.subsystems.Feeder;
//import org.firstinspires.ftc.teamcode.subsystems.Intake;
//import org.firstinspires.ftc.teamcode.subsystems.Shooter;
//
//public class ShovelOnlyCommand extends SequentialCommandGroup {
//    public ShovelOnlyCommand(Feeder feeder) {
////        addRequirements(feeder);
//
//        addCommands(
//            new SequentialCommandGroup(
//                new InstantCommand(feeder::setShovelUp),
//                new WaitCommand(200),
//                new InstantCommand(feeder::setShovelDown)
////                new InstantCommand(() -> {shooter.resetVelocityCount();})
//            )
//        );
//    }
//}
