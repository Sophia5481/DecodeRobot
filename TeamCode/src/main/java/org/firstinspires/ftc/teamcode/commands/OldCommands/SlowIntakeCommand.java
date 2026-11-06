//package org.firstinspires.ftc.teamcode.commands;
//
//import android.util.Log;
//
//import com.arcrobotics.ftclib.command.CommandBase;
//
//import org.firstinspires.ftc.teamcode.subsystems.Intake;
//import org.firstinspires.ftc.teamcode.subsystems.Transfer;
//
//// By default, IntakeCommand assumes that the robot is in auto
//// Thus, it will either do sorting or will intake without using the distance sensor
//public class SlowIntakeCommand extends CommandBase {
//    private Intake intake;
//
//    public SlowIntakeCommand(Intake intake) {
//        addRequirements(intake);
//        this.intake = intake;
//    }
//
//    @Override
//    public void initialize() {
//        intake.startIntake();
//        intake.wristIntake();
//        intake.motorSlow();
//    }
//
//    @Override
//    public void execute() {
//    }
//
//    @Override
//    public boolean isFinished() {
//        return !intake.isIntaking();
//    }
//
//    @Override
//    public void end(boolean interrupted) {
//        Log.d("IntakeCommand::interrupted", "interrupted");
//        intake.motorOff();
//        intake.stopSorting();
//        intake.wristStored();
//        intake.stopIntake();
//    }
//}