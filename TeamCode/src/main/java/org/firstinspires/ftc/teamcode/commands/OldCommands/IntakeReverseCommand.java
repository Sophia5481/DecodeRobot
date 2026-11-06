//package org.firstinspires.ftc.teamcode.commands;
//
//import static java.util.concurrent.TimeUnit.MILLISECONDS;
//
//import android.util.Log;
//
//import com.arcrobotics.ftclib.command.CommandBase;
//import com.arcrobotics.ftclib.util.Timing;
//
//import org.firstinspires.ftc.teamcode.subsystems.Intake;
//import org.firstinspires.ftc.teamcode.subsystems.Transfer;
//
//public class IntakeReverseCommand extends CommandBase {
//    private Intake intake;
//    private Transfer transfer;
//    private int time = 0;
//    private Timing.Timer timer;
//
//    public IntakeReverseCommand(Intake intake, Transfer transfer) {
//        addRequirements(intake);
//        this.intake = intake;
//        this.transfer = transfer;
//    }
//
//    public IntakeReverseCommand(Intake intake, Transfer transfer, int time) {
//        addRequirements(intake);
//        this.intake = intake;
//        this.transfer = transfer;
//        this.time = time;
//        timer = new Timing.Timer(time, MILLISECONDS);
//    }
//
//    @Override
//    public void initialize() {
//        intake.wristReverse();
//        intake.motorReverse();
//        if (timer != null) {
//            timer.start();
//        }
//    }
//
//    @Override
//    public void execute() {
//        transfer.reverseGreenTransfer();
//    }
//
//    @Override
//    public boolean isFinished() {
//        if (timer != null) {
//            Log.d("IntakeReverseCommand::isFinished() ", "done: " + timer.done()
//                + " time remaining: " + timer.remainingTime());
//            return timer.done();
//        } else {
//            return false;
//        }
//    }
//
//    @Override
//    public void end(boolean interrupted) {
//        intake.wristStored();
//        intake.motorOff();
//        transfer.stopTransfer();
//    }
//}