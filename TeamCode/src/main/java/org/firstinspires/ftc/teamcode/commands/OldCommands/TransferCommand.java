//package org.firstinspires.ftc.teamcode.commands;
//
//import android.util.Log;
//
//import com.arcrobotics.ftclib.command.CommandBase;
//
//import org.firstinspires.ftc.teamcode.subsystems.Intake;
//import org.firstinspires.ftc.teamcode.subsystems.Transfer;
//
//public class TransferCommand extends CommandBase {
//    private Transfer transfer;
//
//
//    public TransferCommand(Transfer transfer) {
//        addRequirements(transfer);
//        this.transfer = transfer;
//    }
//
//    @Override
//    public void initialize() {
//
//    }
//
//    @Override
//    public void execute() {
//        transfer.startTransfer();
//        //Log.d("CheckpointEvent: ", "StartedTransfer");
//    }
//
////    @Override
////    public boolean isFinished() {
////    }
//
//    @Override
//    public void end(boolean interrupted) {
//        transfer.stopTransfer();
//    }
//}
