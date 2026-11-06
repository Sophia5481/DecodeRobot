//package org.firstinspires.ftc.teamcode.commands;
//
//import com.arcrobotics.ftclib.command.CommandBase;
//
//import org.firstinspires.ftc.teamcode.subsystems.Transfer;
//
//public class TransferCommandGreen extends CommandBase {
//    private Transfer transfer;
//
//
//    public TransferCommandGreen(Transfer transfer) {
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
//        transfer.startGreenTransfer();
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
