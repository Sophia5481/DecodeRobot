//package org.firstinspires.ftc.teamcode.commands;
//
//import com.arcrobotics.ftclib.command.CommandBase;
//
//import org.firstinspires.ftc.teamcode.subsystems.Intake;
//import org.firstinspires.ftc.teamcode.subsystems.Transfer;
//
//public class ReverseTransferCommand extends CommandBase {
//    private Transfer transfer;
//    private int time = 500;
//
//    public ReverseTransferCommand(Transfer transfer) {
//        addRequirements(transfer);
//        this.transfer = transfer;
//    }
//
//    @Override
//    public void initialize() {
//    }
//
//    @Override
//    public void execute() {
//        transfer.reversePurpleTransfer();
//        transfer.reverseGreenTransfer();
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
