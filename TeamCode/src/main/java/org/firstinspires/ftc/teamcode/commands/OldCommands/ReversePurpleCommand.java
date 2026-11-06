//package org.firstinspires.ftc.teamcode.commands;
//
//import com.arcrobotics.ftclib.command.CommandBase;
//
//import org.firstinspires.ftc.teamcode.subsystems.Intake;
//import org.firstinspires.ftc.teamcode.subsystems.Transfer;
//
//public class ReversePurpleCommand extends CommandBase {
//    private Transfer transfer;
//    private int millis;
//    private long timeLimit;
//
//    public ReversePurpleCommand(Transfer transfer, int millis) {
//        addRequirements(transfer);
//        this.transfer = transfer;
//        this.millis = millis;
//    }
//
//    @Override
//    public void initialize() {
//        timeLimit = System.currentTimeMillis() + millis;
//    }
//
//    @Override
//    public void execute() {
//        transfer.reversePurpleTransfer();
//    }
//
//    @Override
//    public boolean isFinished() {
//        return System.currentTimeMillis() > timeLimit;
//    }
//
//    @Override
//    public void end(boolean interrupted) {
//        transfer.stopTransfer();
//    }
//}
