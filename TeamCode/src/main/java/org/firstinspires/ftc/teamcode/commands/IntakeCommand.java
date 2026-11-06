package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

import java.util.function.BooleanSupplier;

public class IntakeCommand extends CommandBase {
    private Intake intake;

    private boolean reverse;
    private boolean slow;

    private boolean useDistanceSensor = false;

    public IntakeCommand(Intake intake, boolean reverse, boolean slow) {
        addRequirements(intake);
        this.intake = intake;
        this.reverse = reverse;
        this.slow = slow;
    }

    public IntakeCommand(Intake intake, boolean reverse, boolean slow, BooleanSupplier useDistanceSensor) {
        addRequirements(intake);
        this.intake = intake;
        this.reverse = reverse;
        this.slow = slow;
        this.useDistanceSensor = useDistanceSensor.getAsBoolean();
    }

    @Override
    public void initialize() {
        intake.closeLatch();
        if(reverse && slow){
            intake.reverseSlowIntake();
        }else if(reverse && !slow){
            intake.reverseIntake();
        }else if(!reverse && slow){
            intake.slowIntake();
        }else{
            intake.startIntake();
        }
        // TODO: determine if this breaks anything!
        intake.setHasThreeBalls(false);
    }

    @Override
    public void execute() {
        Log.d("IntakeAutoCommand::execute() ", "");
        if (useDistanceSensor) {
            if (intake.hasFirstArtifact() && intake.hasSecondArtifact() && intake.hasThirdArtifact()) {
                intake.stopIntake();
                intake.setHasThreeBalls(true);
            } /* else if (intake.hasFirstArtifact() && intake.hasSecondArtifact() && !intake.hasThirdArtifact()) {
                intake.stopTransfer();
            } */
            // TODO: Determine if we need to stop transfer early when we detect 2 balls
            // Previously the transfer stopped when we had 2 balls in the robot. However, the balls would slide down and would
        }
        intake.getTransferVelocity();
    }

    @Override
    public boolean isFinished() {
        return !intake.isIntaking();
    }

    @Override
    public void end(boolean interrupted) {
        Log.d("IntakeCommand::interrupted", "interrupted");
        intake.stopIntake();
    }
}