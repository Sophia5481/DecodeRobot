//package org.firstinspires.ftc.teamcode.commands;
//
//import android.util.Log;
//
//import com.arcrobotics.ftclib.command.CommandBase;
//
//import org.firstinspires.ftc.teamcode.subsystems.Intake;
//import org.firstinspires.ftc.teamcode.subsystems.Transfer;
//
//import java.util.function.BooleanSupplier;
//
//// By default, IntakeCommand assumes that the robot is in auto
//// Thus, it will either do sorting or will intake without using the distance sensor
//public class IntakeCommand extends CommandBase {
//    // assumes that the slides have already been pre-extended
//    private Intake intake;
//    private Transfer transfer;
//    private Intake.SampleColor color = Intake.SampleColor.NONE;
//    private long lastDetectionTime;
//    private boolean seenThirdArtifact = false;
//    private boolean useDistanceSensor = false;
//
//    public IntakeCommand(Intake intake, Transfer transfer) {
//        addRequirements(intake);
//        this.intake = intake;
//        this.transfer = transfer;
//    }
//
//    // TODO: Clean this up!
//    public IntakeCommand(Intake intake, Transfer transfer, BooleanSupplier useDistanceSensor) {
//        addRequirements(intake);
//        this.intake = intake;
//        this.transfer = transfer;
//        this.useDistanceSensor = useDistanceSensor.getAsBoolean();
//    }
//
//    public IntakeCommand(Intake intake, Transfer transfer, Boolean useDistanceSensor) {
//        addRequirements(intake);
//        this.intake = intake;
//        this.transfer = transfer;
//        this.useDistanceSensor = useDistanceSensor;
//    }
//
//    @Override
//    public void initialize() {
//        intake.startIntake();
//        intake.wristIntake();
//        if (intake.isSorting()) {
//            intake.motorOn();
//        } else {
//            intake.motorFullSpeed();
//        }
//        transfer.slowTransfer();
//    }
//
//    @Override
//    public void execute() {
//        Log.d("IntakeAutoCommand::execute() ", "");
//
//        // teleop: no sorting, spin to one side
//        // TODO: This new code is for the intake w/ hardware limit of 3 balls.
//        // Revert this if the hardware change ain't workin'.
//        if (!intake.isSorting()) {
//            intake.spinRight();
//            if (useDistanceSensor) {
////                boolean hasArtifactOnTransfer = intake.hasArtifactOnTransfer();
////                // use distance sensor to detect when we have 3 balls
////                if (hasArtifactOnTransfer && !seenThirdArtifact) {
////                    // record the time it first saw an artifact
////                    lastDetectionTime = System.currentTimeMillis();
////                    seenThirdArtifact = true;
////                } else if (!hasArtifactOnTransfer) {
////                    seenThirdArtifact = false;
////                } else if (hasArtifactOnTransfer
////                    && seenThirdArtifact
////                    && System.currentTimeMillis() - lastDetectionTime > 200) {
////                    // if it has seen an artifact for a consecutive amount of time
////                    if (intake.hasSecondArtifact() && intake.hasFirstArtifact()) {
////                        intake.stopIntake();
////                        intake.setHasThreeBalls(true);
////                    }
////                }
////                boolean intakeHasArtifact = intake.hasLastArtifact();
////                // use distance sensor to detect when we have 3 balls
////                if (intakeHasArtifact && !seenThirdArtifact) {
////                    // record the time it first saw an artifact
////                    lastDetectionTime = System.currentTimeMillis();
////                    seenThirdArtifact = true;
////                } else if (!intakeHasArtifact) {
////                    seenThirdArtifact = false;
////                } else if (intakeHasArtifact
////                    && seenThirdArtifact
////                    && System.currentTimeMillis() - lastDetectionTime > 150) {
////                    // if it has seen an artifact for a consecutive amount of time
////                    if (intake.hasArtifactOnTransfer() && intake.hasSecondArtifact()) {
////                        intake.stopIntake();
////                        intake.setHasThreeBalls(true);
////                    }
////                }
//                if (intake.hasArtifactOnTransfer() && intake.hasSecondArtifact() && intake.hasLastArtifact()) {
//                    intake.stopIntake();
//                    intake.setHasThreeBalls(true);
//                }
//            }
//        } else {
//            // auto: sorting
//            if (intake.hasArtifact()) {
//                lastDetectionTime = System.currentTimeMillis();
//                // read color sensor and set color
//                color = intake.getArtifactColor();
//                Log.d("IntakeAutoCommand::execute() ", "detected color: " + color);
//            }
//            switch(color) {
//                case GREEN:
//                    if (System.currentTimeMillis() < lastDetectionTime + 175) {
//                        Log.d("IntakeAutoCommand::execute() ", "spin left");
//                        intake.spinLeft();
//                    } else {
//                        intake.stopSorting();
//                    }
//                    break;
//                case PURPLE:
//                    if (System.currentTimeMillis() < lastDetectionTime + 175) {
//                        Log.d("IntakeAutoCommand::execute() ", "spin right");
//                        intake.spinRight();
//                    } else {
//                        intake.stopSorting();
//                    }
//                    break;
//                case NONE:
//                    Log.d("IntakeAutoCommand::execute() ", "stop spin");
//                    intake.stopSorting();
//                    break;
//            }
//        }
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
//        transfer.stopTransfer();
//    }
//}