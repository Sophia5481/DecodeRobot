//package org.firstinspires.ftc.teamcode.subsystems;
//
//import com.arcrobotics.ftclib.command.SubsystemBase;
//import com.arcrobotics.ftclib.hardware.motors.CRServo;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//public class Transfer extends SubsystemBase {
//    private final CRServo transferLeft;
//    private final CRServo transferRight;
//    private final CRServo purpleTransfer;
//
//    public Transfer(HardwareMap hMap) {
//        transferLeft = new CRServo(hMap, "transferLeft");
//        transferRight = new CRServo(hMap, "transferRight");
//        purpleTransfer = new CRServo(hMap, "purpleTransfer");
//    }
//
//    public void init() {
//        transferLeft.set(0);
//        transferRight.set(0);
//        purpleTransfer.set(0);
//    }
//
//    public void startTransfer() {
////        transferRight.set(-1.0);
//        transferLeft.set(1.0);
//        purpleTransfer.set(1.0);
//    }
//
//    public void slowTransfer() {
//        transferRight.set(-1.0);
//        transferLeft.set(1.0);
//        purpleTransfer.set(0.15);
//    }
//
//    public void stopTransfer() {
//        transferRight.set(0.0);
//        transferLeft.set(0.0);
//        purpleTransfer.set(0.0);
//    }
//
//    public void stopPurpleTransfer() {
//        transferLeft.set(0.0);
//    }
//
//    public void stopGreenTransfer() {
//        transferRight.set(0.0);
//    }
//
//    public void startPurpleTransfer() {
//        transferLeft.set(1.0);
//    }
//
//    public void startGreenTransfer() {
//        transferRight.set(-1.0);
//    }
//
//    public void reversePurpleTransfer() {
//        transferLeft.set(-1.0);
//    }
//
//    public void reverseGreenTransfer() {
//        transferRight.set(1.0);
//    }
//
//    // TODO: Add time to this if needed.
//
//    //transfer one artifact
////    public void transferOneArtifact() {
////        long time = System.currentTimeMillis();
////        long finalTime = System.currentTimeMillis() + 500;
////        while (time < finalTime) {
////            time = System.currentTimeMillis();
////            transferLeft.set(1.0);
////            transferRight.set(1.0);
////        }
////    }
////
////    public void transferArtifactTimed(int numOfArtifacts) {
////        long time = System.currentTimeMillis();
////        long finalTime = System.currentTimeMillis() + 500*numOfArtifacts;
////        while (time < finalTime) {
////            time = System.currentTimeMillis();
////            transferLeft.set(1.0);
////            transferRight.set(1.0);
////        }
////    }
//}
