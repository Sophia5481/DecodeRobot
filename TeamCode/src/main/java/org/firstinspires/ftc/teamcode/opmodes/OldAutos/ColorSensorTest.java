//package org.firstinspires.ftc.teamcode.tests;
//
//import android.util.Log;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.arcrobotics.ftclib.hardware.SimpleServo;
//import com.qualcomm.hardware.rev.RevColorSensorV3;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.NormalizedRGBA;
//
//import org.firstinspires.ftc.teamcode.subsystems.Intake;
//
//@Config
//@TeleOp(name = "ColorSensorTest", group = "Test")
//public class ColorSensorTest extends OpMode {
//    private Intake intake;
//
//    @Override
//    public void init() {
//        intake = new Intake(hardwareMap);
//    }
//
//    @Override
//    public void loop() {
//        if (intake.hasArtifact()) {
//            if (intake.isGreen()) {
//                telemetry.addLine("Green");
//                intake.spinLeft();
//            } else if (intake.isPurple()) {
//                telemetry.addLine("Purple");
//                intake.spinRight();
//            }
//        } else {
//            telemetry.addLine("None");
//            intake.stopSorting();
//        }
//    }
//}