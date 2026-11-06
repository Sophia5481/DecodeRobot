package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

@TeleOp(name = "Vision AprilTag Test OpMode", group = "TeleOp")
public class VisionAprilTagTest extends LinearOpMode {
    private Limelight3A limelight = null;
    private LLResult result = null;

    @Override
    public void runOpMode() {
        telemetry.addLine("Initializing...");
        telemetry.update();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        waitForStart();

        if (opModeIsActive()) {
            limelight.pipelineSwitch(6);
            limelight.start();
        }

        while (opModeIsActive()) {
            result = limelight.getLatestResult();
            if (result != null) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    int id = fiducial.getFiducialId();
                    double x = fiducial.getTargetXDegrees();
                    double y = fiducial.getTargetYDegrees();
                    double StrafeDistance_3D = fiducial.getRobotPoseTargetSpace().getPosition().y; //robot pose relative to AprilTag coordinate system
                    double distance_3D_x = fiducial.getRobotPoseTargetSpace().getPosition().x;
                    double distance_3D_z = fiducial.getRobotPoseTargetSpace().getPosition().z;
                    telemetry.addData("x: ", x);
                    telemetry.addData("y: ", y);
                    telemetry.addData("StrafeDistance_3D: ", StrafeDistance_3D);
                    telemetry.addData("distance_3D_x: ", distance_3D_x);
                    telemetry.addData("distance_3D_z: ", distance_3D_z);
                    telemetry.update();
                }
            } else {
                telemetry.addData("Result: ", "null");
                telemetry.update();
            }
//            result = limelight.getLatestResult();
//
//            if (result != null) {
//                telemetry.addData("Pipeline: ", result.getPipelineIndex());
//                List<LLResultTypes.DetectorResult> detected =  result.getDetectorResults();
//                telemetry.addData("Detected objects: ", detected.size());
//                telemetry.update();
//            }
//            else {
//                telemetry.addData("Result: ", "null");
//                telemetry.update();}
        }
    }
}
