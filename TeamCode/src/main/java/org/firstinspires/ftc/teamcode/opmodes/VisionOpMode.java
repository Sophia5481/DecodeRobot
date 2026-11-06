package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.subsystems.ArtifactVision;
import org.firstinspires.ftc.teamcode.vision.VisionLocalizer;
import org.firstinspires.ftc.teamcode.vision.VisionArtifactDetector2;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

@TeleOp(name = "Vision OpMode (Localizer + Artifacts)", group = "Vision")
public class VisionOpMode extends LinearOpMode {

    private static final double HEADING_OFFSET = Math.PI;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private ArtifactVision.ArtifactProcessor artifactProcessor;
    private VisionLocalizer localizer;
    private IMU imu;

    @Override
    public void runOpMode() {
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));

        Mat cameraMatrix = VisionLocalizer.getArducamMatrix();
        MatOfDouble distCoeffs = VisionLocalizer.getArducamDistCoeffs();
        localizer = new VisionLocalizer(VisionLocalizer.DECODE_FIELD_MAP, cameraMatrix, distCoeffs, VisionLocalizer.ARDUCAM_TRANSFORM);

        aprilTag = new AprilTagProcessor.Builder().build();
        artifactProcessor = new ArtifactVision.ArtifactProcessor();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new android.util.Size(1920, 1080))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(aprilTag)
                .addProcessor(artifactProcessor)
                .build();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.startCameraStream(visionPortal, 0);
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.clearAll();

            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            List<VisionLocalizer.Detection> localizerDetections = new ArrayList<>();
            for (AprilTagDetection det : currentDetections) {
                double scale = 1280.0 / 1920.0;
                Point[] corners = new Point[]{
                        new Point(det.corners[3].x * scale, det.corners[3].y * scale),
                        new Point(det.corners[2].x * scale, det.corners[2].y * scale),
                        new Point(det.corners[1].x * scale, det.corners[1].y * scale),
                        new Point(det.corners[0].x * scale, det.corners[0].y * scale)
                };
                localizerDetections.add(new VisionLocalizer.Detection(det.id, corners));
            }

            Map<Integer, VisionLocalizer.VisionPose3D> perTagPoses = localizer.estimateCameraPosePerTag(localizerDetections);
            VisionLocalizer.VisionPose3D bluePose = perTagPoses.get(20);
            VisionLocalizer.VisionPose3D redPose = perTagPoses.get(24);

            double imuYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double fieldHeading = imuYaw + HEADING_OFFSET;
            VisionLocalizer.VisionPose3D cameraPose = localizer.estimateCameraPoseConstrained(localizerDetections, fieldHeading);

            TelemetryPacket packet = new TelemetryPacket(false);
            com.acmerobotics.dashboard.canvas.Canvas field = packet.field();
            field.setAlpha(0.4);
            field.drawImage("/dash/decode.webp", 72, 72, 144, 144, Math.PI, 72, 72, true);
            field.setAlpha(1.0);
            field.drawGrid(0, 0, 144, 144, 7, 7);

            com.acmerobotics.dashboard.canvas.Canvas fieldOverlay = packet.fieldOverlay();
            fieldOverlay.setRotation(Math.PI);

            packet.put("Tags Detected", currentDetections.size());
            if (bluePose != null) { drawPose(fieldOverlay, bluePose, "#0000FF"); packet.put("Blue (Tag 20)", bluePose.toString()); }
            if (redPose != null)  { drawPose(fieldOverlay, redPose,  "#FF0000"); packet.put("Red (Tag 24)",  redPose.toString());  }
            packet.put("IMU Yaw", Math.toDegrees(imuYaw));
            packet.put("Field Heading", Math.toDegrees(fieldHeading));
            if (cameraPose != null) {
                drawPose(fieldOverlay, cameraPose, "#00FF00");
                packet.put("Camera Pose (MT2)", cameraPose.toString());
            } else {
                packet.put("Camera Pose", "Searching for tags...");
            }

            List<VisionArtifactDetector2.Artifact> artifacts = artifactProcessor.getLatestArtifacts();
            telemetry.addData("Artifacts Detected", artifacts.size());
            packet.put("Artifacts Detected", artifacts.size());

            if (artifacts.isEmpty()) {
                telemetry.addLine("No artifacts detected");
            } else {
                for (int i = 0; i < artifacts.size(); i++) {
                    VisionArtifactDetector2.Artifact a = artifacts.get(i);
                    String pos = String.format("fwd=%.1f\" lat=%.1f\"", a.relX, a.relY);
                    telemetry.addData("Artifact " + i, a.type + " " + pos);
                    packet.put("Artifact " + i, a.type + " " + pos);
                }
            }

            if (cameraPose != null) {
                double cosH = Math.cos(fieldHeading);
                double sinH = Math.sin(fieldHeading);
                for (VisionArtifactDetector2.Artifact a : artifacts) {
                    if (a.relX <= 0) continue;
                    double fieldX = cameraPose.x + a.relX * cosH - a.relY * sinH;
                    double fieldY = cameraPose.y + a.relX * sinH + a.relY * cosH;
                    String color = a.type.contains("Green") ? "#00CC00" : "#CC00CC";
                    fieldOverlay.setStroke(color);
                    fieldOverlay.setFill(color);
                    fieldOverlay.strokeCircle(fieldX, fieldY, 2.5);
                    fieldOverlay.fillCircle(fieldX, fieldY, 2.5);
                }
            }

            telemetry.update();
            dashboard.sendTelemetryPacket(packet);
            sleep(200);
        }

        visionPortal.close();
    }

    private void drawPose(com.acmerobotics.dashboard.canvas.Canvas canvas, VisionLocalizer.VisionPose3D pose, String color) {
        canvas.setStroke(color);
        canvas.setStrokeWidth(2);
        canvas.strokeCircle(pose.x, pose.y, 4.0);
    }
}
