package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Canvas;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.vision.VisionArtifactDetector2;
import org.firstinspires.ftc.teamcode.vision.VisionLocalizer;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

/**
 * Subsystem that runs VisionArtifactDetector2 continuously on camera frames.
 * ArtifactProcessor is public so VisionOpMode can add it to its own shared VisionPortal.
 */
public class ArtifactVision extends SubsystemBase {
    private final VisionPortal visionPortal;
    private final ArtifactProcessor processor;

    public ArtifactVision(HardwareMap hMap) {
        processor = new ArtifactProcessor();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new android.util.Size(1920, 1080))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(processor)
                .build();
    }

    public List<VisionArtifactDetector2.Artifact> getLatestArtifacts() {
        return processor.getLatestArtifacts();
    }

    public VisionPortal getVisionPortal() {
        return visionPortal;
    }

    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    public static class ArtifactProcessor implements VisionProcessor {
        private VisionArtifactDetector2 detector;
        private volatile List<VisionArtifactDetector2.Artifact> latestArtifacts = new ArrayList<>();
        private Mat resized = new Mat();
        private Mat cameraMatrix;
        private MatOfDouble distCoeffs;

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            cameraMatrix = VisionLocalizer.getArducamMatrix();
            distCoeffs = VisionLocalizer.getArducamDistCoeffs();
            detector = new VisionArtifactDetector2(cameraMatrix, VisionLocalizer.ARDUCAM_TRANSFORM);
        }

        @Override
        public Object processFrame(Mat input, long captureTimeNanos) {
            Imgproc.resize(input, resized, new org.opencv.core.Size(1280, 720));

            List<VisionArtifactDetector2.Artifact> artifacts = detector.detect(resized);

            if (!artifacts.isEmpty()) {
                org.opencv.core.Point[] distorted = new org.opencv.core.Point[artifacts.size()];
                for (int i = 0; i < artifacts.size(); i++) {
                    distorted[i] = artifacts.get(i).pixelPoint;
                }
                MatOfPoint2f distortedMat = new MatOfPoint2f(distorted);
                MatOfPoint2f undistortedMat = new MatOfPoint2f();
                Calib3d.undistortPoints(distortedMat, undistortedMat, cameraMatrix, distCoeffs, new Mat(), cameraMatrix);
                org.opencv.core.Point[] undistortedPts = undistortedMat.toArray();
                for (int i = 0; i < artifacts.size(); i++) {
                    artifacts.get(i).pixelPoint = undistortedPts[i];
                }
                distortedMat.release();
                undistortedMat.release();
            }

            latestArtifacts = artifacts;

            for (VisionArtifactDetector2.Artifact a : artifacts) {
                Scalar color = a.type.contains("Green") ? new Scalar(0, 255, 0) : new Scalar(255, 0, 255);
                org.opencv.core.Point p1 = new org.opencv.core.Point(a.boundingBox.x, a.boundingBox.y);
                org.opencv.core.Point p2 = new org.opencv.core.Point(a.boundingBox.x + a.boundingBox.width, a.boundingBox.y + a.boundingBox.height);
                Imgproc.rectangle(resized, p1, p2, color, 3);
                Imgproc.circle(resized, new org.opencv.core.Point(a.pixelPoint.x, a.pixelPoint.y), 6, color, -1);
                Imgproc.putText(resized, a.type, new org.opencv.core.Point(p1.x, p1.y - 8), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
            }

            Imgproc.resize(resized, input, input.size());
            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                                float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {}

        public List<VisionArtifactDetector2.Artifact> getLatestArtifacts() {
            return latestArtifacts;
        }
    }
}
