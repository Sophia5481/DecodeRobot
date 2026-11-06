package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Canvas;
import android.util.Log;
import android.util.Size;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;

import java.util.List;
import java.util.ArrayList;

public class Vision2 extends SubsystemBase {
    public class TagPosition {
        public final double x;
        public final double y;
        public final double relativeAngle;
        public final double distance;
        public final double tagHeight;
        public final double yaw;  // Added yaw angle from homography

        public TagPosition(double x, double y, double relativeAngle, double distance, double tagHeight, double yaw) {
            this.x = x;
            this.y = y;
            this.relativeAngle = relativeAngle;
            this.distance = distance;
            this.tagHeight = tagHeight;
            this.yaw = yaw;
        }
    }

    private double tagSizeCm = 16.51;
    // Make this consistent with teamwebcamcalibrations.xml
    // Camera angle is from the horizontal, aimed upwards. Hard-coded focal length
    // Hard code focal lengths from calibration: 786.357, 785.863; scale down to 1280x720 from 1920x1080
    private double f_x_afterresize = 786.357 * 2 / 3;
    private double f_y_afterresize = 785.863 * 2 / 3;
    private double c_x_afterresize = 989.731f * 2 / 3;
    private double c_y_afterresize = 501.663f * 2 / 3;

    private double lastCalculatedAngle;

    public UndistortPipeline pipeline;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    // servo that turns the camera so it can look at the april tags
    private ServoEx miniTurret;
    private static double TURRET_MIN_ANGLE = 0.0;
    private static double TURRET_MAX_ANGLE = 180.0;
    private static double TELE_ANGLE = 173.0;
    private static double RED_AUTO_ANGLE = 75.0; //100.0
    private static double BLUE_AUTO_ANGLE = 10.0;
    private static double FAR_AUTO_ANGLE = 180.0;

    private static double ROBOT_LENGTH = 16.625;
    private static double ROBOT_WIDTH = 17.125;

    public Vision2(final HardwareMap hMap) {
        miniTurret = new SimpleServo(hMap, "miniTurret", TURRET_MIN_ANGLE, TURRET_MAX_ANGLE);
        setupVisionPortal(hMap);
    }

    public void init(boolean teleop, boolean redAlliance) {
        if (teleop) {
            miniTurret.turnToAngle(TELE_ANGLE);
            visionPortal.setProcessorEnabled(aprilTag, true);
        } else if (redAlliance && !Robot.FAR_AUTO) {
            miniTurret.turnToAngle(RED_AUTO_ANGLE);
        } else if (!redAlliance && !Robot.FAR_AUTO){
            miniTurret.turnToAngle(BLUE_AUTO_ANGLE);
        }
        else {
            miniTurret.turnToAngle(FAR_AUTO_ANGLE);
        }
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void setupVisionPortal(final HardwareMap hMap) {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera.
        builder.setCamera(hMap.get(WebcamName.class, "Webcam 1"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(1920, 1080));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        // Framerate increase is likely a bluff
//        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        pipeline = new UndistortPipeline();

        builder.addProcessor(pipeline);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();
//        FtcDashboard.getInstance().startCameraStream(visionPortal, 0);

        // Disable or re-enable the aprilTag processor at any time.
//        visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()

    // Used *only* in auto for the obelisk
    public int getAprilTagId(boolean redAlliance) {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        Log.d("# AprilTags Detected", "" + currentDetections.size());

        // based on testing, the april tag with the lowest relative angle is the one we want on the blue side
        // Atag with the highest relative angle is the one we want on the red side
        double bestRelativeAngle = redAlliance ? Double.MIN_VALUE : Double.MAX_VALUE;
        int id = 21;
        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == 24 || detection.id == 20) {
                continue;
            }
            else if (Robot.FAR_AUTO){
                return detection.id;
            }
            else {
                // Make this consistent with teamwebcamcalibrations.xml
                // Camera angle is from the horizontal, aimed upwards. Hard-coded focal length
                // Focal lengths from calibration: 786.357, 785.863; changed to work a bit better
                TagPosition tag = calculateTagPosition3D(detection);

                double relativeAngle = tag.relativeAngle;

                if (redAlliance) {
                    if (relativeAngle > bestRelativeAngle) {
                        bestRelativeAngle = relativeAngle;
                        id = detection.id;
                    }
                } else {
                    if (relativeAngle < bestRelativeAngle) {
                        bestRelativeAngle = relativeAngle;
                        id = detection.id;
                    }
                }
            }
        }
        return id;
    }

    public TagPosition getRealWorldPosition(boolean redAlliance) {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        Log.d("getRealWorldPosition::# AprilTags Detected", "" + currentDetections.size());
        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (redAlliance && detection.id == 24 || !redAlliance && detection.id == 20) {
                Log.d("getRealWorldPosition::ID: ", String.format("\n==== (ID %d)", detection.id));
                Log.d("getRealWorldPosition::Detection Center: ", String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));

                // Make this consistent with teamwebcamcalibrations.xml
                // Camera angle is from the horizontal, aimed upwards. Hard-coded focal length
                // Focal lengths from calibration: 786.357, 785.863; changed to work a bit better

                TagPosition tag = calculateTagPosition3D(detection);

                // Prevents turning twice based on the same image (which results in duplicate relative angles)
                if (Math.abs(tag.relativeAngle - lastCalculatedAngle) < 0.5) {
                    return null;
                }

                lastCalculatedAngle = tag.relativeAngle;

                Log.d("getRealWorldPosition::Relative Angle", String.format("%6.2f", tag.relativeAngle));
                Log.d("getRealWorldPosition::Real world position: ", String.format("Real World Position %6.2f %6.2f", tag.x, tag.y));
                return tag;
            }
        }
        Log.d("getRealWorldPosition:: ", "No correct april tag found!");
        // just to make java stop complaining at me
        // if there are no detections, return 0.0
        return null;
        // end for() loop
    }

    // Calculate the relative angle to the back of the goal (how much we need to turn!)
    public double backRelativeAngle(TagPosition tag) {
        // Distance from tag to back of goal or where you're firing to (cm)
        double distanceToBack = 30;

        // Displacement from camera to back of goal (cm)
        double backPositionX = tag.x + distanceToBack * Math.cos(Math.toRadians(tag.relativeAngle + tag.yaw - 90));
        double backPositionY = tag.y + distanceToBack * Math.sin(Math.toRadians(tag.relativeAngle + tag.yaw - 90));

        // Displacement from CENTER of robot to BACK of goal (cm)
        // Placeholder values; adjust based on where camera is on robot
        double backDispX = backPositionX + 9.7;
        double backDispY = backPositionY + 8.3;

        return Math.toDegrees(Math.atan2(backDispY, backDispX));
    }

    private TagPosition calculateTagPosition3D(AprilTagDetection detection) {
        // Calculate height using corners
        org.opencv.core.Point[] corners = detection.corners;
        double tagHeight = Math.abs((corners[0].y + corners[1].y) - (corners[2].y + corners[3].y)) / 2;

        // Calculate angle using focal length
        double relativeAngle = 90 - Math.toDegrees(Math.atan2((detection.center.x - c_x_afterresize), f_x_afterresize));
        
        // Calculate yaw using homography decomposition
        double yaw = calculateTagYaw(detection);

        // Calculate position based on size
        double groundDistance = tagSizeCm / tagHeight * f_y_afterresize;
        double x = groundDistance * Math.cos(Math.toRadians(relativeAngle));
        double y = groundDistance * Math.sin(Math.toRadians(relativeAngle));

        return new Vision2.TagPosition(x, y, relativeAngle, groundDistance, tagHeight, yaw);
    }

    private double calculateTagYaw(AprilTagDetection detection) {
        // Get the corners of the AprilTag
        org.opencv.core.Point[] corners = detection.corners;
        
        // Create source points (ideal tag corners in normalized coordinates)
        MatOfPoint2f srcPoints = new MatOfPoint2f();
        srcPoints.fromArray(
            new Point(-tagSizeCm/2, tagSizeCm/2),   // top-left
            new Point(tagSizeCm/2, tagSizeCm/2),    // top-right
            new Point(tagSizeCm/2, -tagSizeCm/2),   // bottom-right
            new Point(-tagSizeCm/2, -tagSizeCm/2)   // bottom-left
        );
        
        // Create destination points (detected corners in image coordinates)
        MatOfPoint2f dstPoints = new MatOfPoint2f();
        dstPoints.fromArray(
            new Point(corners[0].x, corners[0].y),
            new Point(corners[1].x, corners[1].y),
            new Point(corners[2].x, corners[2].y),
            new Point(corners[3].x, corners[3].y)
        );
        
        // Calculate homography matrix
        Mat H = Calib3d.findHomography(srcPoints, dstPoints);
        
        // Create camera matrix
        Mat K = new Mat(3, 3, CvType.CV_64F);
        K.put(0, 0, 
            f_x_afterresize, 0, c_x_afterresize,
            0, f_y_afterresize, c_y_afterresize,
            0, 0, 1
        );
        
        // Decompose homography
        List<Mat> rotationMatrices = new ArrayList<>();
        List<Mat> translationVectors = new ArrayList<>();
        List<Mat> normalVectors = new ArrayList<>();
        
        Calib3d.decomposeHomographyMat(H, K, rotationMatrices, translationVectors, normalVectors);
        
        // Get rotation matrix (we'll use the first solution since tag is vertical)
        Mat R = new Mat(3, 3, CvType.CV_64F);
        if (!rotationMatrices.isEmpty()) {
            rotationMatrices.get(0).copyTo(R);
        }
        
        // Convert rotation matrix to Euler angles
        // For a vertical tag, yaw is the rotation around the Y-axis
        double[] r = new double[9];
        R.get(0, 0, r);
        
        // Calculate yaw (rotation around Y-axis)
        // Converting from standard yaw (-180 to +180) to our angle convention (0 to 360)
        // where 90 is facing the camera, 0 is facing left, 180 is facing right
        double standardYaw = Math.atan2(r[2], r[0]);  // standard yaw angle
        double yaw = 180 - Math.toDegrees(standardYaw); // convert to our convention
        
        // Normalize to 0-360 range
        if (yaw < 0) yaw += 360;
        if (yaw >= 360) yaw -= 360;
        
        // Clean up
        srcPoints.release();
        dstPoints.release();
        H.release();
        K.release();
        R.release();
        
        // Release all matrices in the lists
        for (Mat m : rotationMatrices) m.release();
        for (Mat m : translationVectors) m.release();
        for (Mat m : normalVectors) m.release();
        
        return yaw;
    }

    public class UndistortPipeline implements VisionProcessor {
        private Mat cameraMatrix;
        private Mat distCoeffs;
        public long lastCaptureTimeNanos;
        public long lastSystemTime;

        public double[] cameraMatrixArray;

        public long[] timeDeltas = new long[2];

        private org.opencv.core.Size imageSize = new org.opencv.core.Size(1920, 1080); // adjust to your camera stream resolution

        @Override
        public void init(int width, int height, CameraCalibration calibration) {

        }

        // Returns delta of [System time, Camera time] in ms; sets last
        public long[] findCaptureTimeDelta(long cameraTime) {
            long systemTimeDelta = System.currentTimeMillis() - lastSystemTime;
            long cameraTimeDelta = (cameraTime - lastCaptureTimeNanos) / 1000000;

            lastSystemTime = System.currentTimeMillis();
            lastCaptureTimeNanos = cameraTime;
            return new long[]{systemTimeDelta, cameraTimeDelta};
        }

        @Override
        public Object processFrame(Mat input, long captureTimeNanos) {
            timeDeltas = findCaptureTimeDelta(captureTimeNanos);

            // Don't run if still using previous image
            // if (Math.abs(timeDeltas[1] - timeDeltas[0]) > 100) {
            //     return null;
            // }

            // Do lazy initialization
            if (cameraMatrix == null) {
                cameraMatrix = new Mat(3, 3, CvType.CV_64F);
                distCoeffs = new Mat(1, 5, CvType.CV_64F);

                // Example values — replace with your own calibrated ones; these are for 1920x1080 (scaled to 1280x720)
                cameraMatrix.put(0, 0, 1023.878f); // fx
                cameraMatrix.put(0, 1, 0);
                cameraMatrix.put(0, 2, 989.731f);
                cameraMatrix.put(1, 0, 0);
                cameraMatrix.put(1, 1, 1019.899f); // fy
                cameraMatrix.put(1, 2, 501.663f);
                cameraMatrix.put(2, 0, 0);
                cameraMatrix.put(2, 1, 0);
                cameraMatrix.put(2, 2, 1);

                // Example distortion coefficients: k1, k2, p1, p2, k3
                distCoeffs.put(0, 0, -0.370767f, 0.106516f, 0.000118f, -0.000542f, -0.012277f);
            }

            Mat undistorted = new Mat();
            Mat resized = new Mat();

            Calib3d.undistort(input, undistorted, cameraMatrix, distCoeffs, cameraMatrix);

            org.opencv.core.Size newSize = new org.opencv.core.Size(1280, 720);
            Imgproc.resize(undistorted, resized, newSize);

            // Draw a blue dot at the principal point using pre-calculated after-resize coordinates
            // Imgproc.circle(resized, new Point(c_x_afterresize, c_y_afterresize), 5, new Scalar(0, 0, 255), -1);

            // (Optional) Do further image processing here
            // e.g. detect apriltags, contours, colors, etc.

            // Replace the input with resized image for display in the RC preview
            resized.copyTo(input);
            undistorted.release();
            resized.release();
            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                                float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
            // You can draw overlays here if you want (optional)
        }
    }

    // stops vision to save resources (check here: https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/visionportal_cpu_and_bandwidth/visionportal-cpu-and-bandwidth.html#close-visionportal)
    public void stopVision() {
        visionPortal.setProcessorEnabled(aprilTag, false);
    }

    public void startVision() {
        visionPortal.setProcessorEnabled(aprilTag, true);
    }

    public VisionPortal getVisionPortal() {
        return visionPortal;
    }
}
