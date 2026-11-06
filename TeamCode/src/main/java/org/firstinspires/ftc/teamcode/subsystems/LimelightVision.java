package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.Robot.RED_ALLIANCE;

import android.util.Log;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class LimelightVision extends SubsystemBase {
    private final Limelight3A limelight;
    private static double lastDetectedAngle = 0.0;
//    private Follower follower;

    // TODO: Determine if ball pipeline 1 works better than ball pipeline 2
    private static final int BALL_PIPELINE = 1;
    private static final double BALL_SEARCH_MIN_TX = -31.0;
    private static final double BALL_SEARCH_MAX_TX = 31.0;
    private static final double BALL_WINDOW_WIDTH_DEG = 12.0;
    private static final double BALL_STEP_SIZE_DEG = 2.0;
    private static final int BALL_FRAME_SAMPLE_COUNT = 5;
    private static final long BALL_FRAME_SAMPLE_DELAY_MS = 50;
    private static final double LIMELIGHT_TX_TO_PEDRO_HEADING_SIGN = -1.0;
    public static double BALL_WALL_ROBOT_CENTER_Y_ABS_IN = 58.75;
    public static double BALL_WALL_MIN_X_IN = 0.0;
    public static double BALL_WALL_MAX_X_IN = 70.75;
    public static double BALL_WALL_TARGET_TOLERANCE_IN = 4.0;
    public static double BALL_WALL_STALL_VELOCITY_IN_PER_SEC = 2.0;
    public static long BALL_WALL_MIN_DRIVE_TIME_MS = 250;
    public static long BALL_WALL_STALL_TIME_MS = 150;
    public static long BALL_WINDOW_CACHE_MAX_AGE_MS = 2000;

    private BallFrameHeading cachedBallWindowHeading;
    private long cachedBallWindowHeadingTimeMs;

    private double lastDistanceError, distanceIntegral, lastAngleError, angleIntegral = 0;

    public LimelightVision(HardwareMap hMap) {
        this.limelight = hMap.get(Limelight3A.class, "Ethernet Device");
//        follower = new Follower(hMap);

        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(20);
        // we want limelight off by default, and want to activate it only when the driver says so
        limelight.start();

        // only for testing purposes:
//        limelight.stop();
    }

    public void setLastDetectedAngle(double angle) {
        lastDetectedAngle = angle;
    }

    public double getLastDetectedAngle() {
        return lastDetectedAngle;
    }

    /** Returns the horizontal offset from the target (tx) */
    public double getTx() {
        return limelight.getLatestResult().getTx();
    }

    /** Returns the vertical offset from the target (ty) */
    public double getTy() {
        return limelight.getLatestResult().getTy();
    }

    /** Returns the detected target area (ta) */
    public double getTa() {
        return limelight.getLatestResult().getTa();
    }

    /** Checks if there is a valid target */
    public boolean isTargetValid() {
        return limelight.getLatestResult().isValid();
    }

    /** Returns the robot’s 3D pose (MT1) */
    public Pose3D getBotPoseMT1() {
        LLResult r = limelight.getLatestResult();
        if (r != null && r.isValid()) return r.getBotpose();
        return null;
    }

    /** Returns a list of detected fiducials (AprilTags) */
    public List<LLResultTypes.FiducialResult> getFiducials() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) return result.getFiducialResults();
        return null;
    }

    /** Returns the distance to a given fiducial */
    public double getDistanceToFiducial(LLResultTypes.FiducialResult fiducial) {
        if (fiducial != null) {
            Pose3D robotPose = fiducial.getRobotPoseTargetSpace();
            return Math.sqrt(
                    robotPose.getPosition().x * robotPose.getPosition().x +
                            robotPose.getPosition().y * robotPose.getPosition().y +
                            robotPose.getPosition().z * robotPose.getPosition().z
            );
        }
        return -1;
    }

    /** Returns the yaw (horizontal angle) to the fiducial */
    public double getYawToFiducial(LLResultTypes.FiducialResult fiducial) {
        if (fiducial != null) {
            Log.d("heading", "yawCalculation: " + fiducial.getTargetXDegrees());
            return fiducial.getTargetXDegrees();
        }
        return 0;
    }

    /** Returns the pitch (vertical angle) to the fiducial */
    public double getPitchToFiducial(LLResultTypes.FiducialResult fiducial) {
        if (fiducial != null) {
            Pose3D robotPose = fiducial.getRobotPoseTargetSpace();
            double horizontalDist = Math.sqrt(
                    robotPose.getPosition().x * robotPose.getPosition().x +
                            robotPose.getPosition().y * robotPose.getPosition().y
            );
            return Math.toDegrees(Math.atan2(robotPose.getPosition().z, horizontalDist));
        }
        return 0;
    }

    //DON'T USE THIS DOES NOT WORK, use getFieldPosition
    /** Returns the estimated robot position on the field */
    public Pose3D getEstimatedFieldPosition() {
        // this method is now empty since the Limelight subsystem can't have a follower.
//        updateRobotOrientation();

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D mt2 = result.getBotpose_MT2();
            if (mt2 != null) return mt2;
            Pose3D mt1 = result.getBotpose();
            if (mt1 != null) return mt1;
        }
        return null;
    }

//    public Pose3D getFieldPosition(double robotYawDegrees) {
////        updateRobotOrientation();
//        limelight.updateRobotOrientation(robotYawDegrees);
//
//        limelight.pipelineSwitch(0);
////        limelight.setPollRateHz(100);
//        LLResult result = limelight.getLatestResult();
//        Pose3D botpose = null;
//        if (result != null && result.isValid()) {
//            botpose = result.getBotpose();
//            Log.d("ManualDrive::Relocalize", "botpose is: " + botpose);
//            if (botpose != null) {
//                double x = botpose.getPosition().x;
//                double y = botpose.getPosition().y;
//                double heading = botpose.getOrientation().getYaw(AngleUnit.DEGREES);
//                Log.d("ManualDrive::Relocalize", "MT1 Location " + x + ", " + y + "," + heading);
//            }
//        }
//        limelight.setPollRateHz(100);
//        return botpose;
//    }

    // First, tell Limelight which way your robot is facing
    public Pose3D getFieldPosition(double robotYawDegrees) {
       limelight.updateRobotOrientation(robotYawDegrees);
       LLResult result = limelight.getLatestResult();
       if (result != null && result.isValid()) {
           Pose3D botpose_mt2 = result.getBotpose_MT2();
           if (botpose_mt2 != null) {
               double x = botpose_mt2.getPosition().x;
               double y = botpose_mt2.getPosition().y;
               double heading = botpose_mt2.getOrientation().getYaw(AngleUnit.DEGREES);
               Log.d("ManualDrive::Relocalize", "Mt2 Pose: (" + x + ", " + y + ", " + heading + ")");
               return botpose_mt2;
           } else {
               return null;
           }
       }
       return null;
//         LLResult result = limelight.getLatestResult();
//         if (result != null && result.isValid()) {
//             Pose3D botpose = result.getBotpose();
//             if (botpose != null) {
//                 double x = botpose.getPosition().x;
//                 double y = botpose.getPosition().y;
// //                telemetry.addData("MT1 Location", "(" + x + ", " + y + ")");
//             }
//             return botpose;
//         } else {
//             return null;
//         }
    }

    public void stopCameraStream() {
        limelight.stop();
    }

    public void pauseCameraStream() {
        limelight.pause();
    }

    public void startCameraStream() {
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public void setPollRate(int pollRate) {
        limelight.setPollRateHz(pollRate);
    }


    // ==========================
    // LIMELIGHT STATUS
    // ==========================

    /** Returns the time since the last update (ms) */
    public long getTimeSinceLastUpdate() {
        return limelight.getTimeSinceLastUpdate();
    }

    /** Checks if Limelight is connected */
    public boolean isConnected() {
        return limelight.isConnected();
    }

    // ==========================
    // PIPELINE METHODS
    // ==========================

    /** Switches to the desired pipeline by index */
    public boolean switchPipeline(int index) {
        return limelight.pipelineSwitch(index);
    }

    /** Reloads the current pipeline */
    public boolean reloadPipeline() {
        return limelight.reloadPipeline();
    }

    /** Captures a snapshot */
    public boolean captureSnapshot(String name) {
        return limelight.captureSnapshot(name);
    }

    /** Deletes a snapshot */
    public boolean deleteSnapshot(String name) {
        return limelight.deleteSnapshot(name);
    }



    // ==========================
    // FIELD / ORIENTATION METHODS
    // ==========================

    // Updates robot orientation for MegaTag2
//    private void updateRobotOrientation() {
////        double yaw = follower.getPose().getHeading();
////        limelight.updateRobotOrientation(yaw);
//    }
    
    /**
     * Updates the robot orientation for MegaTag2 localization.
     * @param robotYawDegrees The robot's current yaw in degrees
     */
    public void updateRobotOrientation(double robotYawDegrees) {
        limelight.updateRobotOrientation(robotYawDegrees);
    }
    
    /**
     * Gets the field position using MT2 with robot orientation.
     * first update the robot orientation for better accuracy
     * @param robotYawDegrees the robot's current yaw in degrees
     * @return the robot's Pose3D on the field, or null if N/A
     */
    public Pose3D getFieldPositionMT2(double robotYawDegrees) {

        limelight.pipelineSwitch(0);
        List<Double> xSamples = new ArrayList<>();
        List<Double> ySamples = new ArrayList<>();
        List<Double> zSamples = new ArrayList<>();
        List<Double> yawSamples = new ArrayList<>();
        List<Double> pitchSamples = new ArrayList<>();
        List<Double> rollSamples = new ArrayList<>();
        limelight.updateRobotOrientation(robotYawDegrees);
        try {
            Thread.sleep(150); // 150ms delay to let Limelight compute MT2 pose
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            return null;
        }
        for (int i = 0; i < 5; i++) {

            try {
                Thread.sleep(60); // 150ms delay to let Limelight compute MT2 pose
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return null;
            }

            LLResult result = limelight.getLatestResult();
            if (result == null) {
                Log.d("LimelightVision::getFieldPositionMT2", "Frame " + (i + 1) + ": result is null");
                continue;
            }

            if (!result.isValid()) {
                Log.d("LimelightVision::getFieldPositionMT2", "Frame " + (i + 1) + ": result is invalid");
                continue;
            }

            Pose3D botpose_mt2 = result.getBotpose_MT2();
            if (botpose_mt2 == null) {
                Log.d("LimelightVision::getFieldPositionMT2", "Frame " + (i + 1) + ": MT2 pose is null");
                continue;
            }

            double x = botpose_mt2.getPosition().x;
            double y = botpose_mt2.getPosition().y;
            double z = botpose_mt2.getPosition().z;
            double yaw = botpose_mt2.getOrientation().getYaw(AngleUnit.DEGREES);
            double pitch = botpose_mt2.getOrientation().getPitch(AngleUnit.DEGREES);
            double roll = botpose_mt2.getOrientation().getRoll(AngleUnit.DEGREES);

            xSamples.add(x);
            ySamples.add(y);
            zSamples.add(z);
            yawSamples.add(yaw);
            pitchSamples.add(pitch);
            rollSamples.add(roll);

            Log.d(
                    "LimelightVision::getFieldPositionMT2",
                    "Frame " + (i + 1) + ": x=" + x + ", y=" + y + ", z=" + z
                            + ", yaw=" + yaw + ", pitch=" + pitch + ", roll=" + roll
            );
        }

        if (xSamples.isEmpty()) {
            Log.d("LimelightVision::getFieldPositionMT2", "Final frame: no valid MT2 pose");
            return null;
        }

        double finalX = summarizeSamples(xSamples);
        double finalY = summarizeSamples(ySamples);
        double finalZ = summarizeSamples(zSamples);
        double finalYaw = summarizeSamples(yawSamples);
        double finalPitch = summarizeSamples(pitchSamples);
        double finalRoll = summarizeSamples(rollSamples);

        Pose3D finalPose = new Pose3D(
                new Position(DistanceUnit.METER, finalX, finalY, finalZ, 0L),
                new YawPitchRollAngles(AngleUnit.DEGREES, finalYaw, finalPitch, finalRoll, 0L)
        );

        Log.d(
                "LimelightVision::getFieldPositionMT2",
                "Final pose (" + (xSamples.size() == 3 ? "median" : "average") + "): x=" + finalX
                        + ", y=" + finalY + ", z=" + finalZ
                        + ", yaw=" + finalYaw + ", pitch=" + finalPitch + ", roll=" + finalRoll
        );

        return finalPose;
    }


    // ==========================
    // FIDUCIAL UTILITIES
    // ==========================

    /** Returns the ID of the closest fiducial */
    public Integer getClosestFiducialID() {
        List<LLResultTypes.FiducialResult> fiducials = getFiducials();
        if (fiducials == null || fiducials.isEmpty()) return null;

        LLResultTypes.FiducialResult closest = fiducials.get(0);
        double minDist = getDistanceToFiducial(closest);
        for (LLResultTypes.FiducialResult f : fiducials) {
            double dist = getDistanceToFiducial(f);
            if (dist < minDist) {
                minDist = dist;
                closest = f;
            }
        }
        return closest.getFiducialId();
    }

    // ==========================
    // MOVEMENT SETUP & CONTROL
    // ==========================

    /**
     * Moves the robot (mecanum drive) toward a fiducial until it reaches the desired distance.
     * @return true if target distance is reached
     */


    /*
    public boolean moveToFiducialByIdMecanum(int targetID double desiredDistance,) {

        List<LLResultTypes.FiducialResult> fiducials = getFiducials();
        if (fiducials == null || fiducials.isEmpty()) return false;

        LLResultTypes.FiducialResult target = null;
        for (LLResultTypes.FiducialResult f : fiducials) {
            if (f.getFiducialId() == targetID) { target = f; break; }
        }
        if (target == null) return false;

        double distance = getDistanceToFiducial(target);
        double driveDistance = distance; //- desiredDistance;
        double yawToTagDegrees = getYawToFiducial(target);
        double yawToTagRadians = Math.toRadians(getYawToFiducial(target));

        Pose currentPose = follower.getPose();
        double robotHeading = currentPose.getHeading();

        double forwardRel =  driveDistance * Math.cos(yawToTagRadians);
        double leftRel =  driveDistance * Math.sin(yawToTagRadians);

        double cosH = Math.cos(robotHeading);
        double sinH = Math.sin(robotHeading);

        double dx = forwardRel * cosH - leftRel * sinH;
        double dy = forwardRel * sinH + leftRel * cosH;

        // New world position
        double targetX = currentPose.getX() + dx;
        double targetY = currentPose.getY() + dy;

        Point currPosition = new Point(currentPose.getX(), currentPose.getY(), Point.CARTESIAN);
        double currHeading = robotHeading;

        Point targetPoint = new Point(targetX, targetY, Point.CARTESIAN);;
        double targetHeading = robotHeading + yawToTagRadians;

        Path toAlignment = new Path(new BezierLine(
                currPosition,
                targetPoint)
        );
        toAlignment.setLinearHeadingInterpolation(currHeading, targetHeading);

        follower.followPath(toAlignment);

        // Telemetry
        if (telemetry != null) {
            telemetry.addData("Fiducial ID", target.getFiducialId());
            telemetry.addData("distance", distance);
            telemetry.addData("Drive Distance", driveDistance);
            telemetry.addData("yawToTagDegrees", yawToTagDegrees);
            telemetry.addData("Beginning Robot Heading", robotHeading);
            telemetry.addData("Target x", targetX);
            telemetry.addData("Target y", targetY);
            telemetry.addData("Target heading", targetHeading);
            telemetry.update();
        }

        return false;
    }
    */

    // ==========================
    // UTILITY
    // ==========================

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private double summarizeSamples(List<Double> samples) {
        if (samples.size() == 5) {
            List<Double> sortedSamples = new ArrayList<>(samples);
            Collections.sort(sortedSamples);
            return sortedSamples.get(2);

        } else if (samples.size() == 4) {
            List<Double> sortedSamples = new ArrayList<>(samples);
            Collections.sort(sortedSamples);
            return ((sortedSamples.get(1) + sortedSamples.get(2))/2.0);

        } else if (samples.size() == 3){
            List<Double> sortedSamples = new ArrayList<>(samples);
            Collections.sort(sortedSamples);
            return sortedSamples.get(1);
        }

        double sum = 0.0;
        for (double sample : samples) {
            sum += sample;
        }
        return sum / samples.size();
    }

    public Limelight3A getLimelightCamera() {
        return limelight;
    }

    public Command turnToBiggestBallGroupCommand(Follower follower) {
        return followBestBallWindowToWallCommand(follower);
    }

    public Command sampleBestBallWindowCommand() {
        return new CommandBase() {
            private BallFrameHeading[] frameHeadings;
            private int frameCount;
            private long nextSampleTimeMs;

            {
                addRequirements(LimelightVision.this);
            }

            @Override
            public void initialize() {
                frameHeadings = new BallFrameHeading[BALL_FRAME_SAMPLE_COUNT];
                frameCount = 0;
                nextSampleTimeMs = 0L;

                limelight.pipelineSwitch(BALL_PIPELINE);
                limelight.setPollRateHz(100);
                limelight.start();
            }

            @Override
            public void execute() {
                long now = System.currentTimeMillis();
                if (frameCount >= BALL_FRAME_SAMPLE_COUNT || now < nextSampleTimeMs) {
                    return;
                }

                frameHeadings[frameCount] = getBallFrameHeading();
                frameCount++;
                nextSampleTimeMs = now + BALL_FRAME_SAMPLE_DELAY_MS;
            }

            @Override
            public boolean isFinished() {
                return frameCount >= BALL_FRAME_SAMPLE_COUNT;
            }

            @Override
            public void end(boolean interrupted) {
                BallFrameHeading bestHeading = getBestBallWindowHeading(frameHeadings);
                if (bestHeading != null) {
                    cacheBallWindowHeading(bestHeading);
                    Log.d(
                            "LimelightVision::BallWindowSample",
                            "cached tx=" + bestHeading.limelightTxDeg
                                    + " windowCount=" + bestHeading.targetWindow.count
                    );
                }
            }
        };
    }

    public Command followBestBallWindowToWallCommand(Follower follower) {
        return new CommandBase() {
            private boolean finished;
            private boolean hasTarget;
            private BallFrameHeading selectedFrame;
            private double targetX;
            private double targetY;
            private double targetHeadingRad;
            private long driveStartTimeMs;
            private long stallStartTimeMs;

            {
                addRequirements(LimelightVision.this);
            }

            @Override
            public void initialize() {
                finished = false;
                hasTarget = false;
                selectedFrame = null;
                driveStartTimeMs = 0L;
                stallStartTimeMs = 0L;

                limelight.pipelineSwitch(BALL_PIPELINE);
                limelight.setPollRateHz(100);
                limelight.start();

                selectedFrame = getCachedBallWindowHeading();
                if (selectedFrame == null) {
                    selectedFrame = getBestBallWindowHeading(sampleBallFrameHeadings());
                    cacheBallWindowHeading(selectedFrame);
                }

//                if (selectedFrame == null) {
//                    finished = true;
//                    Log.d("LimelightVision::BallWallPath", "No ball window found");
//                    return;
//                }
                //temporary, is probably a bit sketched
                if (selectedFrame == null) {
                    Pose currentPose = follower.getPose();

                    double headingRad = currentPose.getHeading();
                    targetX = currentPose.getX() + 50.0 * Math.cos(headingRad);
                    targetY = currentPose.getY() + 50.0 * Math.sin(headingRad);
                    targetHeadingRad = headingRad;

                    hasTarget = true;

                    Path path = new Path(new BezierLine(
                            new Point(currentPose.getX(), currentPose.getY(), Point.CARTESIAN),
                            new Point(targetX, targetY, Point.CARTESIAN)
                    ));
                    path.setConstantHeadingInterpolation(targetHeadingRad);
                    path.setZeroPowerAccelerationMultiplier(4.5);
                    path.setPathEndTimeoutConstraint(50);

                    follower.setMaxPower(1.0);
                    follower.followPath(path);

                    Log.d("LimelightVision::followBestBallWindowToWallCommand", "No ball window found. Target point: "
                                    + " targetX=" + targetX  + " targetY=" + targetY + " headingDeg=" + Math.toDegrees(targetHeadingRad)
                    );
                    return;
                }

                Pose currentPose = follower.getPose();
                double currentHeadingDeg = Math.toDegrees(currentPose.getHeading());
                double targetHeadingDeg = normalizeSignedDegrees(
                        currentHeadingDeg + LIMELIGHT_TX_TO_PEDRO_HEADING_SIGN * selectedFrame.limelightTxDeg
                );
                lastDetectedAngle = targetHeadingDeg;

                WallIntersection intersection = findWallIntersection(currentPose, Math.toRadians(targetHeadingDeg));
                if (intersection == null) {
                    // TODO: check if this failsafe works!
//                    finished = true;
                    Log.d("LimelightVision::BallWallPath", "Ball heading does not hit wall in front of robot");
//                    return;

                    double headingRad = currentPose.getHeading();
                    targetX = currentPose.getX() + 50.0 * Math.cos(headingRad);
                    targetY = currentPose.getY() + 50.0 * Math.sin(headingRad);
                    targetHeadingRad = headingRad;

                    hasTarget = true;

                    Path path = new Path(new BezierLine(
                        new Point(currentPose.getX(), currentPose.getY(), Point.CARTESIAN),
                        new Point(targetX, targetY, Point.CARTESIAN)
                    ));
                    path.setConstantHeadingInterpolation(targetHeadingRad);
                    path.setZeroPowerAccelerationMultiplier(4.5);
                    path.setPathEndTimeoutConstraint(50);

                    follower.setMaxPower(1.0);
                    follower.followPath(path);

                    Log.d("LimelightVision::followBestBallWindowToWallCommand", "No ball window found. Target point: "
                        + " targetX=" + targetX  + " targetY=" + targetY + " headingDeg=" + Math.toDegrees(targetHeadingRad)
                    );
                    return;
                }

                targetX = intersection.x;
                targetY = intersection.y;
                targetHeadingRad = getWallFacingHeadingRad();
                hasTarget = true;
                driveStartTimeMs = System.currentTimeMillis();

                Path path = new Path(new BezierLine(
                        new Point(currentPose.getX(), currentPose.getY(), Point.CARTESIAN),
                        // TODO: Be aware of this hardcoded addition that makes the robot driver far enough to the wall
                        new Point(targetX, targetY /*+ 6.75*/, Point.CARTESIAN)
                ));
                path.setConstantHeadingInterpolation(targetHeadingRad);
                path.setZeroPowerAccelerationMultiplier(4.5);
                path.setPathEndTimeoutConstraint(50);

                follower.setMaxPower(1.0);
                follower.followPath(path);

                Log.d(
                        "LimelightVision::BallWallPath",
                        "tx=" + selectedFrame.limelightTxDeg
                                + " targetHeadingDeg=" + targetHeadingDeg
                                + " wallX=" + targetX
                                + " wallY=" + targetY
                                + " faceWallDeg=" + Math.toDegrees(targetHeadingRad)
                                + " windowCount=" + selectedFrame.targetWindow.count
                );
            }

            @Override
            public void execute() {
                if (!hasTarget) {
                    return;
                }

                if (telemetry != null) {
                    telemetry.addData("Ball tx", "%.1f deg", selectedFrame.limelightTxDeg);
                    telemetry.addData("Ball ray heading", "%.1f deg", lastDetectedAngle);
                    telemetry.addData("Ball wall x", "%.1f in", targetX);
                    telemetry.addData("Ball wall y", "%.1f in", targetY);
                    telemetry.addData("Ball face wall", "%.1f deg", Math.toDegrees(targetHeadingRad));
                    telemetry.addData("Ball window count", selectedFrame.targetWindow.count);
                }

                Pose currentPose = follower.getPose();
                double distanceToTarget = Math.hypot(targetX - currentPose.getX(), targetY - currentPose.getY());
                long now = System.currentTimeMillis();
                boolean pastMinDriveTime = now - driveStartTimeMs >= BALL_WALL_MIN_DRIVE_TIME_MS;
                boolean stalled = pastMinDriveTime
                        && follower.getVelocity().getMagnitude() <= BALL_WALL_STALL_VELOCITY_IN_PER_SEC;

                if (stalled && stallStartTimeMs == 0L) {
                    stallStartTimeMs = now;
                } else if (!stalled) {
                    stallStartTimeMs = 0L;
                }

                boolean stalledLongEnough = stallStartTimeMs != 0L
                        && now - stallStartTimeMs >= BALL_WALL_STALL_TIME_MS;

                if (!follower.isBusy() || distanceToTarget <= BALL_WALL_TARGET_TOLERANCE_IN || stalledLongEnough) {
                    finished = true;
                }
            }

            @Override
            public boolean isFinished() {
                return finished;
            }

            @Override
            public void end(boolean interrupted) {
                follower.breakFollowing();
            }
        };
    }

    public BallFrameHeading getBestBallHeadingFromFiveFrames() {
        limelight.pipelineSwitch(BALL_PIPELINE);
        limelight.setPollRateHz(100);
        limelight.start();
        return getBestBallWindowHeading(sampleBallFrameHeadings());
    }

    private void cacheBallWindowHeading(BallFrameHeading heading) {
        if (heading == null) {
            return;
        }

        cachedBallWindowHeading = heading;
        cachedBallWindowHeadingTimeMs = System.currentTimeMillis();
    }

    private BallFrameHeading getCachedBallWindowHeading() {
        if (cachedBallWindowHeading == null) {
            return null;
        }

        if (System.currentTimeMillis() - cachedBallWindowHeadingTimeMs > BALL_WINDOW_CACHE_MAX_AGE_MS) {
            return null;
        }

        return cachedBallWindowHeading;
    }

    private BallFrameHeading[] sampleBallFrameHeadings() {
        BallFrameHeading[] frameHeadings = new BallFrameHeading[BALL_FRAME_SAMPLE_COUNT];

        for (int i = 0; i < BALL_FRAME_SAMPLE_COUNT; i++) {
            frameHeadings[i] = getBallFrameHeading();
            sleepQuietly(BALL_FRAME_SAMPLE_DELAY_MS);
        }

        return frameHeadings;
    }

    private BallFrameHeading getBallFrameHeading() {
        LLResult result = limelight.getLatestResult();
        List<BallDetectionTarget> targets = getBallTargets(result);
        BallTargetWindow targetWindow = findBestBallWindow(targets);

        BallFrameHeading frameHeading = new BallFrameHeading();
        frameHeading.resultValid = result != null && result.isValid();
        frameHeading.targetCount = targets.size();
        frameHeading.targetWindow = targetWindow;

        if (targetWindow != null) {
            frameHeading.hasHeading = true;
            frameHeading.limelightTxDeg = targetWindow.meanTxDeg;
            frameHeading.windowCount = targetWindow.count;
            frameHeading.windowStartDeg = targetWindow.windowStartDeg;
            frameHeading.windowCenterDeg = targetWindow.windowCenterDeg;
        }

        return frameHeading;
    }

    private BallFrameHeading getBestBallWindowHeading(BallFrameHeading[] frameHeadings) {
        BallFrameHeading bestHeading = null;

        for (BallFrameHeading frameHeading : frameHeadings) {
            if (frameHeading == null || !frameHeading.hasHeading) {
                continue;
            }

            if (bestHeading == null || frameHeading.targetWindow.count > bestHeading.targetWindow.count) {
                bestHeading = frameHeading;
            }
        }

        return bestHeading;
    }

    private List<BallDetectionTarget> getBallTargets(LLResult result) {
        List<BallDetectionTarget> targets = new ArrayList<>();

        if (result == null || !result.isValid()) {
            return targets;
        }

        List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();
        if (detections == null) {
            return targets;
        }

        for (LLResultTypes.DetectorResult detection : detections) {
            BallDetectionTarget target = new BallDetectionTarget();
            target.className = detection.getClassName() == null ? "" : detection.getClassName();
            target.txDeg = detection.getTargetXDegrees();
            targets.add(target);
        }

        return targets;
    }

    private BallTargetWindow findBestBallWindow(List<BallDetectionTarget> targets) {
        if (targets.isEmpty()) {
            return null;
        }

        BallTargetWindow bestWindow = null;
        double minTx = Double.POSITIVE_INFINITY;
        double maxTx = Double.NEGATIVE_INFINITY;

        for (BallDetectionTarget target : targets) {
            minTx = Math.min(minTx, target.txDeg);
            maxTx = Math.max(maxTx, target.txDeg);
        }

        double searchStart = Math.min(BALL_SEARCH_MIN_TX, minTx - BALL_WINDOW_WIDTH_DEG / 2.0);
        double searchEnd = Math.max(BALL_SEARCH_MAX_TX, maxTx + BALL_WINDOW_WIDTH_DEG / 2.0);

        for (double start = searchStart; start <= searchEnd - BALL_WINDOW_WIDTH_DEG; start += BALL_STEP_SIZE_DEG) {
            double end = start + BALL_WINDOW_WIDTH_DEG;
            int count = 0;
            double txSum = 0.0;

            for (BallDetectionTarget target : targets) {
                if (target.txDeg >= start && target.txDeg <= end) {
                    count++;
                    txSum += target.txDeg;
                }
            }

            double windowCenter = start + BALL_WINDOW_WIDTH_DEG / 2.0;
            if (bestWindow == null || count > bestWindow.count) {
                bestWindow = new BallTargetWindow();
                bestWindow.count = count;
                bestWindow.windowStartDeg = start;
                bestWindow.windowCenterDeg = windowCenter;
                bestWindow.meanTxDeg = count == 0 ? windowCenter : txSum / count;
            }
        }

        return bestWindow;
    }

    private WallIntersection findWallIntersection(Pose currentPose, double rayHeadingRad) {
        double sin = Math.sin(rayHeadingRad);
        if (Math.abs(sin) < 1e-6) {
            return null;
        }

        double wallY = (RED_ALLIANCE ? 1.0 : -1.0) * BALL_WALL_ROBOT_CENTER_Y_ABS_IN;
        double distanceAlongRay = (wallY - currentPose.getY()) / sin;
        if (distanceAlongRay <= 0.0) {
            return null;
        }

        WallIntersection intersection = new WallIntersection();
        intersection.x = clamp(currentPose.getX() + distanceAlongRay * Math.cos(rayHeadingRad), BALL_WALL_MIN_X_IN, BALL_WALL_MAX_X_IN);
        intersection.y = wallY;
        return intersection;
    }

    private double getWallFacingHeadingRad() {
        return Math.toRadians(RED_ALLIANCE ? 90.0 : -90.0);
    }

    private void sleepQuietly(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    private double normalizeSignedDegrees(double degrees) {
        while (degrees > 180.0) {
            degrees -= 360.0;
        }

        while (degrees <= -180.0) {
            degrees += 360.0;
        }

        return degrees;
    }

    private static class BallDetectionTarget {
        String className;
        double txDeg;
    }

    private static class BallTargetWindow {
        int count;
        double windowStartDeg;
        double windowCenterDeg;
        double meanTxDeg;
    }

    private static class WallIntersection {
        double x;
        double y;
    }

    public static class BallFrameHeading {
        public boolean resultValid;
        public boolean hasHeading;
        public int targetCount;
        public int windowCount;
        public double limelightTxDeg;
        public double windowStartDeg;
        public double windowCenterDeg;
        private BallTargetWindow targetWindow;
    }
}
