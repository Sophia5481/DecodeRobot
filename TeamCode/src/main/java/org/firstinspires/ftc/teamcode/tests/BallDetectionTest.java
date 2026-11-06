package org.firstinspires.ftc.teamcode.tests;

import android.util.Log;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

import java.util.ArrayList;
import java.util.List;

//@TeleOp(name = "Ball Odo Turn Sliding Tx Window", group = "Test")
public class BallDetectionTest extends LinearOpMode {

    private Limelight3A limelight;
    private Follower follower;

    private static final int PIPELINE = 2;

    private static final double SEARCH_MIN_TX = -31.0;
    private static final double SEARCH_MAX_TX = 31.0;
    private static final double WINDOW_WIDTH_DEG = 12.0;
    private static final double STEP_SIZE_DEG = 2.0;
    private static final int FRAME_SAMPLE_COUNT = 5;
    private static final long FRAME_SAMPLE_DELAY_MS = 50;
    private static final double TURN_MAX_POWER = 1.0;
    private static final double LIMELIGHT_TX_TO_PEDRO_HEADING_SIGN = -1.0;

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "Ethernet Device");
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        follower.setMaxPower(TURN_MAX_POWER);

        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(PIPELINE);
        limelight.start();

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        FrameHeading[] frameHeadings = sampleFrameHeadings();
        FrameHeading targetHeading = getBestFrameHeading(frameHeadings);
        double targetHeadingDeg = 0.0;

        if (targetHeading != null) {
            targetHeadingDeg = LIMELIGHT_TX_TO_PEDRO_HEADING_SIGN * targetHeading.limelightTxDeg;
            double targetHeadingRad = Math.toRadians(targetHeadingDeg);
            follower.holdPoint(new BezierPoint(new Point(0, 0, Point.CARTESIAN)), targetHeadingRad);
        }

        while (opModeIsActive()) {
            follower.update();

            telemetry.addLine("----- Odo Turn Target -----");
            telemetry.addLine("----- 5 Frame Heading Batch -----");
            outputFrameHeadings(frameHeadings);
            telemetry.addData("Odo x", "%.2f", follower.getPose().getX());
            telemetry.addData("Odo y", "%.2f", follower.getPose().getY());
            double currentHeadingDeg = normalizeSignedDegrees(Math.toDegrees(follower.getPose().getHeading()));
            telemetry.addData("Odo heading", "%.1f deg", currentHeadingDeg);

            if (targetHeading == null) {
                telemetry.addLine("No target heading found");
            } else {
                telemetry.addData("Selected Limelight tx", "%.1f deg", targetHeading.limelightTxDeg);
                telemetry.addData("Pedro target heading", "%.1f deg", targetHeadingDeg);
                telemetry.addData("Heading error", "%.1f deg", normalizeSignedDegrees(targetHeadingDeg - currentHeadingDeg));
            }

            telemetry.update();
        }

        follower.breakFollowing();
        limelight.stop();
    }

    private FrameHeading[] sampleFrameHeadings() {
        FrameHeading[] frameHeadings = new FrameHeading[FRAME_SAMPLE_COUNT];

        for (int i = 0; i < FRAME_SAMPLE_COUNT && opModeIsActive(); i++) {
            frameHeadings[i] = getFrameHeading();
            sleep(FRAME_SAMPLE_DELAY_MS);
        }

        return frameHeadings;
    }

    private FrameHeading getFrameHeading() {
        LLResult result = limelight.getLatestResult();
        List<DetectionTarget> targets = getTargets(result);
        TargetGroup targetGroup = findBiggestGroup(targets);

        FrameHeading frameHeading = new FrameHeading();
        frameHeading.resultValid = result != null && result.isValid();
        frameHeading.targetCount = targets.size();
        frameHeading.targetGroup = targetGroup;

        if (targetGroup != null) {
            frameHeading.hasHeading = true;
            frameHeading.limelightTxDeg = targetGroup.meanTxDeg;
        }

        return frameHeading;
    }

    private void outputFrameHeadings(FrameHeading[] frameHeadings) {
        FrameHeading bestHeading = getBestFrameHeading(frameHeadings);

        for (int i = 0; i < frameHeadings.length; i++) {
            FrameHeading frameHeading = frameHeadings[i];

            if (frameHeading == null) {
                telemetry.addData("Frame " + (i + 1), "not sampled");
                continue;
            }

            if (!frameHeading.hasHeading) {
                telemetry.addData(
                        "Frame " + (i + 1),
                        "no heading valid=%s targets=%d",
                        frameHeading.resultValid,
                        frameHeading.targetCount
                );
                continue;
            }

            telemetry.addData(
                    "Frame " + (i + 1),
                    "tx=%.1f total=%d group=%d",
                    frameHeading.limelightTxDeg,
                    frameHeading.targetCount,
                    frameHeading.targetGroup.count
            );

        }

        if (bestHeading == null) {
            telemetry.addLine("Selected frame: none");
        } else {
            telemetry.addData("Selected Limelight tx", "%.1f deg", bestHeading.limelightTxDeg);
            telemetry.addData("Selected total detections", bestHeading.targetCount);
            telemetry.addData("Selected group count", bestHeading.targetGroup.count);
        }
    }

    private FrameHeading getBestFrameHeading(FrameHeading[] frameHeadings) {
        FrameHeading bestHeading = null;

        for (FrameHeading frameHeading : frameHeadings) {
            if (frameHeading == null || !frameHeading.hasHeading) {
                continue;
            }

            if (bestHeading == null || frameHeading.targetGroup.count > bestHeading.targetGroup.count) {
                bestHeading = frameHeading;
            }
        }

        return bestHeading;
    }

    private List<DetectionTarget> getTargets(LLResult result) {
        List<DetectionTarget> targets = new ArrayList<>();

        if (result == null || !result.isValid()) {
            return targets;
        }

        List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();
        if (detections == null) {
            return targets;
        }

        for (LLResultTypes.DetectorResult d : detections) {
            DetectionTarget target = new DetectionTarget();
            target.className = d.getClassName() == null ? "" : d.getClassName();
            target.txDeg = d.getTargetXDegrees();
            target.tyDeg = d.getTargetYDegrees();
            targets.add(target);
        }

        return targets;
    }

    private TargetGroup findBiggestGroup(List<DetectionTarget> targets) {
        if (targets.isEmpty()) {
            return null;
        }

        TargetGroup bestGroup = null;
        double minTx = Double.POSITIVE_INFINITY;
        double maxTx = Double.NEGATIVE_INFINITY;

        for (DetectionTarget target : targets) {
            minTx = Math.min(minTx, target.txDeg);
            maxTx = Math.max(maxTx, target.txDeg);
        }

        double searchStart = Math.min(SEARCH_MIN_TX, minTx - WINDOW_WIDTH_DEG / 2.0);
        double searchEnd = Math.max(SEARCH_MAX_TX, maxTx + WINDOW_WIDTH_DEG / 2.0);

        for (double start = searchStart; start <= searchEnd - WINDOW_WIDTH_DEG; start += STEP_SIZE_DEG) {
            double end = start + WINDOW_WIDTH_DEG;
            int count = 0;
            double txSum = 0.0;

            for (DetectionTarget target : targets) {
                if (target.txDeg >= start && target.txDeg <= end) {
                    count++;
                    txSum += target.txDeg;
                }
            }

            double windowCenter = start + WINDOW_WIDTH_DEG / 2.0;
            if (bestGroup == null || count > bestGroup.count) {
                bestGroup = new TargetGroup();
                bestGroup.count = count;
                Log.d("BallDetectionTest", "Heading given to limelight: " + bestGroup);
                bestGroup.windowStartDeg = start;
                bestGroup.windowCenterDeg = windowCenter;
                bestGroup.meanTxDeg = count == 0 ? windowCenter : txSum / count;
            }
        }

        return bestGroup;
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

    private static class DetectionTarget {
        String className;
        double txDeg;
        double tyDeg;
    }

    private static class TargetGroup {
        int count;
        double windowStartDeg;
        double windowCenterDeg;
        double meanTxDeg;
    }

    private static class FrameHeading {
        boolean resultValid;
        boolean hasHeading;
        int targetCount;
        double limelightTxDeg;
        TargetGroup targetGroup;
    }
}
