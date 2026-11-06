package org.firstinspires.ftc.teamcode.tests;



import android.util.Log;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pinpoint.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pinpoint.GoBildaPinpointDriver;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
@TeleOp(name = "Megatag2Test", group = "Test")
public class Megatag2Test extends LinearOpMode {

    private Limelight3A limelight;
    private GoBildaPinpointDriver pinpoint;

    private Follower follower;
    // CHANGE THESE
    private static final String LIMELIGHT_NAME = "Ethernet Device";
    private static final String PINPOINT_NAME = "pinpoint";
    private static final int APRILTAG_PIPELINE = 0;

    // Replace with your real pod offsets in mm
    // X pod: left positive, right negative
    // Y pod: forward positive, backward negative
    private static final double X_POD_OFFSET_MM = 0.0;
    private static final double Y_POD_OFFSET_MM = 0.0;

    // Auto-seed settings
    private static final int REQUIRED_VALID_FRAMES = 10;
    private static final boolean AUTO_SEED_ONCE = true;

    private boolean hasSeeded = false;
    private int validFrameCount = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, PINPOINT_NAME);

        // Pinpoint setup
        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD
        );

        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        pinpoint.setOffsets(X_POD_OFFSET_MM, Y_POD_OFFSET_MM);

        telemetry.addLine("Initializing Pinpoint...");
        telemetry.addLine("Keep robot completely still.");
        telemetry.update();



        pinpoint.resetPosAndIMU();
        sleep(300);

        // Limelight setup
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();

        follower = new Follower(hardwareMap);

        telemetry.addLine("Ready.");
        telemetry.addLine("No gamepad required.");
        telemetry.addLine("Robot will auto-seed Pinpoint from Limelight once stable.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Pinpoint only updates when update() is called
            pinpoint.update();

            double ppXmm = pinpoint.getPosX();
            double ppYmm = pinpoint.getPosY();

            // getHeading() is radians in your driver
            double ppYawDeg = wrapDeg(Math.toDegrees(pinpoint.getHeading()));

            // Feed Pinpoint heading into Limelight MT2
            limelight.updateRobotOrientation(ppYawDeg);

            LLResult result = limelight.getLatestResult();
            Pose3D mt2 = null;
            Pose3D mt1 = null;

            if (result != null && result.isValid()) {
                mt2 = result.getBotpose_MT2();
                mt1 = result.getBotpose();
            }

            follower.update();

            double followerYawDeg = wrapDeg(Math.toDegrees(follower.getPose().getHeading()));
            double followerTotalYawDeg = Math.toDegrees(follower.getTotalHeading());

            double s = Math.toDegrees(wrapRad(follower.getPose().getHeading()));

            telemetry.addLine("=== Pinpoint ===");
            telemetry.addData("PP X (mm)", ppXmm);
            telemetry.addData("PP Y (mm)", ppYmm);
            telemetry.addData("PP Yaw (deg)", ppYawDeg);
            telemetry.addData("PP Status", pinpoint.getDeviceStatus());
            telemetry.addData("PP Freq (Hz)", pinpoint.getFrequency());

            telemetry.addLine("=== Follower ===");
            telemetry.addData("Follower X (mm)", follower.getPose().getX());
            telemetry.addData("Follower Y (mm)", follower.getPose().getY());
            telemetry.addData("Follower Yaw (deg)", followerYawDeg);
            telemetry.addData("Follower Yaw (deg)s", s);


            if (mt1 != null) {
                double mt1X = mt1.getPosition().x * 1000.0;
                double mt1Y = mt1.getPosition().y * 1000.0;
                double mt1Yaw = mt1.getOrientation().getYaw(AngleUnit.DEGREES);

                telemetry.addLine("=== MT1 ===");
                telemetry.addData("X (mm)", mt1X);
                telemetry.addData("Y (mm)", mt1Y);
                telemetry.addData("Yaw (deg)", mt1Yaw);
            } else {
                telemetry.addLine("MT1: null");
            }
            if (mt2 != null) {
                double llXmm = mt2.getPosition().x * 1000.0;
                double llYmm = mt2.getPosition().y * 1000.0;
                double llYawDeg = wrapDeg(mt2.getOrientation().getYaw(AngleUnit.DEGREES));

                telemetry.addLine("=== Limelight MT2 ===");
                telemetry.addData("LL X (mm)", llXmm);
                telemetry.addData("LL Y (mm)", llYmm);
                telemetry.addData("LL Yaw (deg)", llYawDeg);
                double mt1X = mt1.getPosition().x * 1000.0;
                double mt1Y = mt1.getPosition().y * 1000.0;
                double dx = llXmm - mt1X;
                double dy = llYmm - mt1Y;
                double dh = angleDiffDeg(llYawDeg, ppYawDeg);

                telemetry.addLine("=== Difference (LL - PP) ===");
                telemetry.addData("dX (mm)", dx);
                telemetry.addData("dY (mm)", dy);
                telemetry.addData("dYaw (deg)", dh);

                validFrameCount++;

                // Automatically seed Pinpoint once after several stable valid frames
                if (AUTO_SEED_ONCE && !hasSeeded && validFrameCount >= REQUIRED_VALID_FRAMES) {
                    pinpoint.setPosition(new Pose2D(
                            DistanceUnit.MM,
                            llXmm,
                            llYmm,
                            AngleUnit.DEGREES,
                            llYawDeg
                    ));
                    hasSeeded = true;
                }

                telemetry.addData("Valid MT2 Frames", validFrameCount);
                telemetry.addData("Auto-Seeded", hasSeeded);
            } else {
                validFrameCount = 0;
                telemetry.addLine("No valid MT2 pose");
                telemetry.addData("Valid MT2 Frames", validFrameCount);
                telemetry.addData("Auto-Seeded", hasSeeded);
            }

            telemetry.update();
            sleep(20);
        }

        limelight.stop();
    }

    private static double wrapDeg(double deg) {
        while (deg > 180.0) deg -= 360.0;
        while (deg < -180.0) deg += 360.0;
        return deg;
    }

    private static double angleDiffDeg(double target, double current) {
        return wrapDeg(target - current);



    }
    public static double wrapAngle(double r) {
        while (r >= 360) r -= 360;
        while (r < 0) r += 360;
        return r;
    }
    private static double wrapRad(double r) {
        while (r > Math.PI) r -= 2.0 * Math.PI;
        while (r < -Math.PI) r += 2.0 * Math.PI;
        return r;
    }

}