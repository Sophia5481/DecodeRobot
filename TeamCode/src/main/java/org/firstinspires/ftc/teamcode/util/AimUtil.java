package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

public class AimUtil {

    public static class AimResult {
        public final double angleDeg;
        public final double distance;

        public AimResult(double angleDeg, double distance) {
            this.angleDeg = angleDeg;
            this.distance = distance;
        }
    }

    /**
     * Calculates the angle from the robot to the goal in the field coordinate system.
     * @return The angle in degrees (0-360)
     */
    public static AimResult getAngleToGoal(
        Pose robotPose,
        double goalX,
        double goalY
    ) {
        double dx = goalX - robotPose.getX();
        double dy = goalY - robotPose.getY();

        double worldAngleRad = Math.atan2(dy, dx);
        double distanceToGoal = Math.hypot(dx, dy);

        double angleToGoalDeg  = Math.toDegrees(worldAngleRad);

        // normalize to [0, 360)
        while (angleToGoalDeg  < 0) angleToGoalDeg  += 360;
        while (angleToGoalDeg  >= 360) angleToGoalDeg  -= 360;

        return new AimResult(angleToGoalDeg , distanceToGoal);
    }

    /**
     * Calculates the required turret angle to aim at the goal.
     * The turret angle is relative to the robot heading
     */
    public static double getTurretAngle(
        double angleToGateDeg,
        double robotHeadingRad
    ) {
        double robotHeadingDeg = Math.toDegrees(robotHeadingRad);

        double turretAngle = angleToGateDeg - robotHeadingDeg;

        // normalize to [-180, 180]
        while (turretAngle > 180) turretAngle -= 360;
        while (turretAngle < -180) turretAngle += 360;
        return turretAngle;
    }
}
