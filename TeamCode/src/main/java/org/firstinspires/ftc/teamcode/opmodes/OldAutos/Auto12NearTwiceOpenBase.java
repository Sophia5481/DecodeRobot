package org.firstinspires.ftc.teamcode.opmodes.OldAutos;

import static org.firstinspires.ftc.teamcode.Robot.RED_ALLIANCE;
import static org.firstinspires.ftc.teamcode.opmodes.ManualDrive.BLUE_GOAL_X;
import static org.firstinspires.ftc.teamcode.opmodes.ManualDrive.BLUE_GOAL_Y;
import static org.firstinspires.ftc.teamcode.opmodes.ManualDrive.RED_GOAL_X;
import static org.firstinspires.ftc.teamcode.opmodes.ManualDrive.RED_GOAL_Y;
import static org.firstinspires.ftc.teamcode.opmodes.ManualDrive.TURRET_MAX_ANGLE;
import static org.firstinspires.ftc.teamcode.opmodes.ManualDrive.TURRET_MIN_ANGLE;

import android.util.Log;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.commands.IdleShooterCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.Memory;
import org.firstinspires.ftc.teamcode.commands.RapidFireShoot;
import org.firstinspires.ftc.teamcode.commands.StartShooterCommand;
import org.firstinspires.ftc.teamcode.commands.StopShootingCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathCallback;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

import java.util.concurrent.TimeUnit;

public abstract class Auto12NearTwiceOpenBase extends CommandOpMode {
    protected Follower follower;
    protected Robot robot;

    protected static final double FULL_SPEED = 0.9;
    protected static final double MEDIUM_SPEED = 0.75;
    protected static final double SLOW = 0.4;

    protected double startHeading;
    protected Point start;
    protected Point end;
    protected double endHeading;

    protected Point scoring;
    protected Point cycleScoring;
//    protected double scoringHeading;
//    protected double cycleScoringHeading;

    protected double intakeHeading;
    protected Point intake1;
    protected Point intake2;
    protected Point intake3;

    protected Point gatePoint;
    protected Point gatePoint2;
    protected Point gatePoint3;
    protected Point scoringControlPoint;

    protected double gateOpeningHeading;

    protected Timing.Timer timer = new Timing.Timer(30, TimeUnit.SECONDS);
    public static long varWaitTime = 0;
    protected int directionNumber(){
        if (isRedAlliance()){
            return 1;
        }
        return -1;
    }
    protected abstract boolean isRedAlliance();
    protected void setupAllianceSpecifics(){
        startHeading = Math.toRadians(-52.0 * directionNumber());
        start = new Point(-58.06, 46.05 * directionNumber(), Point.CARTESIAN);
        end = new Point(-24.5, 32.0 * directionNumber(), Point.CARTESIAN);
        endHeading = Math.toRadians(-90.0 * directionNumber());

        scoring = new Point(-24.5, 12.0 * directionNumber(), Point.CARTESIAN);
        cycleScoring = new Point(-24.5, 12.0 * directionNumber(), Point.CARTESIAN);
//        scoringHeading = Math.toRadians(-43.0 * directionNumber());
//        cycleScoringHeading = Math.toRadians(-45.0 * directionNumber());

        intakeHeading = Math.toRadians(90.0 * directionNumber());
        intake1 = new Point(4.0, 21.5 * directionNumber(), Point.CARTESIAN);
        intake2 = new Point(-23.0, 23.0 * directionNumber(), Point.CARTESIAN);
        intake3 = new Point(-27, 17.5 * directionNumber(), Point.CARTESIAN);

        gatePoint = new Point(-(20.0 - 24.0 + 2.0), (22.0 + 31.5 - 10.0) * directionNumber(), Point.CARTESIAN);
        gatePoint2 = new Point(-(20.0 - 24.0 + 6.0), (22.0 + 31.5 - 10.0) * directionNumber(), Point.CARTESIAN);
        gatePoint3 = new Point(-(20.0 - 24.0 + 6.0), (22.0 + 31.5 - 6.0) * directionNumber(), Point.CARTESIAN);
        scoringControlPoint = new Point(-(20.0 - 24.0 - 1.0), 18.0 * directionNumber(), Point.CARTESIAN);

        gateOpeningHeading = -20.0 * directionNumber();
    };
    protected abstract double getIntakeYIncrement(int cycle);
    protected abstract double getGatePoint2YIncrement(int cycle);

    @Override
    public void initialize() {
        telemetry.addLine("Initializing...");
        telemetry.update();

        setupAllianceSpecifics();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(start.getX(), start.getY(), startHeading));
        follower.setMaxPower(FULL_SPEED);
        Robot.RED_ALLIANCE = isRedAlliance();
        Robot.FAR_AUTO = false;
        Robot.AUTO_NOSORTING = true;
        robot = new Robot(true, hardwareMap);

        while (opModeInInit()) {
            telemetry.addLine("NO SORTING");
            telemetry.update();
        }

        waitForStart();
        timer.start();

        schedule(
                new SequentialCommandGroup(
                        scorePreload(),
                        intakeAndScore(1),
                        intakeAndScore(2),
                        intakeAndScore(3),
                        new ParallelRaceGroup(
                                park(),
                                new StopShootingCommand(robot.getShooter())
                        )
                )
        );
    }

    @Override
    public void run() {
        super.run();
        Log.d("Follower ", "Heading value: " + Math.toDegrees(follower.getTotalHeading()));
        Log.d("Follower ", "Pose: " + follower.getPose().toString());
        follower.update();
        follower.drawOnDashBoard();
        robot.resetCache();
        if (timer.elapsedTime() > 26 && timer.elapsedTime() < 29.9) {
            Memory.autoEndPose = follower.getPose();
            Log.d("Auto End Pose: ", "X: " + Memory.autoEndPose.getX() + "Y: " + Memory.autoEndPose.getY() + "Heading: " + Memory.autoEndPose.getHeading());
        }
        //aiming
        double targetTurretAngle = getTargetTurretAngle();
        if (targetTurretAngle >= TURRET_MIN_ANGLE && targetTurretAngle <= TURRET_MAX_ANGLE) {
            robot.getTurret().setTurretAngle(targetTurretAngle);
        } else {
            telemetry.addData("WARNING: ", "Target out of turret range!");
        }
    }

    protected Command scorePreload() {
        Path toScoring = new Path(new BezierLine(start, scoring));
        toScoring.setLinearHeadingInterpolation(startHeading, intakeHeading);
        toScoring.setPathEndTimeoutConstraint(300);

        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, toScoring),
                        new StartShooterCommand(robot.getShooter(), false).withTimeout(1800)
                ),
                new InstantCommand(() -> {
                    Log.d("Scoring Position: ", "X: " + follower.getPose().getX() + "Y: " + follower.getPose().getY() + "Heading: " + Math.toDegrees(follower.getPose().getHeading()));
                }),
                new RapidFireShoot(robot.getIntake(), robot.getShooter(), false)
        );
    }

    protected Command intakeAndScore(int cycle) {
        Point intakePosition;
        switch (cycle) {
            case 1: intakePosition = intake1; break;
            case 2: intakePosition = intake2; break;
            default: intakePosition = intake3; break;
        }

        Path toIntake = new Path(new BezierLine(cycle == 1 ? scoring : cycleScoring, intakePosition));
        toIntake.setConstantHeadingInterpolation(intakeHeading);
        toIntake.setPathEndTimeoutConstraint(200);

        double intakeYInc = getIntakeYIncrement(cycle);
        Path intakeBalls = new Path(new BezierLine(intakePosition, incrementY(intakePosition, intakeYInc)));
        intakeBalls.setConstantHeadingInterpolation(intakeHeading);

        Path toScoring;
        if (cycle == 1 || cycle == 2) {
            toScoring = new Path(new BezierCurve(gatePoint3, scoringControlPoint, cycleScoring));
        } else {
            toScoring = new Path(new BezierLine(incrementY(intakePosition, intakeYInc), cycleScoring));
        }
        toScoring.setConstantHeadingInterpolation(intakeHeading);
        toScoring.setPathEndTimeoutConstraint(300);

        double gateYInc = getGatePoint2YIncrement(cycle);
        Path toGate = new Path(new BezierCurve(incrementY(intakePosition, intakeYInc), gatePoint, incrementY(gatePoint2, gateYInc)));
        toGate.setConstantHeadingInterpolation(intakeHeading);

        Path openGate = new Path(new BezierLine(incrementY(gatePoint2, gateYInc), gatePoint3));
        openGate.setZeroPowerAccelerationMultiplier(4.5);

        PathChain gatePath = new PathChain(toGate, openGate);
        gatePath.setCallbacks(new PathCallback(0.9, () -> robot.getIntake().stopIntake(), PathCallback.PARAMETRIC, 0));

        return new SequentialCommandGroup(
                new InstantCommand(robot.getIntake()::closeLatch),
                new ParallelRaceGroup(
                        new IdleShooterCommand(robot.getShooter(), false),
                        new FollowPathCommand(follower, toIntake)
                ),
                new ParallelCommandGroup(
                        new IntakeCommand(robot.getIntake(), false, false),
                        new SequentialCommandGroup(
                                new InstantCommand(() -> follower.setMaxPower(MEDIUM_SPEED)),
                                new FollowPathCommand(follower, intakeBalls).withTimeout(2000),
                                new ConditionalCommand(
                                        new SequentialCommandGroup(
                                                new InstantCommand(() -> follower.setMaxPower(MEDIUM_SPEED)),
                                                new WaitCommand(varWaitTime),
                                                new FollowPathCommand(follower, gatePath).withTimeout(3000)
                                        ),
                                        new InstantCommand(),
                                        () -> (cycle == 1 || cycle == 2)
                                ),
                                new InstantCommand(() -> follower.setMaxPower(FULL_SPEED)),
                                new ParallelCommandGroup(
                                        new FollowPathCommand(follower, toScoring).withTimeout(2500),
                                        new StartShooterCommand(robot.getShooter(), false).withTimeout(2000),
                                        new SequentialCommandGroup(
                                                new WaitCommand(400),
                                                new InstantCommand(robot.getIntake()::closeLatch)
                                        )
                                ),
                                new InstantCommand(() -> robot.getIntake().stopIntake())
                        )
                ),
                new InstantCommand(() -> {
                    Log.d("Scoring Position: ", "X: " + follower.getPose().getX() + "Y: " + follower.getPose().getY() + "Heading: " + Math.toDegrees(follower.getPose().getHeading()));
                }),
                new RapidFireShoot(robot.getIntake(), robot.getShooter(), false)
        );
    }

    protected Command park() {
        Path parking = new Path(new BezierLine(cycleScoring, end));
        parking.setLinearHeadingInterpolation(intakeHeading, endHeading);
        return new FollowPathCommand(follower, parking);
    }

    protected Point incrementY(Point point, double increment) {
        return new Point(point.getX(), point.getY() + increment, Point.CARTESIAN);
    }
    public double getAngleToGoal() {
        Pose robotPose = follower.getPose();

        // Get the appropriate goal coordinates based on alliance
        double goalX = RED_ALLIANCE ? RED_GOAL_X : BLUE_GOAL_X;
        double goalY = RED_ALLIANCE ? RED_GOAL_Y : BLUE_GOAL_Y;

        // Calculate the world angle from robot to goal
        double dx = goalX - robotPose.getX();
        double dy = goalY - robotPose.getY();
        telemetry.addData("dx: ", dx);
        telemetry.addData("dy: ", dy);
        double worldAngleToGoal = Math.atan2(dy, dx);
        telemetry.addData("worldAngleToGoal: ", worldAngleToGoal);

        double angleToGoalDeg = Math.toDegrees(worldAngleToGoal);
        telemetry.addData("angleToGoalDeg: ", angleToGoalDeg);
        while (angleToGoalDeg < 0) angleToGoalDeg += 360;
        while (angleToGoalDeg >= 360) angleToGoalDeg -= 360;

        return angleToGoalDeg;
    }
    /**
     * Calculates the required turret angle to aim at the goal.
     * The turret angle is relative to the robot heading
     * @return The turret angle in degrees (-165 to +165)
     */
    public double getTargetTurretAngle() {
        Pose robotPose = follower.getPose();

        // Calculate turret angle relative to robot heading
        // Turret angle = world angle to goal - robot heading
        double robotHeading = Math.toDegrees(robotPose.getHeading());

        // if we don't multiply by -1 it will aim at the opposite goal 😂
        double turretAngle = (getAngleToGoal() - robotHeading);

        double turretAngleDeg = turretAngle;
        while (turretAngleDeg > 180) turretAngleDeg -= 360;
        while (turretAngleDeg < -180) turretAngleDeg += 360;

        return turretAngleDeg;
    }
}