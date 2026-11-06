package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Log;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.Memory;
import org.firstinspires.ftc.teamcode.commands.RapidFireShoot;
import org.firstinspires.ftc.teamcode.commands.StartShooterCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathCallback;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

import java.util.concurrent.TimeUnit;

@Disabled
public class Blue18FarAuto extends CommandOpMode {
    protected Follower follower;
    protected Robot robot;

    protected static final double FULL_SPEED = 1.0;
    protected static final double SLOW = 0.30;
    protected static final double MEDIUM_SPEED = 0.75;
    protected static final double WALL_INTAKE_SPEED = 0.55;

    protected int AprilTagID = 21;

    protected double startHeading;
    protected Point start;

    protected Point scoringInitial;
    protected Point scoring;
    protected double scoringHeading;

    protected Point intake1;
    protected double intakeHeading;
    protected Point intakeSlow1;

    protected Point beforeForward;
    protected double beforeForwardHeading;
    protected Point wallIntake;
    protected double wallIntakeHeading;
    protected Point wallIntakeSlow;
    protected Point wallIntakeBack;

    protected double intakeOffset = 8.0;
    protected double intakeStrafeDistance = 14.0;

    protected Point parking;
    protected double parkingHeading;

    protected Timing.Timer timer = new Timing.Timer(30, TimeUnit.SECONDS);
    public static long varWaitTime = 0;

    private static int directionNumber = -1;

    protected void setupAllianceSpecifics() {
        startHeading = Math.toRadians(0);
        start = new Point(60.0, 12.0 * directionNumber, Point.CARTESIAN);

        scoringInitial = new Point(55.0, 12.0 * directionNumber, Point.CARTESIAN);
        scoring = new Point(55.0, (12.0 + 4.0) * directionNumber, Point.CARTESIAN);
        scoringHeading = Math.toRadians(-21.0 * directionNumber);

        intake1 = new Point(36.0 - 2.0, 25.0 * directionNumber, Point.CARTESIAN);
        intakeHeading = Math.toRadians(90.0 * directionNumber);
        intakeSlow1 = new Point(36.0 - 2.0, 59.5 * directionNumber, Point.CARTESIAN);

        beforeForward = new Point(53.0 - 20.0, 16.0 * directionNumber, Point.CARTESIAN);
        beforeForwardHeading = Math.toRadians(90 * directionNumber);
        wallIntake = new Point(59.0, 44.0 * directionNumber, Point.CARTESIAN);
        wallIntakeHeading = Math.toRadians(90 * directionNumber);
        wallIntakeSlow = new Point(59.0, (44.0 + 12.0) * directionNumber, Point.CARTESIAN);
        wallIntakeBack = new Point(59.0, (44.0 + 8.0) * directionNumber, Point.CARTESIAN);

        intakeOffset = -8.0;

        parking = new Point(40.0, 14.0 * directionNumber, Point.CARTESIAN);
        parkingHeading = Math.toRadians(180.0);
    }

    @Override
    public void initialize() {
        telemetry.addLine("Initializing...");
        telemetry.update();

        setupAllianceSpecifics();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(start.getX(), start.getY(), startHeading));
        follower.setMaxPower(FULL_SPEED);
        Robot.RED_ALLIANCE = false;
        Robot.FAR_AUTO = true;
        Robot.AUTO_NOSORTING = true;
        robot = new Robot(true, hardwareMap);

        while (opModeInInit()) {
            switch (AprilTagID) {
                case 21:
                    Memory.motif = "GPP";
                    break;
                case 22:
                    Memory.motif = "PGP";
                    break;
                case 23:
                    Memory.motif = "PPG";
                    break;
                default:
                    break;
            }
            telemetry.addLine("Motif: " + Memory.motif);
            telemetry.update();
        }

        waitForStart();
        timer.start();

        schedule(
                scorePreload(),
                scoreLoadingZone(),
                scoreWall(1),
                scoreWall(2),
                scoreWall(3),
                scoreWall(4),
                park()
        );
    }



    @Override
    public void run() {
        super.run();
        Log.d("Follower ", "Heading value: " + Math.toDegrees(follower.getTotalHeading()));
        Log.d("Follower ", "Pose: " + follower.getPose().toString());
        follower.update();
        robot.resetCache();
        if (timer.elapsedTime() > 27 && timer.elapsedTime() < 29.9) {
            Memory.autoEndPose = follower.getPose();
            Log.d("Auto End Pose: ", "X: " + Memory.autoEndPose.getX() + "Y: " + Memory.autoEndPose.getY() + "Heading: " + Memory.autoEndPose.getHeading());
        }
    }

    protected Command scorePreload() {
        Path toScoring = new Path(new BezierLine(
                start,
                scoringInitial)
        );
        toScoring.setLinearHeadingInterpolation(startHeading, scoringHeading);
        toScoring.setPathEndTimeoutConstraint(300);

        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, toScoring),
                        new StartShooterCommand(robot.getShooter(), true).withTimeout(1800)
                ),
                new ParallelRaceGroup(
                        new RapidFireShoot(robot.getIntake(), robot.getShooter(), true),
                        new IntakeCommand(robot.getIntake(), false, false)
                )
        );
    }

    protected Command scoreLoadingZone() {
        Point intakePosition = intake1;
        Point intakeSlowPosition = intakeSlow1;

        Path toLoadingZone = new Path(new BezierLine(
                scoringInitial,
                intakePosition)
        );
        toLoadingZone.setLinearHeadingInterpolation(scoringHeading, intakeHeading);
        toLoadingZone.setPathEndTimeoutConstraint(50);

        Path intakeBalls = new Path(new BezierLine(
                intakePosition,
                intakeSlowPosition
        ));
        intakeBalls.setConstantHeadingInterpolation(intakeHeading);
        intakeBalls.setPathEndTimeoutConstraint(50);
        intakeBalls.setZeroPowerAccelerationMultiplier(4.5);

        Path shootLoadingZone = new Path(new BezierLine(
                intakeSlowPosition,
                scoring
        ));
        shootLoadingZone.setLinearHeadingInterpolation(intakeHeading, scoringHeading);
        shootLoadingZone.setPathEndTimeoutConstraint(300);

        return new SequentialCommandGroup(
                new InstantCommand(robot.getIntake()::closeLatch),
                new FollowPathCommand(follower, toLoadingZone),
                new ParallelCommandGroup(
                        new IntakeCommand(robot.getIntake(), false, false),
                        new SequentialCommandGroup(
                                new InstantCommand(() -> follower.setMaxPower(MEDIUM_SPEED)),
                                new FollowPathCommand(follower, intakeBalls),
                                new InstantCommand(() -> follower.setMaxPower(FULL_SPEED)),
                                new WaitCommand(200),
                                new ParallelCommandGroup(
                                        new FollowPathCommand(follower, shootLoadingZone),
                                        new WaitCommand(varWaitTime),
                                        new StartShooterCommand(robot.getShooter(), true).withTimeout(2000),
                                        new SequentialCommandGroup(
                                                new WaitCommand(400),
                                                new InstantCommand(robot.getIntake()::openLatch)
                                        )
                                ),
                                new InstantCommand(robot.getIntake()::stopIntake)
                        )
                ),
                new ParallelRaceGroup(
                        new RapidFireShoot(robot.getIntake(), robot.getShooter(), true),
                        new IntakeCommand(robot.getIntake(), false, true)
                )
        );
    }

    protected Command scoreWall(int cycle) {
        Path toWallZone = new Path(new BezierLine(
                (cycle == 1) ? scoringInitial : scoring,
                (cycle == 3 || cycle == 2) ? incrementX(wallIntake, intakeOffset) : wallIntake
        ));
        toWallZone.setLinearHeadingInterpolation(beforeForwardHeading, wallIntakeHeading, 0.5);
        toWallZone.setPathEndTimeoutConstraint(50);

        Path intakeWall = new Path(new BezierLine(
                (cycle == 3 || cycle == 2) ? incrementX(wallIntake, intakeOffset) : wallIntake,
                (cycle == 3 || cycle == 2) ? incrementX(wallIntakeSlow, intakeOffset) : wallIntakeSlow
        ));
        intakeWall.setConstantHeadingInterpolation(wallIntakeHeading);
        intakeWall.setPathEndTimeoutConstraint(50);
        intakeWall.setZeroPowerAccelerationMultiplier(4.5);

//        Path goBack = new Path(new BezierLine(
//                wallIntakeSlow,
//                (cycle == 3 || cycle == 2) ? incrementX(wallIntakeBack, intakeOffset + intakeStrafeDistance) : wallIntakeBack
//        ));
//        if ((cycle == 2 || cycle == 3) && intakeStrafeDistance > 4.0) {
//            goBack.setPathEndTValueConstraint(0.8);
//        }
//        goBack.setConstantHeadingInterpolation(wallIntakeHeading);
//        goBack.setPathEndTimeoutConstraint(50);
//        goBack.setZeroPowerAccelerationMultiplier(4.5);
//
//        Path secondIntakeWall = new Path(new BezierLine(
//                (cycle == 3 || cycle == 2) ? incrementX(wallIntakeBack, intakeOffset + intakeStrafeDistance) : wallIntakeBack,
//                (cycle == 3 || cycle == 2) ? incrementX(wallIntakeSlow, intakeOffset + intakeStrafeDistance) : wallIntakeSlow
//        ));
//        secondIntakeWall.setConstantHeadingInterpolation(wallIntakeHeading);
//        secondIntakeWall.setPathEndTimeoutConstraint(50);
//        secondIntakeWall.setZeroPowerAccelerationMultiplier(4.5);

        PathChain WallIntakePath = new PathChain(toWallZone, intakeWall);

        WallIntakePath.setCallbacks(
                new PathCallback(0, () -> follower.setMaxPower(MEDIUM_SPEED), PathCallback.PARAMETRIC, 0),
                new PathCallback(0, () -> follower.setMaxPower(SLOW), PathCallback.PARAMETRIC, 1),
                new PathCallback(0, () -> follower.setMaxPower(FULL_SPEED), PathCallback.PARAMETRIC, 2)
        );

        Path shootWallZone = new Path(new BezierLine(
                (cycle == 3 || cycle == 2) ? incrementX(wallIntakeSlow, intakeOffset + intakeStrafeDistance) : wallIntakeSlow,
                scoring
        ));
        shootWallZone.setLinearHeadingInterpolation(wallIntakeHeading, scoringHeading);
        shootWallZone.setPathEndTimeoutConstraint(300);

        return new SequentialCommandGroup(
                new InstantCommand(robot.getIntake()::closeLatch),
                new ParallelCommandGroup(
                        new IntakeCommand(robot.getIntake(), false, false),
                        new SequentialCommandGroup(
                                new InstantCommand(() -> follower.setMaxPower(FULL_SPEED)),
                                new FollowPathCommand(follower, WallIntakePath, true).withTimeout((cycle == 3 || cycle == 2) ? 3300 : 2100),
                                new InstantCommand(() -> follower.setMaxPower(FULL_SPEED)),
                                new WaitCommand(200),
                                new ParallelCommandGroup(
                                        new FollowPathCommand(follower, shootWallZone),
                                        new WaitCommand(varWaitTime),
                                        new StartShooterCommand(robot.getShooter(), true).withTimeout(1500),
                                        new SequentialCommandGroup(
                                                new WaitCommand(400),
                                                new InstantCommand(robot.getIntake()::openLatch)
                                        )
                                ),
                                new InstantCommand(robot.getIntake()::stopIntake)
                        )
                ),
                new ParallelRaceGroup(
                        new RapidFireShoot(robot.getIntake(), robot.getShooter(), true),
                        useSlowIntakeAtEnd() ? new IntakeCommand(robot.getIntake(), false, true) : new IntakeCommand(robot.getIntake(), false, false))
                );
    }

    protected boolean useSlowIntakeAtEnd() {
        return true;
    }

    protected Command park() {
        Path toParking = new Path(new BezierLine(scoring, parking));
        toParking.setLinearHeadingInterpolation(scoringHeading, parkingHeading);
        return new SequentialCommandGroup(new FollowPathCommand(follower, toParking));
    }

    protected Point incrementX(Point point, double increment) {
        return new Point(point.getX() + increment, point.getY(), Point.CARTESIAN);
    }
}