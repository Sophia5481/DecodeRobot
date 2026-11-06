package org.firstinspires.ftc.teamcode.opmodes.OldAutos;

import android.util.Log;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

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

@Disabled
@Autonomous(name = "Blue Near 15 Ball Auto 🪼", group = "Auto")
public class BlueNear15BallAuto extends CommandOpMode {
    private Follower follower;
    private Robot robot;

    private static double FULL_SPEED = 0.9;
    private static double MEDIUM_SPEED = 0.75;
    private static double SLOW = 0.4;

//    private int AprilTagID = 21;

    private double startHeading = Math.toRadians(180.0 + 52.0);
    // After converting to pedro coordinate system, add 72.0 to both x and y
    // The center of the field is now (72, 72), not (0, 0)
    Point start = new Point(58.06, 46.05, Point.CARTESIAN);
    public static Point end = new Point(24.5, 32.0, Point.CARTESIAN);
    public static double endHeading = Math.toRadians(180.0 + 90.0);

    // The initial scoring position is adjusted for drift
    public static Point scoring = new Point(24.5, 12.0, Point.CARTESIAN);
    // scoring point adjusted for drift after intaking
    public static Point cycleScoring = new Point(24.5, 12.0, Point.CARTESIAN);
    public static double scoringHeading = Math.toRadians(180.0 + 43.0);
    public static double cycleScoringHeading = Math.toRadians(180 + 45.0);

    public static double intakeHeading = Math.toRadians(180.0 - 90.0);
    // intake the second row of balls first before opening the gate
    public static Point intakeControlPoint1 = new Point(20.0 - 24.0 - 1.0 + 1.5, 0.0, Point.CARTESIAN);
    public static Point intake1 = new Point(20.0 - 24.0 - 1.0 + 1.5, 20.0, Point.CARTESIAN);

    public static Point intakeControlPoint2 = new Point(20.0, 0.0, Point.CARTESIAN);
    public static Point intake2 = new Point(20.0, 16.0, Point.CARTESIAN);

    public static Point intakeControlPoint3 = new Point(20.0 - 24.0 * 2, 0.0, Point.CARTESIAN);
    public static Point intake3 = new Point(20.0 - 24.0 * 2, 21.5, Point.CARTESIAN);
    // open gate points
    // drives back 12 inches
    public static Point gatePoint = new Point(20.0 - 24.0 - 1.0 + 3.0, 22.0 + 31.5 - 10.0, Point.CARTESIAN);
    // aligns with gate
    public static Point gatePoint2 = new Point(20.0 - 24.0 - 1.0 + 7.0, 22.0 + 31.5 - 10.0, Point.CARTESIAN);
    public static Point gatePoint3 = new Point(20.0 - 24.0 - 1.0 + 7.0, 22.0 + 31.5 - 6.0, Point.CARTESIAN);
    public static Point scoringControlPoint = new Point(20.0 - 24.0 - 1.0, 18.0, Point.CARTESIAN);

    public static double gateOpeningHeading = 180.0 - 20.0;

    public static Point gateIntakeControlPoint = new Point(12.5, 12.0, Point.CARTESIAN);
    public static Point openGateForIntake = new Point(0, 47.5, Point.CARTESIAN);
    public static Point gateIntake = new Point(-8, 47.5, Point.CARTESIAN);
    public static double gateIntakeHeading = Math.toRadians(180 - 90 + 20.0);

    private Timing.Timer timer = new Timing.Timer(30, TimeUnit.SECONDS);

    public static long varWaitTime = 0;

    @Override
    public void initialize() {
        telemetry.addLine("Initializing...");
        telemetry.update();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(start.getX(), start.getY(), startHeading));
        follower.setMaxPower(FULL_SPEED);
        Robot.RED_ALLIANCE = false;
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
                        gateCycle(),
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
        // start tracking end pose for correct field centric drive
        //
        if (timer.elapsedTime() > 26 && timer.elapsedTime() < 29.9) {
            Memory.autoEndPose = follower.getPose();
            Log.d("Auto End Pose: ", "X: " + Memory.autoEndPose.getX() + "Y: " + Memory.autoEndPose.getY() + "Heading: " + Memory.autoEndPose.getHeading());
        }
    }

    private Command scorePreload() {
        Path toScoring = new Path(new BezierLine(
                start,
                scoring)
        );
        toScoring.setLinearHeadingInterpolation(startHeading, scoringHeading);

        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new FollowPathCommand(
                                follower, toScoring
                        ),
                        new StartShooterCommand(robot.getShooter(), false).withTimeout(1800)
                ),
                new InstantCommand(() -> {
                    Log.d("Scoring Position: ", "X: " + follower.getPose().getX() + "Y: " + follower.getPose().getY() + "Heading: " + Math.toDegrees(follower.getPose().getHeading()));
                }),
                new ParallelRaceGroup(
                    new RapidFireShoot(robot.getIntake(), robot.getShooter(), false),
                    new IntakeCommand(robot.getIntake(), false, false)
                )
        );
    }

    private Command gateCycle() {
        Path toGate = new Path(new BezierCurve(
            scoring,
            gateIntakeControlPoint,
            openGateForIntake
        ));
        toGate.setLinearHeadingInterpolation(scoringHeading, gateIntakeHeading);

        Path intakePath = new Path(new BezierLine(
            openGateForIntake,
            gateIntake
        ));

        Path toScoring = new Path(new BezierLine(
            gateIntake,
            scoring
        ));
        toScoring.setLinearHeadingInterpolation(gateIntakeHeading, scoringHeading);

        return new SequentialCommandGroup(
                new InstantCommand(robot.getIntake()::closeLatch),
            new ParallelRaceGroup(
                new FollowPathCommand(follower, toGate).withTimeout(5000),
                new IdleShooterCommand(robot.getShooter(), false),
                    new IntakeCommand(robot.getIntake(), false, false)
            ),
            new ParallelRaceGroup(
                new FollowPathCommand(follower, intakePath),
                    new IntakeCommand(robot.getIntake(), false, false)
            ),
            new ParallelRaceGroup(
                new FollowPathCommand(follower, toScoring),
                new StartShooterCommand(robot.getShooter(), false),
                    new IntakeCommand(robot.getIntake(), false, false),
                new SequentialCommandGroup(
                    new WaitCommand(400),
                        new InstantCommand(robot.getIntake()::openLatch)
                )
            ),
            new InstantCommand(() -> {
                Log.d("Scoring Position: ", "X: " + follower.getPose().getX() + "Y: " + follower.getPose().getY() + "Heading: " + Math.toDegrees(follower.getPose().getHeading()));
            }),
            new ParallelRaceGroup(
                    new RapidFireShoot(robot.getIntake(), robot.getShooter(), false),
                    new IntakeCommand(robot.getIntake(), false, false)
            )

        );
    }

    private Command intakeAndScore(int cycle) {
        Point intakePosition = intake1;
        Point intakeControlPoint = intakeControlPoint1;
        switch (cycle) {
            case 1:
                intakePosition = intake1;
                intakeControlPoint = intakeControlPoint1;
                break;
            case 2:
                intakePosition = intake2;
                intakeControlPoint = intakeControlPoint2;
                break;
            case 3:
                intakePosition = intake3;
                intakeControlPoint = intakeControlPoint3;
                break;

        }

        // drive from scoring to intake position
        Path toIntake = new Path(new BezierLine(
                cycle == 1 ? scoring : cycleScoring,
                intakePosition)
        );
        toIntake.setLinearHeadingInterpolation(cycle == 1 ? scoringHeading : cycleScoringHeading, intakeHeading);

        // drive forward to intake balls
        Path intakeBalls = new Path(new BezierLine(
                intakePosition,
                incrementY(intakePosition, cycle == 1 || cycle == 3 ? 31.5 : 29)
        ));
        intakeBalls.setConstantHeadingInterpolation(intakeHeading);

        PathChain intakePathChain = new PathChain(
            toIntake,
            intakeBalls
        );

        Path smoothIntakePath = new Path(new BezierCurve(
            cycle == 1 ? scoring : cycleScoring,
            intakePosition,
            intakeControlPoint,
            incrementY(intakePosition, cycle == 1 || cycle == 3 ? 31.5 : 29)
        ));

        Path toScoring;
        // drive from intake position to scoring position
        toScoring = new Path(new BezierLine(
            incrementY(intakePosition, cycle == 1 || cycle == 3 ? 31.5 : 29),
            cycleScoring)
        );
        toScoring.setLinearHeadingInterpolation(intakeHeading, cycleScoringHeading);

        Path toGate = new Path(new BezierCurve(
                incrementY(intakePosition, cycle == 1 || cycle == 3 ? 31.5 : 29),
                gatePoint,
                gatePoint2
        ));
        toGate.setConstantHeadingInterpolation(intakeHeading);

        Path openGate = new Path(new BezierLine(
            gatePoint2,
            gatePoint3
        ));

        openGate.setZeroPowerAccelerationMultiplier(4.5);

        PathChain gatePath = new PathChain(
            toGate, openGate
        );
        gatePath.setCallbacks(new PathCallback(
            0.9, () -> {robot.getIntake().stopIntake();}, PathCallback.PARAMETRIC, 0
        ));

        return new SequentialCommandGroup(
                // drive to intaking
                // close purple feeder before intaking so we have the hardware limit
                new InstantCommand(robot.getIntake()::closeLatch),
                new ParallelRaceGroup(
//                new StopShootingCommand(robot.getShooter()),
                        new IdleShooterCommand(robot.getShooter(), false),
                        new FollowPathCommand(follower, smoothIntakePath)
                ),
                // intake balls, drive to scoring
                new ParallelCommandGroup(
                        //new IntakeCommand(robot.getIntake(), robot.getTransfer()),
                        new SequentialCommandGroup(
                                new InstantCommand(() -> {
                                    follower.setMaxPower(MEDIUM_SPEED);
                                }),

                                new ParallelRaceGroup(
                                        new IntakeCommand(robot.getIntake(), false, false).withTimeout(2200),
                                    new FollowPathCommand(follower, intakePathChain).withTimeout(5000)
                                ),
                                new InstantCommand(() -> {
                                    follower.setMaxPower(FULL_SPEED);
                                }),
                                // drive to scoring & prepare shooter
                                // in the middle of the path (750ms later) stop the intake & transfer
                                new ParallelCommandGroup(
                                    // follow the path to score while turning on the intake
                                    // the whole sequence ends as soon as the first command is finished
                                    // we need parallel race group bc IntakeCommand runs indefinitely without a timeout limit
                                        new ParallelRaceGroup(
                                                new IntakeCommand(robot.getIntake(), false, false),
                                            new ParallelCommandGroup(
                                                new FollowPathCommand(follower, toScoring),
                                                new StartShooterCommand(robot.getShooter(), false).withTimeout(2000),
                                                new SequentialCommandGroup(
                                                    new WaitCommand(750),
                                                        new InstantCommand(robot.getIntake()::openLatch)
                                                )
                                            )
                                        )
                                        // TODO: A new change for the intake w/ hardware limit of 3: don't need to reverse intake anymore!
//                                        new SequentialCommandGroup(
//                                                new WaitCommand(700),
//                                                new InstantCommand(robot.getIntake()::stopIntake),
//                                                new IntakeReverseCommand(robot.getIntake(), robot.getTransfer(), 350)
//                                                //new ReversePurpleCommand(robot.getTransfer(), 500)
//                                        )
                                )
                        )
                ),
                new InstantCommand(() -> {
                    Log.d("Scoring Position: ", "X: " + follower.getPose().getX() + "Y: " + follower.getPose().getY() + "Heading: " + Math.toDegrees(follower.getPose().getHeading()));
                }),
                new ParallelRaceGroup(
                        new RapidFireShoot(robot.getIntake(), robot.getShooter(), false),
                        new IntakeCommand(robot.getIntake(), false, false)
                )
        );
    }

    private Command park() {
        Path parking = new Path(new BezierLine(
                cycleScoring,
                end
        )
        );
        parking.setLinearHeadingInterpolation(cycleScoringHeading, endHeading, 0.5);
        return new FollowPathCommand(
                follower, parking
        );
    }

    private Point incrementY(Point point, double increment) {
        return new Point(point.getX(), point.getY() + increment, Point.CARTESIAN);
    }
}