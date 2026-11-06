//package org.firstinspires.ftc.teamcode.opmodes.OldAutos;
//
//import android.util.Log;
//
//import com.arcrobotics.ftclib.command.Command;
//import com.arcrobotics.ftclib.command.CommandOpMode;
//import com.arcrobotics.ftclib.command.InstantCommand;
//import com.arcrobotics.ftclib.command.ParallelCommandGroup;
//import com.arcrobotics.ftclib.command.ParallelRaceGroup;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.arcrobotics.ftclib.command.WaitCommand;
//import com.arcrobotics.ftclib.util.Timing;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//import org.firstinspires.ftc.teamcode.Robot;
//import org.firstinspires.ftc.teamcode.commands.FastAutoSortedShootCommand;
//import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
//import org.firstinspires.ftc.teamcode.commands.IdleShooterCommand;
//import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
//import org.firstinspires.ftc.teamcode.commands.Memory;
//import org.firstinspires.ftc.teamcode.commands.StartShooterCommand;
//import org.firstinspires.ftc.teamcode.commands.StopShootingCommand;
//import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
//import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
//
//import java.util.concurrent.TimeUnit;
//
//
//@Autonomous(name = "Blue Motif Near Auto 9 Artifact ❄️", group = "Auto")
//public class BlueNearAuto extends CommandOpMode {
//    private Follower follower;
//    private Robot robot;
//
//    private static double FULL_SPEED = 1.0;
//    private static double MID_SPEED = 0.8;
//    private static double SLOW = 0.19;
//
//    private int AprilTagID = 21;
//
//    private double startHeading = Math.toRadians(180.0 + 52.0);
//    // After converting to pedro coordinate system, add 72.0 to both x and y
//    // The center of the field is now (72, 72), not (0, 0)
//    Point start = new Point(58.06, 46.05, Point.CARTESIAN);
//    public static Point end = new Point(24.5, 32.0, Point.CARTESIAN);
//    public static double endHeading = Math.toRadians(180.0 + 90.0);
//
//    // The initial scoring position is adjusted for drift
//    public static Point scoring = new Point(24.5, 12.0, Point.CARTESIAN);
//    // scoring point adjusted for drift after intaking
//    // kept the same for now to be consistent with the mirrored red side auto
//    public static Point cycleScoring = new Point(24.5, 12.0, Point.CARTESIAN);
//    public static double scoringHeading = Math.toRadians(180.0 + 43.0);
//    public static double cycleScoringHeading = Math.toRadians(180 + 45.0);
//
//    public static double intakeHeading = Math.toRadians(180.0 - 90.0);
//    public static Point intake1 = new Point(20.0, 18.0, Point.CARTESIAN);
//    public static Point intake2 = new Point(20.0 - 24.0 - 1.0, 22.0, Point.CARTESIAN);
//    public static Point scoringControlPoint = new Point(20.0 - 24.0 - 1.0, 18.0, Point.CARTESIAN);
//    public static Point intake3 = new Point(20.0 - 24.0 * 2 + 2.0, 18.0, Point.CARTESIAN);
//
//    private Timing.Timer timer = new Timing.Timer(30, TimeUnit.SECONDS);
//
//    @Override
//    public void initialize() {
//        telemetry.addLine("Initializing...");
//        telemetry.update();
//
//        follower = new Follower(hardwareMap);
//        follower.setStartingPose(new Pose(start.getX(), start.getY(), startHeading));
//        follower.setMaxPower(FULL_SPEED);
//        Robot.RED_ALLIANCE = false;
//        Robot.FAR_AUTO = false;
//        Robot.AUTO_NOSORTING = false;
//        robot = new Robot(true, hardwareMap);
//
//        while (opModeInInit()) {
////            AprilTagID = robot.getVision().getAprilTagId(Robot.RED_ALLIANCE);
////            telemetry.addLine("AprilTagID: " + AprilTagID);
//            switch (AprilTagID) {
//                case 21:
//                    Memory.motif = "GPP";
//                    break;
//                case 22:
//                    Memory.motif = "PGP";
//                    break;
//                case 23:
//                    Memory.motif = "PPG";
//                    break;
//                default:
//                    break;
//            }
//            telemetry.addLine("Motif: " + Memory.motif);
//            telemetry.update();
//        }
//
//        waitForStart();
//        timer.start();
//
//        schedule(
//            new SequentialCommandGroup(
//                    new InstantCommand(() -> {
//                        follower.setMaxPower(0.6);
//                    }),
//                    scorePreload(),
//                    new InstantCommand(() -> {
//                        follower.setMaxPower(FULL_SPEED);
//                    }),
//                intakeAndScore(1),
//                intakeAndScore(2),
////                        intakeAndScore(3),
//                new ParallelRaceGroup(
//                        park(),
//                        new StopShootingCommand(robot.getShooter())
//                )
//            )
//        );
//    }
//
//    @Override
//    public void run() {
//        super.run();
//        Log.d("Follower ", "Heading value: " + Math.toDegrees(follower.getTotalHeading()));
//        Log.d("Follower ", "Pose: " + follower.getPose().toString());
//        follower.update();
//        follower.drawOnDashBoard();
//        robot.resetCache();
//        // start tracking end pose for correct field centric drive
//        double time = timer.elapsedTime();
//        if (time > 28 && time < 29.5) {
//            Memory.autoEndPose = follower.getPose();
//        }
//    }
//
//    private Command scorePreload() {
//        Path toScoring = new Path(new BezierLine(
//            start,
//            scoring)
//        );
//        toScoring.setLinearHeadingInterpolation(startHeading, scoringHeading);
//        toScoring.setZeroPowerAccelerationMultiplier(2.5);
//
//        return new SequentialCommandGroup(
//            new ParallelRaceGroup(
//                new ParallelCommandGroup(
//                    new FollowPathCommand(
//                        follower, toScoring
//                    )
////                    new InstantCommand(() -> {
////                        robot.getVision().getVisionPortal().close();
////                    }
////                    )
//                ),
//                new StartShooterCommand(robot.getShooter(), false),
//                new ParallelCommandGroup(
//                    new InstantCommand(() -> {
//                        if (Memory.motif.charAt(0) == 'P') {
//                            robot.getFeeder().feedPurpleArtifact();
//                        } else {
//                            robot.getFeeder().feedGreenArtifact();
//                        }
//                    }),
//                    new IntakeCommand(robot.getIntake(), robot.getTransfer())
//                )
//            ),
//            new InstantCommand(() -> {
//                robot.getFeeder().closeGreenFeeder();
//                robot.getFeeder().closePurpleFeeder();
//            }),
//            new WaitCommand(100),
//            new InstantCommand(() -> {
//                Math.toDegrees(follower.getPose().getHeading());
//            }),
//            new FastAutoSortedShootCommand(robot.getFeeder(), robot.getShooter(), robot.getIntake(), robot.getTransfer(), Memory.motif, false)
//        );
//    }
//
//    private Command intakeAndScore(int cycle) {
//        Point intakePosition = intake1;
//        switch (cycle) {
//            case 1:
//                intakePosition = intake1;
//                break;
//            case 2:
//                intakePosition = intake2;
//                break;
//            case 3:
//                intakePosition = intake3;
//                break;
//
//        }
//
//        // drive from scoring to intake position
//        Path toIntake = new Path(new BezierLine(
//            cycle == 1 ? scoring : cycleScoring,
//            intakePosition)
//        );
//        toIntake.setLinearHeadingInterpolation(cycle == 1 ? scoringHeading : cycleScoringHeading, intakeHeading);
//
//        // drive forward to intake balls
//        Path intakeBalls = new Path(new BezierLine(
//            intakePosition,
//            incrementY(intakePosition, cycle >= 2 ? 30.5 : 28.0)
//        ));
//        intakeBalls.setConstantHeadingInterpolation(intakeHeading);
//
//        Path toScoring;
//        if (cycle == 2) {
//            // special path to avoid hitting the gate while driving
//            toScoring = new Path(new BezierCurve(
//                incrementY(intakePosition, cycle >= 2 ? 30.5 : 28.0),
//                scoringControlPoint,
//                cycleScoring)
//            );
//            toScoring.setLinearHeadingInterpolation(intakeHeading, cycleScoringHeading);
//        } else {
//            toScoring = new Path(new BezierLine(
//                incrementY(intakePosition, cycle >= 2 ? 30.5 : 28.0),
//                cycleScoring)
//            );
//            toScoring.setLinearHeadingInterpolation(intakeHeading, cycleScoringHeading);
//        }
//
//        return new SequentialCommandGroup(
//            // drive to intaking
//            new ParallelRaceGroup(
//                //new StopShootingCommand(robot.getShooter()),
//                new IdleShooterCommand(robot.getShooter(), false),
//                new FollowPathCommand(follower, toIntake)
//            ),
//            // intake balls, drive to scoring
//            new ParallelCommandGroup(
//                new IntakeCommand(robot.getIntake(), robot.getTransfer()),
//                new SequentialCommandGroup(
//                    new InstantCommand(() -> {
//                        follower.setMaxPower(SLOW);
//                    }),
//                    new FollowPathCommand(follower, intakeBalls),
////                                new WaitCommand(1500),
//                    new InstantCommand(() -> {
//                        follower.setMaxPower(FULL_SPEED);
//                    }),
//                    // drive to scoring & prepare shooter
//                    // in the middle of the path (750ms later) stop the intake & transfer
//                    new ParallelCommandGroup(
//                        new ParallelRaceGroup(
//                            new FollowPathCommand(follower, toScoring),
//                            new StartShooterCommand(robot.getShooter(), false)
//                        ),
//                        new SequentialCommandGroup(
//                            new InstantCommand(() -> {
//                                if (Memory.motif.charAt(0) == 'P') {
//                                    robot.getFeeder().feedPurpleArtifact();
//                                } else {
//                                    robot.getFeeder().feedGreenArtifact();
//                                }
//                            }),
//                            new WaitCommand(750),
//                            new InstantCommand(robot.getIntake()::stopIntake)
//                        )
//                    )
//                )
//            ),
//            new WaitCommand(750),
//            new InstantCommand(() -> {
//                robot.getFeeder().closeGreenFeeder();
//                robot.getFeeder().closePurpleFeeder();
//            }),
//            // shoot the balls according to the motif pattern
//            new InstantCommand(() -> {
//                Math.toDegrees(follower.getPose().getHeading());
//            }),
//            new FastAutoSortedShootCommand(robot.getFeeder(), robot.getShooter(), robot.getIntake(), robot.getTransfer(), Memory.motif, false)
//        );
//    }
//
//    private Command park() {
//        Path parking = new Path(new BezierLine(
//                cycleScoring,
//                end
//                )
//        );
//        parking.setLinearHeadingInterpolation(cycleScoringHeading, endHeading, 0.5);
//        return new FollowPathCommand(
//            follower, parking
//        );
//    }
//
//    private Point incrementY(Point point, double increment) {
//        return new Point(point.getX(), point.getY() + increment, Point.CARTESIAN);
//    }
//}
