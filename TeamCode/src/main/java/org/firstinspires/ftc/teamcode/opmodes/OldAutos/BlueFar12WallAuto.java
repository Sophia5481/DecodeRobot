//package org.firstinspires.ftc.teamcode.opmodes;
//
//import android.util.Log;
//
//import com.arcrobotics.ftclib.command.Command;
//import com.arcrobotics.ftclib.command.CommandOpMode;
//import com.arcrobotics.ftclib.command.ConditionalCommand;
//import com.arcrobotics.ftclib.command.InstantCommand;
//import com.arcrobotics.ftclib.command.ParallelCommandGroup;
//import com.arcrobotics.ftclib.command.ParallelRaceGroup;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.arcrobotics.ftclib.command.WaitCommand;
//import com.arcrobotics.ftclib.command.WaitUntilCommand;
//import com.arcrobotics.ftclib.util.Timing;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//import org.firstinspires.ftc.teamcode.Robot;
//import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
//import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
//import org.firstinspires.ftc.teamcode.commands.IntakeReverseCommand;
//import org.firstinspires.ftc.teamcode.commands.Memory;
//import org.firstinspires.ftc.teamcode.commands.RapidFireShoot;
//import org.firstinspires.ftc.teamcode.commands.ShootThreeCommand;
//import org.firstinspires.ftc.teamcode.commands.SlowIntakeCommand;
//import org.firstinspires.ftc.teamcode.commands.StartShooterCommand;
//import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
//import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathCallback;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
//import org.firstinspires.ftc.teamcode.subsystems.Shooter;
//
//import java.util.concurrent.TimeUnit;
//
//@Autonomous(name = "Blue Far 12 Wall Auto 🐳", group = "Auto")
//public class BlueFar12WallAuto extends CommandOpMode {
//    private Follower follower;
//    private Robot robot;
//
//    private static double FULL_SPEED = 1.0;
//    private static double SLOW = 0.30;
//    private static double MEDIUM_SPEED = 0.75;
//
//    private int AprilTagID = 21;
//
//    private double startHeading = Math.toRadians(180.0);
//    // After converting to pedro coordinate system, add 72.0 to both x and y
//    // The center of the field is now (72, 72), not (0, 0)
//    Point start = new Point(-60.0, 12.0, Point.CARTESIAN);
//
//    // The preload cycle is always off to the left compared to the other cycles
//
//    // the scoring position for the preload cycle
//    public static Point scoringInitial = new Point(-55.0, 12.0, Point.CARTESIAN);
//    // scoring position accounted for drift after the preload cycle
//    public static Point scoring = new Point(-55.0, 12.0 + 4.0, Point.CARTESIAN);
//    public static double scoringHeading = Math.toRadians(180.0 + 22.0);
//
//    public static Point intake1 = new Point(-36.0 + 2.0, 25.0, Point.CARTESIAN);
//    public static double intakeHeading = Math.toRadians(180.0 -  90.0);
//    public static Point intakeSlow1 = new Point(-36.0 + 2.0, 59.5, Point.CARTESIAN);
//
//    public static Point beforeForward = new Point(-53.0 + 20.0, 16.0, Point.CARTESIAN);
//    public static double beforeForwardHeading = Math.toRadians(180.0);
//    public static Point wallIntake = new Point(-61.0, 43.0, Point.CARTESIAN);
//    public static double wallIntakeHeading = Math.toRadians(180.0 - 90.0);
//    public static Point wallIntakeSlow = new Point(-61.0, 44.0 + 12.0, Point.CARTESIAN);
//    public static Point wallIntakeBack = new Point(-61.0, 44.0 + 8.0, Point.CARTESIAN);
//
//    public static Point parking = new Point(-40.0, 14.0, Point.CARTESIAN);
//    public static double parkingHeading = Math.toRadians(180.0);
//
////    public static Point intake2 = new Point(20.0 - 24.0 - 1.0 + 3.0, 19.0, Point.CARTESIAN);
////    public static Point intakeSlow2 = new Point(20 - 24.0 - 1.0, 19.0 + 30.0, Point.CARTESIAN);
//
//    private Timing.Timer timer = new Timing.Timer(30, TimeUnit.SECONDS);
//
//    public static long varWaitTime = 0;
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
//        Robot.FAR_AUTO = true;
//        Robot.AUTO_NOSORTING = true;
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
//                new SequentialCommandGroup(
//                        scorePreload(),
//                        scoreWall(true),
//                        scoreWall(false),
//                        scoreWall(false),
//                        park()
//                )
//        );
//    }
//
//    @Override
//    public void run() {
//        super.run();
//        Log.d("Follower ", "Heading value: " + Math.toDegrees(follower.getTotalHeading()));
//        Log.d("Follower ", "Pose: " + follower.getPose().toString());
//        follower.update();
//        robot.resetCache();
//        if (timer.elapsedTime() > 27 && timer.elapsedTime() < 29.9) {
//            Memory.autoEndPose = follower.getPose();
//            Log.d("Auto End Pose: ", "X: " + Memory.autoEndPose.getX() + "Y: " + Memory.autoEndPose.getY() + "Heading: " + Memory.autoEndPose.getHeading());
//        }
//    }
//
//    private Command scorePreload() {
//        Path toScoring = new Path(new BezierLine(
//                start,
//                scoringInitial)
//        );
//        toScoring.setLinearHeadingInterpolation(startHeading, scoringHeading);
//
//        return new SequentialCommandGroup(
//                new ParallelCommandGroup(
//                        new ParallelCommandGroup(
//                                new FollowPathCommand(
//                                        follower, toScoring
//                                )
////                                new InstantCommand(() -> {
////                                    robot.getVision().getVisionPortal().close();
////                                }
////                                )
//                        ),
//                        new StartShooterCommand(robot.getShooter(), true).withTimeout(1800)
//                ),
//
////                new ParallelCommandGroup(
////                    new InstantCommand(() -> {
////                        robot.getVision().getVisionPortal().close();
////                    }
////                    ),
////                        new StartShooterCommand(robot.getShooter(), true).withTimeout(2500)
////                ),
//                new ParallelRaceGroup(
//                        new RapidFireShoot(robot.getFeeder(), robot.getShooter(), robot.getTransfer(), true),
//                        new IntakeCommand(robot.getIntake(), robot.getTransfer())
//                )
//        );
//    }
//
//    private Command scoreWall(boolean first) {
////        Path goForward = new Path(new BezierLine(
////                scoring,
////                beforeForward
////        ));
////        goForward.setLinearHeadingInterpolation(scoringHeading, beforeForwardHeading);
////
////        Path toWallZone = new Path(new BezierLine(
////                beforeForward,
////                wallIntake
////        ));
////        toWallZone.setLinearHeadingInterpolation(beforeForwardHeading, wallIntakeHeading);
//
//        Path toWallZone = new Path(new BezierLine(
//                first ? scoringInitial : scoring,
//                wallIntake
//        ));
//        toWallZone.setLinearHeadingInterpolation(beforeForwardHeading, wallIntakeHeading, 0.5);
//        toWallZone.setPathEndTimeoutConstraint(50);
//
//        Path intakeWall = new Path(new BezierLine(
//                wallIntake,
//                wallIntakeSlow
//        ));
//        intakeWall.setConstantHeadingInterpolation(wallIntakeHeading);
//
//        Path goBack = new Path(new BezierLine(
//                wallIntakeSlow,
//                wallIntakeBack
//        ));
//        goBack.setConstantHeadingInterpolation(wallIntakeHeading);
//        goBack.setPathEndTimeoutConstraint(50);
//        goBack.setZeroPowerAccelerationMultiplier(4.5);
//
//        Path secondIntakeWall = new Path(new BezierLine(
//                wallIntakeBack,
//                wallIntakeSlow
//        ));
//        secondIntakeWall.setConstantHeadingInterpolation(wallIntakeHeading);
//        secondIntakeWall.setPathEndTimeoutConstraint(50);
//        secondIntakeWall.setZeroPowerAccelerationMultiplier(4.5);
//
//        PathChain WallIntakePath = new PathChain(
//            toWallZone,
//            intakeWall,
//            goBack,
//            secondIntakeWall
//        );
//
//        WallIntakePath.setCallbacks(
//            new PathCallback(
//                0, () -> {follower.setMaxPower(MEDIUM_SPEED);}, PathCallback.PARAMETRIC, 0),
//            new PathCallback(
//            0, () -> {follower.setMaxPower(SLOW);}, PathCallback.PARAMETRIC, 1),
//            new PathCallback(
//                0, () -> {follower.setMaxPower(MEDIUM_SPEED);}, PathCallback.PARAMETRIC, 2)
//        );
//
//        Path shootLoadingZone = new Path(new BezierLine(
//                wallIntakeSlow,
//                scoring
//        ));
//        shootLoadingZone.setLinearHeadingInterpolation(wallIntakeHeading, scoringHeading);
//
//        return new SequentialCommandGroup(
//                new InstantCommand(robot.getFeeder()::closePurpleFeeder),
//                new ParallelCommandGroup(
//                        // intaking balls in loading zone
//                            // always turn intake on. Once we're done intaking, reverse it to get rid of extra balls
//                        new IntakeCommand(robot.getIntake(), robot.getTransfer()),
//                        new SequentialCommandGroup(
//                                new InstantCommand(() -> {
//                                    follower.setMaxPower(MEDIUM_SPEED);
//                                }),
//                                new FollowPathCommand(follower, WallIntakePath, true).withTimeout(3100),
//                                new ConditionalCommand(
//                                    new InstantCommand(),
//                                    retryIntake().withTimeout(1500),
//                                    robot.getIntake()::hasAnyBallOnTransfer
//                                ),
////                                new FollowPathCommand(follower, intakeWall),
//////                                new InstantCommand(() -> {
//////                                    follower.setMaxPower(FULL_SPEED);
//////                                }),
////
////                                new FollowPathCommand(follower, goBack),
//////                                new InstantCommand(() -> {
//////                                    follower.setMaxPower(SLOW);
//////                                }),
////                                new FollowPathCommand(follower, secondIntakeWall),
//                                new InstantCommand(() -> {
//                                    follower.setMaxPower(FULL_SPEED);
//                                }),
//
//                                new WaitCommand(200), //to prevent 1/50 chance of ball accidentally shooting
//                                // drive back to starting position to shoot
//
//                                new ParallelCommandGroup(
//                                        new ParallelCommandGroup(
//                                                new FollowPathCommand(follower, shootLoadingZone),
//                                                new StartShooterCommand(robot.getShooter(), true).withTimeout(2000)
//                                        ),
//                                        new SequentialCommandGroup(
//                                                new WaitCommand(400),
//                                                new InstantCommand(robot.getFeeder()::feedPurpleArtifact)
////                                                new ReversePurpleCommand(robot.getTransfer(), 500)
//                                        )
//                                ),
//                            new InstantCommand(robot.getIntake()::stopIntake)
//                        )
//                ),
//
//                // shoot the balls according to the motif pattern
//                new ParallelRaceGroup(
//                        new RapidFireShoot(robot.getFeeder(), robot.getShooter(), robot.getTransfer(), true),
//                        new SlowIntakeCommand(robot.getIntake())
//                )
//        );
//    }
//
//    private Command retryIntake() {
//        Pose currentPose = follower.getPose();
//        Pose backOut = new Pose(currentPose.getX(), currentPose.getY() + 4, intakeHeading);
//        Path backOutPath = new Path(new BezierLine(
//            wallIntakeSlow,
//            wallIntakeBack
//        ));
//        backOutPath.setConstantHeadingInterpolation(intakeHeading);
//
//        Path toWallZone = new Path(new BezierLine(
//            wallIntakeBack,
//            wallIntakeSlow
//        ));
//        toWallZone.setConstantHeadingInterpolation(intakeHeading);
//
//        PathChain retryPath = new PathChain(
//            backOutPath,
//            toWallZone
//        );
//
//        return new FollowPathCommand(
//            follower,
//            retryPath
//        );
//    }
//
//    private Command park() {
//        Path toParking = new Path(new BezierLine(
//                scoring,
//                parking
//        ));
//        toParking.setLinearHeadingInterpolation(scoringHeading, parkingHeading);
//
//        return new SequentialCommandGroup(
//                new FollowPathCommand(
//                        follower,
//                        toParking
//                )
//        );
//    }
//}
