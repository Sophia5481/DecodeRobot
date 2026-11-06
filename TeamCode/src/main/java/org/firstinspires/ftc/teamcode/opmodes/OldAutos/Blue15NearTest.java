package org.firstinspires.ftc.teamcode.opmodes.OldAutos;

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
@Autonomous(name = "Blue Near 15 Ball TEST", group = "Auto")
public class Blue15NearTest extends CommandOpMode {
    private Follower follower;
    private Robot robot;

    private static double FULL_SPEED = 0.9;
    private static double MEDIUM_SPEED = 0.75;
    private static double SLOW = 0.4;

//    private int AprilTagID = 21;
    //FTC -> Pedro: X = 72+X, Y = 72-Y
    //Pedro -> FTC: X = 72-Y, Y = X-72

    //Intake1 path: from scoring 8.64, 59.37 with control point of 51.83, 50.40 and 24.16, 64 constant 180
    //Intake1 to gate: -> 15.27, 69.22 with control point of 33.70, 62.55, constant 180

    public static int directionNumber = -1;

    public double startHeading = Math.toRadians(-52.0 * directionNumber);
    public static Point start = new Point(-58.06, 46.05 * directionNumber, Point.CARTESIAN);
    public static Point end = new Point(-24.5, 32.0 * directionNumber, Point.CARTESIAN);
    public double endHeading = Math.toRadians(180.0 + 90.0 * directionNumber);

    public static Point scoring = new Point(-24.5, 12.0 * directionNumber, Point.CARTESIAN);
    public static Point cycleScoring = new Point(-24.5, 12.0 * directionNumber, Point.CARTESIAN);
    public double scoringHeading = Math.toRadians(-43.0 * directionNumber);
    public double cycleScoringHeading = Math.toRadians(-45.0 * directionNumber);
    public double intakeHeading = Math.toRadians(180.0 - 90.0 * directionNumber);

    public static Point intake1 = new Point(9.63, -52.8, Point.CARTESIAN);
    public static Point intake1ControlPoint1 = new Point(18.6, -24.21, Point.CARTESIAN); //47.79, 53.97
    public static Point intake1ControlPoint2 = new Point(5,-39.87, Point.CARTESIAN);
    public static Point intake2 = new Point(-20.0, 16.0 * directionNumber, Point.CARTESIAN);
    public static Point intake3 = new Point(34.53, 56.27 * directionNumber, Point.CARTESIAN); //12.28, 37.3
    public static Point intake3ControlPoint1 = new Point(47.08, -17.27, Point.CARTESIAN); //51.28, 24.92
    public static Point intake3ControlPoint2 = new Point(38.03,-33.42, Point.CARTESIAN);//31.90, 33.97

    public static Point gatePoint = new Point(-1 * (20.0 - 24.0 - 1.0 + 7.0), directionNumber * (5.0 + 31.5), Point.CARTESIAN);
    public static Point gatePoint2 = new Point(-1 * (20.0 - 24.0 - 1.0 + 10.0), directionNumber * (22.0 + 31.5 - 6.0), Point.CARTESIAN);
    public static Point scoringControlPoint = new Point(-1 * (20.0 - 24.0 - 1.0), directionNumber * 18.0, Point.CARTESIAN);
    public double gateOpeningHeading = -20.0 * directionNumber;
    public static Point gateIntakeControlPoint = new Point(-12.5, 12.0 * directionNumber, Point.CARTESIAN);
    public static Point openGateForIntake = new Point(0, 47.5 * directionNumber, Point.CARTESIAN);
    public static Point gateIntake = new Point(8, 47.5 * directionNumber, Point.CARTESIAN);
    public static double gateIntakeHeading = Math.toRadians(-90 + 20.0 * directionNumber);

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
                        intakeAndScore(2),
                        intakeAndScore(3),
                        gateCycle(),
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
                new InstantCommand(robot.getIntake()::openLatch),
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
                                new InstantCommand(robot.getIntake()::closeLatch)
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

    private Command intakeAndScore(int cycle) {
        Point intakePosition = intake1;
        switch (cycle) {
            case 1:
                intakePosition = intake1;
                break;
            case 2:
                intakePosition = intake2;
                break;
            case 3:
                intakePosition = intake3;
                break;

        }
        Path toIntake;
        if(cycle == 1){
            toIntake = new Path(new BezierCurve(
                    scoring, intake1ControlPoint1, intake1ControlPoint2,
                    intakePosition)
            );
            toIntake.setConstantHeadingInterpolation(intakeHeading);
        }else if (cycle == 3){
            toIntake = new Path(new BezierCurve(
                    cycleScoring, intake3ControlPoint1, intake3ControlPoint2, intakePosition
            ));
            toIntake.setConstantHeadingInterpolation(intakeHeading);
        }else{
            toIntake = new Path(new BezierLine(
                    cycleScoring,
                    intakePosition)
            );
            toIntake.setConstantHeadingInterpolation(intakeHeading);
        }

        // drive forward to intake balls
        Path intakeBalls;
        if(cycle==2){
            intakeBalls = new Path(new BezierLine(
                    intakePosition,
                    incrementY(intakePosition,-29)
            ));
            intakeBalls.setConstantHeadingInterpolation(intakeHeading);
        }else{
            intakeBalls = null;
        }

        Path toScoring;
        // drive from intake position to scoring position
        if (cycle == 1) {
            // special path to avoid hitting the gate while driving
            toScoring = new Path(new BezierCurve(
                    gatePoint2,
                    scoringControlPoint,
                    cycleScoring)
            );
            toScoring.setLinearHeadingInterpolation(intakeHeading, scoringHeading);
        }else if (cycle == 3){
            toScoring = new Path(new BezierLine(
                    intake3,
                    cycleScoring)
            );
            toScoring.setLinearHeadingInterpolation(intakeHeading, cycleScoringHeading);
        }else{
            toScoring = new Path(new BezierLine(intake2, cycleScoring));
            toScoring.setLinearHeadingInterpolation(intakeHeading, cycleScoringHeading);
        }
        //openGate

//        Path toGate = new Path(new BezierCurve(
//                incrementY(intakePosition, cycle == 1 || cycle == 3 ? 31.5 : 29),
//                gatePoint,
//                gatePoint2
//        ));
//        toGate.setConstantHeadingInterpolation(intakeHeading);

        Path openGate = new Path(new BezierCurve(
                intake1,
                gatePoint,
                gatePoint2
        ));
        openGate.setConstantHeadingInterpolation(intakeHeading);

        openGate.setZeroPowerAccelerationMultiplier(4.5);

        PathChain gatePath = new PathChain(openGate);
        gatePath.setCallbacks(new PathCallback(
                0.9, () -> {robot.getIntake().stopIntake();}, PathCallback.PARAMETRIC, 0
        ));
        if(!(cycle == 2)){
            return new SequentialCommandGroup(
                    // drive to intaking
                    // close purple feeder before intaking so we have the hardware limit
                    new InstantCommand(robot.getIntake()::closeLatch),
                    new InstantCommand(() -> {
                        follower.setMaxPower(MEDIUM_SPEED);
                    }),
                    new ParallelRaceGroup(
//                new StopShootingCommand(robot.getShooter()),
                            new IdleShooterCommand(robot.getShooter(), false),
                            new IntakeCommand(robot.getIntake(), false, false),
                            new FollowPathCommand(follower, toIntake)
                    ),
                    // intake balls, drive to scoring
                    new ParallelCommandGroup(
                                    new ConditionalCommand(
                                            new SequentialCommandGroup(
                                                    new InstantCommand(() -> {
                                                        follower.setMaxPower(FULL_SPEED);
                                                    }),
                                                    new WaitCommand(varWaitTime),
                                                    new InstantCommand(() -> Log.d("autonomous", "entered!")),
                                                    new FollowPathCommand(follower, gatePath).withTimeout(3000)
//                                                new WaitCommand(100)
                                            ),
                                            new InstantCommand(() -> {
                                            }),
                                            () -> cycle == 1
                                    ),
//                                new WaitCommand(1500),
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
                    ),
                    new InstantCommand(() -> {
                        Log.d("Scoring Position: ", "X: " + follower.getPose().getX() + "Y: " + follower.getPose().getY() + "Heading: " + Math.toDegrees(follower.getPose().getHeading()));
                    }),
                    new ParallelRaceGroup(
                            new RapidFireShoot(robot.getIntake(), robot.getShooter(), false),
                            new IntakeCommand(robot.getIntake(), false, false)
                    )
            );
        }else{
            return new SequentialCommandGroup(
                    // drive to intaking
                    // close purple feeder before intaking so we have the hardware limit
                    new InstantCommand(robot.getIntake()::closeLatch),
                    new InstantCommand(() -> {
                        follower.setMaxPower(FULL_SPEED);
                    }),
                    new ParallelRaceGroup(
//                new StopShootingCommand(robot.getShooter()),
                            new IdleShooterCommand(robot.getShooter(), false),
                            new FollowPathCommand(follower, toIntake)
                    ),
                    // intake balls, drive to scoring
                    new ParallelCommandGroup(
                            //new IntakeCommand(robot.getIntake(), robot.getTransfer()),
                            new SequentialCommandGroup(
                                    new InstantCommand(() -> {
                                        follower.setMaxPower(MEDIUM_SPEED);
                                    }),
                                    new ParallelRaceGroup(
                                            new IntakeCommand(robot.getIntake(), false, false),
                                            new FollowPathCommand(follower, intakeBalls).withTimeout(2000)
                                    ),

                                    new ConditionalCommand(
                                            new SequentialCommandGroup(
                                                    new InstantCommand(() -> {
                                                        follower.setMaxPower(MEDIUM_SPEED);
                                                    }),
                                                    new WaitCommand(varWaitTime),
                                                    new InstantCommand(() -> Log.d("autonomous", "entered!"))
//                                                new WaitCommand(100)
                                            ),
                                            new InstantCommand(() -> {
                                            }),
                                            () -> cycle == 1
                                    ),
//                                new WaitCommand(1500),
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