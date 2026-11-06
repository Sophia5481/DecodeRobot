package org.firstinspires.ftc.teamcode.opmodes;

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
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.commands.IdleShooterCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.Memory;
import org.firstinspires.ftc.teamcode.commands.RapidFireShoot;
import org.firstinspires.ftc.teamcode.commands.StartShooterCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathCallback;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.AimUtil;

import java.util.concurrent.TimeUnit;


public abstract class Auto21BallNearBase extends CommandOpMode {
    protected Follower follower;
    protected Robot robot;

    protected static final double FULL_SPEED = 1.0; //0.95
    protected static final double MEDIUM_SPEED = 0.75;
    protected static final double SLOW = 0.7; //try decreasing this more if it isn't accurate

    // Alliance-specific fields (to be set in derived classes)
    protected Point start;
    protected Point end;
    protected double startHeading;
//    protected double endHeading;

    // The initial scoring position is adjusted for drift
    protected Point scoring;
    protected double straightScoringHeading;
    protected double scoringHeadingDiagonal;
    // scoring point adjusted for drift after intaking
//    protected Point cycleScoring;
//    protected double scoringHeading;
//    protected double cycleScoringHeading;

    protected double intakeHeading;
    // intake points
    protected Point intake1;
    protected Point intake1Control;
    protected Point intake2;
    protected Point intake2Control;
    protected Point gateOpen;
    protected Point intake3;
    protected Point intake3Control;
    // open gate points
    protected Point scoringControlPoint;
    protected Point gateIntake;
    protected Point gateIntakeControlPoint;
    protected double interceptingGateHeading;
    protected double gateIntakeHeading;

    protected double gateOpeningHeading;
    protected double scoringGateHeading;
    protected Point wallGateControl;
    protected Point newGateIntake;

    protected Timing.Timer timer = new Timing.Timer(30, TimeUnit.SECONDS);

    public static long varWaitTime = 50;

    protected int directionNumber(){
        if (isRedAlliance()){
            return 1;
        }
        return -1;
    }
    protected abstract boolean isRedAlliance();
    protected abstract Command scheduleSequence();
    private void setupAllianceSpecifics(){
        ///FTC(X) -> 72-PY
        ///FTC (y) -> P(x) -72
        startHeading = Math.toRadians(128.8 * directionNumber());
        start = new Point(-47.393, (49.5968) * directionNumber(), Point.CARTESIAN);
        end = new Point(-5.5, 25.0 * directionNumber(), Point.CARTESIAN);
//        endHeading = Math.toRadians(180.0 + 90.0 * directionNumber());

        scoring = new Point(-5.5, 9.5 * directionNumber(), Point.CARTESIAN); //try moving it closer to the origin if possible
//        cycleScoring = new Point(-18.5, 15.0 * directionNumber(), Point.CARTESIAN);
        straightScoringHeading = Math.toRadians(90 * directionNumber());
        //TODO: check if this is true: Pedropathing visualizer states that the optimal angle for movement while gate intaking is 74 (FTC coordinate)
        scoringGateHeading = Math.toRadians(74.0 * directionNumber()); //70.5
        scoringHeadingDiagonal = Math.toRadians(60.0 * directionNumber()); //only done for intake1
//        cycleScoringHeading = Math.toRadians(-45.0 * directionNumber());
        intakeHeading = Math.toRadians(90.0 * directionNumber());

//        intake1 = new Point(-1 * (-4.0), 15.0 * directionNumber(), Point.CARTESIAN);
        intake1 = new Point(5.5, 65.0 * directionNumber(), Point.CARTESIAN);
        intake1Control = new Point(18.5, 8.0 * directionNumber(), Point.CARTESIAN);
        // opening the gate after intaking the 2nd row of balls but not gate intaking
        gateOpen = new Point(9.0, 51.0 * directionNumber(), Point.CARTESIAN);
        intake2 = new Point(-12.0, 53.5 * directionNumber(), Point.CARTESIAN);
        intake2Control = new Point (-13.5, 15.0 * directionNumber(), Point.CARTESIAN);
//        intake3 = new Point( -1 * (-27), 17.0 * directionNumber(), Point.CARTESIAN);
        intake3 = new Point (34.0, 54.5 * directionNumber(), Point.CARTESIAN);
        intake3Control = new Point(39.5, 4.0, Point.CARTESIAN);

        // gate intake points
        scoringControlPoint = new Point(-1 * (20.0 - 24.0 - 1.0), directionNumber() * 18.0, Point.CARTESIAN);
        gateIntake = new Point(12.5, 60.7 * directionNumber(), Point.CARTESIAN);
        gateIntakeControlPoint = new Point(13.5, 25 * directionNumber(), Point.CARTESIAN);
        gateIntakeHeading = Math.toRadians(119.0 * directionNumber()); //120 seems to be better on manual drive, will be tested

        //new gate intake method points (for testing)
        newGateIntake = new Point(20.5, 60.5 * directionNumber(), Point.CARTESIAN);;
        interceptingGateHeading = Math.toRadians(135.0 * directionNumber());
        wallGateControl = new Point(18.5, 55.5 * directionNumber(), Point.CARTESIAN);
    }

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

        FollowerConstants.pathEndTimeoutConstraint = 50;

        waitForStart();
        timer.start();

        schedule(
                scheduleSequence()
        );
    }

    @Override
    public void run() {
        super.run();

        follower.update();
        follower.drawOnDashBoard();
        robot.resetCache();
        Pose robotPos = follower.getPose();
        Log.d("Follower ", "Heading value: " + Math.toDegrees(follower.getTotalHeading()));
        Log.d("Follower ", "Pose: " + robotPos.toString());
        // start tracking end pose for correct field centric drive
        if (timer.elapsedTime() > 26 && timer.elapsedTime() < 29.9) {
            Memory.autoEndPose = robotPos;
            Log.d("Auto End Pose: ", "X: " + Memory.autoEndPose.getX() + "Y: " + Memory.autoEndPose.getY() + "Heading: " + Memory.autoEndPose.getHeading());
        }

        double goalX = RED_ALLIANCE ? RED_GOAL_X : BLUE_GOAL_X;
        double goalY = RED_ALLIANCE ? RED_GOAL_Y : BLUE_GOAL_Y;
        //aiming
        AimUtil.AimResult aim = AimUtil.getAngleToGoal(robotPos, goalX, goalY);
        double targetTurretAngle = Turret.turretAngleToServoDegrees(
            AimUtil.getTurretAngle(aim.angleDeg, robotPos.getHeading()));
        if (targetTurretAngle >= TURRET_MIN_ANGLE && targetTurretAngle <= TURRET_MAX_ANGLE) {
            robot.getTurret().setTurretAngle(targetTurretAngle);
        } else {
            telemetry.addData("WARNING: ", "Target out of turret range!");
        }
    }

    protected Command scorePreload() {
        Path toScoring = new Path(new BezierLine(
                start,
                scoring)
        );
        //maintaining start heading seems to be better
        //Plan: maintain start heading for preload path, and then for the next path, we snap to scoringHeadingDiagonal. This should allow it to be faster.
        toScoring.setConstantHeadingInterpolation(startHeading);
        toScoring.setZeroPowerAccelerationMultiplier(3.0);
        PathChain scoringPath = new PathChain(toScoring);
        scoringPath.setCallbacks(
                new PathCallback(0, () -> follower.setMaxPower(FULL_SPEED), PathCallback.PARAMETRIC, 0),
                new PathCallback(0.8, () -> follower.setMaxPower(SLOW), PathCallback.PARAMETRIC, 0) //see if this can be increased
        );
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, scoringPath, true).withTimeout(2000),
                        new SequentialCommandGroup(
                                new InstantCommand(() -> {
                                    robot.getIntake().setHasThreeBalls(false);
                                }),
                                new StartShooterCommand(robot.getShooter(), false),
                                new WaitUntilCommand(() -> follower.getCurrentTValue() > 0.80),
                                new RapidFireShoot(robot.getIntake(), robot.getShooter(), false),
                                new InstantCommand(() -> {
                                    telemetry.addData("scoring position: ", "X: " + follower.getPose().getX() + "Y: " + follower.getPose().getY() + "Heading: " + Math.toDegrees(follower.getPose().getHeading()));
                                    Log.d("Scoring Position: ", "X: " + follower.getPose().getX() + "Y: " + follower.getPose().getY() + "Heading: " + Math.toDegrees(follower.getPose().getHeading()));
                                })
                        )
                )
        );
    }
//                new FollowPathCommand(follower, toScoring).withTimeout(1800)
//                        .deadlineWith(
//                                new StartShooterCommand(robot.getShooter(), false)
//                        ),
//                new InstantCommand(() -> {
//                    telemetry.addData("scoring position: ", "X: " + follower.getPose().getX() + "Y: " + follower.getPose().getY() + "Heading: " + Math.toDegrees(follower.getPose().getHeading()));
//                    Log.d("Scoring Position: ", "X: " + follower.getPose().getX() + "Y: " + follower.getPose().getY() + "Heading: " + Math.toDegrees(follower.getPose().getHeading()));
//                }),
//                new WaitUntilCommand(() -> follower.getCurrentTValue() > 0.8),
//                new RapidFireShoot(robot.getIntake(), robot.getShooter(), false)


    protected Command intakeAndScore(int cycle) {
        Point intakePosition = intake1;
        Point controlPoint = intake1Control;
        switch (cycle) {
            case 1:
                intakePosition = intake1;
                controlPoint = intake1Control;
                break;
            case 2:
                intakePosition = intake2;
                controlPoint = intake2Control;
                break;
            case 3:
                intakePosition = intake3;
                controlPoint = intake3Control;
                break;
        }

        // drive from scoringPath to intake position
        Path toIntake;
        if (cycle != 2) {
            toIntake = new Path(new BezierCurve(
                    scoring,
                    controlPoint,
                    intakePosition)
            );
            toIntake.setLinearHeadingInterpolation(scoringHeadingDiagonal, intakeHeading, 0.45);
            toIntake.setZeroPowerAccelerationMultiplier(4.5); //try increasing zero power acceleration if possible
        }else{
            toIntake = new Path(new BezierCurve(
                    scoring, controlPoint, intakePosition
            ));
            toIntake.setConstantHeadingInterpolation(intakeHeading);
            toIntake.setZeroPowerAccelerationMultiplier(4.5);//try increasing zero power acceleration if possible
        }

        // if intaking the 2nd row of balls, open gate after that (but not gate intake)
        //No longer needed, first cycle already opens gate
//        Path gateOpen;
//        gateOpen = new Path(new BezierLine(
//            intake1,
//                this.gateOpen
//        ));
//        gateOpen.setConstantHeadingInterpolation(intakeHeading);

        Path toScoring;
        // drive from intake position to scoringPath position
        //TODO: see if line is feasible or not
        if (cycle == 1) {
            // special path to avoid hitting the gate while driving
            toScoring = new Path(new BezierCurve(
                    intakePosition,//this.gateOpen (is wrong, we never go to gateOpen anymore)
//                    scoringControlPoint,
                    scoring)
            );

            toScoring.setLinearHeadingInterpolation(intakeHeading, scoringGateHeading, 0.8); ///done for speed TODO: tune as needed
            toScoring.setZeroPowerAccelerationMultiplier(3.0);
        }else{
            toScoring = new Path(new BezierLine(
                    intakePosition,
                    scoring)
            );
            toScoring.setConstantHeadingInterpolation(intakeHeading);
            toScoring.setZeroPowerAccelerationMultiplier(3.0); ///switch back if drifting happens
        }
        toScoring.setPathEndTimeoutConstraint(300);

        //prevents drifting
        PathChain scoringPath = new PathChain(toScoring);
        scoringPath.setCallbacks(
                new PathCallback(0, () -> follower.setMaxPower(FULL_SPEED), PathCallback.PARAMETRIC, 0),
                new PathCallback(0.525, () -> follower.setMaxPower(SLOW), PathCallback.PARAMETRIC, 0)
        );
        return new SequentialCommandGroup(
                // drive to intaking
                new InstantCommand(()->{
                    follower.setMaxPower(FULL_SPEED);
                    robot.getIntake().setHasThreeBalls(false);
                }),
                new FollowPathCommand(follower, toIntake).withTimeout(cycle == 3 ? 2200 : cycle == 2 ? 1200 : 1800)
                        .deadlineWith(
                                new IdleShooterCommand(robot.getShooter(), false),
                                new InstantCommand(()->{robot.getIntake().fullIntake();})
                        ),
                new InstantCommand(robot.getIntake()::stopIntake),
                new InstantCommand(()->{follower.setMaxPower(FULL_SPEED);}),
                // after intaking 2nd row of balls, open the gate
                //cycle == 1 ? new FollowPathCommand(follower, gateOpen).withTimeout(1000) : new InstantCommand(),

                new ParallelCommandGroup(
                        new FollowPathCommand(follower, scoringPath, true),
                        new SequentialCommandGroup(
                                new InstantCommand(() -> {
                                    robot.getIntake().setHasThreeBalls(false);
                                }),
                                new StartShooterCommand(robot.getShooter(), false)
                                        .deadlineWith(new IntakeCommand(robot.getIntake(), false, false, () -> true)),
                                new WaitUntilCommand(() -> follower.getCurrentTValue() > 0.9),//increase for accuratcyu
                                new RapidFireShoot(robot.getIntake(), robot.getShooter(), false)
                        )
                )
//                new FollowPathCommand(follower, toScoring,true).withTimeout(1800)
//                        .deadlineWith(
//                                new ParallelCommandGroup(
//                                        new IntakeCommand(robot.getIntake(), false, false, ()->true),
//                                        new SequentialCommandGroup(
//                                                new StartShooterCommand(robot.getShooter(), false),
//                                                new InstantCommand(() -> {
//                                                    robot.getIntake().setHasThreeBalls(false);
//                                                })
//                                        )
//                                )
//
//                        ),
//                new RapidFireShoot(robot.getIntake(), robot.getShooter(), false)
//                new SequentialCommandGroup(
//                        new InstantCommand(() -> {
//                            follower.setMaxPower(FULL_SPEED);
//                            Log.d("AutoCheckpoint", "Debugging");
//                        }),
//                        new FollowPathCommand(follower, toScoring).withTimeout(3000).deadlineWith(
//                                new StartShooterCommand(robot.getShooter(), false),
//                                new IntakeCommand(robot.getIntake(), false, false, () -> true)
//                        )
//                ),
//                new InstantCommand(() -> Log.d("AutoCheckpoint", "Debugging")),
//                new InstantCommand(() -> {
//                    telemetry.addData("scoringPath position: ", "X: " + follower.getPose().getX() + "Y: " + follower.getPose().getY() + "Heading: " + Math.toDegrees(follower.getPose().getHeading()));
//                    Log.d("Scoring Position: ", "X: " + follower.getPose().getX() + "Y: " + follower.getPose().getY() + "Heading: " + Math.toDegrees(follower.getPose().getHeading()));
//                }),
////                new InstantCommand(()->{robot.getIntake().deployRest();}),
//                new RapidFireShoot(robot.getIntake(), robot.getShooter(), false),
//                new InstantCommand(() -> Log.d("AutoCheckpoint", "Debugging"))
//        );
        );
    }
//    public Command interceptingGateCycle(){
//        Path openGate = new Path(new BezierLine(
//                scoring,
//                gateOpen
//        ));
//        openGate.setLinearHeadingInterpolation(scoringHeadingDiagonal, intakeHeading);
//        openGate.setZeroPowerAccelerationMultiplier(4.5);
//
//        Path gateIntake = new Path(new BezierLine(
//                gateOpen,
//                newGateIntake
//        ));
//        gateIntake.setLinearHeadingInterpolation(intakeHeading, interceptingGateHeading);
//        gateIntake.setZeroPowerAccelerationMultiplier(4.5);
//
//        Path toScoring = new Path(new BezierLine(
//                newGateIntake,
//                scoring
//        ));
//        toScoring.setConstantHeadingInterpolation(scoringHeadingDiagonal);
//        toScoring.setZeroPowerAccelerationMultiplier(4.5);
//
//        return new SequentialCommandGroup(
//                new InstantCommand(()->{follower.setMaxPower(FULL_SPEED);}),
//                //open gate path
//                new FollowPathCommand(follower, openGate).withTimeout(1000),
//                new IntakeCommand(robot.getIntake(), false, false, () -> true).withTimeout(2000)
//                        .deadlineWith(new FollowPathCommand(follower, gateIntake)),
//                //prevent over possession
//                new InstantCommand(()->{robot.getIntake().stopIntake();}),
//
//                new SequentialCommandGroup(
//                        new InstantCommand(() -> {
//                            follower.setMaxPower(FULL_SPEED);
//                        }),
//                        new FollowPathCommand(follower, toScoring).withTimeout(2000)
//                                .deadlineWith(new StartShooterCommand(robot.getShooter(), false))
//                ),
//                new InstantCommand(() -> {
//                    telemetry.addData("scoring position: ", "X: " + follower.getPose().getX() + "Y: " + follower.getPose().getY() + "Heading: " + Math.toDegrees(follower.getPose().getHeading()));
//                    Log.d("Scoring Position: ", "X: " + follower.getPose().getX() + "Y: " + follower.getPose().getY() + "Heading: " + Math.toDegrees(follower.getPose().getHeading()));
//                }),
////                new ParallelCommandGroup(
////                        new FollowPathCommand(follower, toScoring).withTimeout(),
////                        new SequentialCommandGroup(
////                                new InstantCommand(() -> {
////                                    robot.getIntake().setHasThreeBalls(false);
////                                }),
////                                new StartShooterCommand(robot.getShooter(), false),
////                                new WaitUntilCommand(() -> follower.getCurrentTValue() > 0.8),
////                                new RapidFireShoot(robot.getIntake(), robot.getShooter(), false)
////                        )
////                )
//                new RapidFireShoot(robot.getIntake(), robot.getShooter(), false)
//        );
//    }
//    public Command wallGateCycle(){
//        Path openGate = new Path(new BezierLine(
//                scoring,
//                gateOpen
//        ));
//        openGate.setLinearHeadingInterpolation(scoringHeadingDiagonal, intakeHeading);
//        openGate.setZeroPowerAccelerationMultiplier(4.5);
//
//        Path wallIntake = new Path(new BezierCurve(
//                gateOpen,
//                wallGateControl,
//                newGateIntake
//        ));
//        wallIntake.setConstantHeadingInterpolation(intakeHeading);
//        wallIntake.setZeroPowerAccelerationMultiplier(4.5);
//
//        Path toScoring = new Path(new BezierLine(
//                newGateIntake, scoring
//        ));
//        toScoring.setLinearHeadingInterpolation(intakeHeading, scoringHeadingDiagonal);
//        toScoring.setZeroPowerAccelerationMultiplier(4.5);
//
//        return new SequentialCommandGroup(
//                new InstantCommand(()->{follower.setMaxPower(FULL_SPEED);}),
//                //open gate path
//                new FollowPathCommand(follower, openGate).withTimeout(1000),
//                new IntakeCommand(robot.getIntake(), false, false, () -> true).withTimeout(2000)
//                        .deadlineWith(new FollowPathCommand(follower, wallIntake)),
//                //prevent over possession
//                new InstantCommand(()->{robot.getIntake().stopIntake();}),
//
//                new SequentialCommandGroup(
//                        new InstantCommand(() -> {
//                            follower.setMaxPower(FULL_SPEED);
//                        }),
//                        new FollowPathCommand(follower, toScoring).withTimeout(2000)
//                                .deadlineWith(new StartShooterCommand(robot.getShooter(), false))
//                ),
//                new InstantCommand(() -> {
//                    telemetry.addData("scoring position: ", "X: " + follower.getPose().getX() + "Y: " + follower.getPose().getY() + "Heading: " + Math.toDegrees(follower.getPose().getHeading()));
//                    Log.d("Scoring Position: ", "X: " + follower.getPose().getX() + "Y: " + follower.getPose().getY() + "Heading: " + Math.toDegrees(follower.getPose().getHeading()));
//                }),
////                new ParallelCommandGroup(
////                        new FollowPathCommand(follower, toScoring).withTimeout(),
////                        new SequentialCommandGroup(
////                                new InstantCommand(() -> {
////                                    robot.getIntake().setHasThreeBalls(false);
////                                }),
////                                new StartShooterCommand(robot.getShooter(), false),
////                                new WaitUntilCommand(() -> follower.getCurrentTValue() > 0.8),
////                                new RapidFireShoot(robot.getIntake(), robot.getShooter(), false)
////                        )
////                )
//                new RapidFireShoot(robot.getIntake(), robot.getShooter(), false)
//        );
//    }

    public Command gateCycle() {
    //TODO: check if making it into a line works or not
        Path toGate = new Path(new BezierCurve(
                scoring,
//                gateIntakeControlPoint,
                gateIntake
        ));
        toGate.setLinearHeadingInterpolation(scoringGateHeading, gateIntakeHeading, 0.95);
        toGate.setZeroPowerAccelerationMultiplier(4.5); //speed

        //straight line added for speed
        Path toScoring = new Path(new BezierCurve(
                gateIntake,
//                gateIntakeControlPoint,
                scoring
        ));
        toScoring.setLinearHeadingInterpolation(gateIntakeHeading, scoringGateHeading, 0.12);////done for speed
        toScoring.setZeroPowerAccelerationMultiplier(3.0); //speed

        PathChain scoringPath = new PathChain(toScoring);
        scoringPath.setCallbacks(
                new PathCallback(0, () -> follower.setMaxPower(FULL_SPEED), PathCallback.PARAMETRIC, 0),
                new PathCallback(0.52, () -> follower.setMaxPower(SLOW), PathCallback.PARAMETRIC, 0)
        );

        return new SequentialCommandGroup(
                new InstantCommand(()->{robot.getIntake().setHasThreeBalls(false);}),
                new InstantCommand(()->{follower.setMaxPower(FULL_SPEED);}),
                new FollowPathCommand(follower, toGate).withTimeout(2000)
                        .deadlineWith(
                                new IntakeCommand(robot.getIntake(), false, false, ()-> true),
                                new InstantCommand(()->{
                            Log.d("GateCyclingPosition", "X: " + follower.getPose().getX() + "Y: " + follower.getPose().getY() + "Heading: " + Math.toDegrees(follower.getPose().getHeading()));
                            telemetry.addData("gatecycling position: ", "X: " + follower.getPose().getX() + "Y: " + follower.getPose().getY() + "Heading: " + Math.toDegrees(follower.getPose().getHeading()));
                        })),
//                new InstantCommand(() -> Log.d("AutoCheckpoint", "Debugging")),
//                new InstantCommand(()-> {
//                    Log.d("AutonomousPeriod", "X: " + follower.getPose().getX());
//                    Log.d("AutonomousPeriod,", "Y: " + follower.getPose().getY());
//                    Log.d("AutonomousPeriod,", "heading: " + follower.getPose().getHeading());
//                }),
                new ParallelRaceGroup(
                        new IntakeCommand(robot.getIntake(), false, false, () -> true),
                        new WaitUntilCommand(robot.getIntake()::hasThreeBalls).withTimeout(900)
                ),
//                new IntakeCommand(robot.getIntake(), false, false, ()-> true).withTimeout(1200),

                // try shooting the balls a little earlier than the robot stops
                new InstantCommand(()->{follower.setMaxPower(FULL_SPEED);}),
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, scoringPath, true).withTimeout(2000),
                        new SequentialCommandGroup(
                                new InstantCommand(() -> {
                                    robot.getIntake().setHasThreeBalls(false);
                                }),
                                new StartShooterCommand(robot.getShooter(), false)
                                        .deadlineWith(new IntakeCommand(robot.getIntake(), false, false)),
                                new WaitUntilCommand(() -> follower.getCurrentTValue() > 0.9),
                                new RapidFireShoot(robot.getIntake(), robot.getShooter(), false)
                        )
                )
//                new FollowPathCommand(follower, toScoring,true).withTimeout(2000)
//                        .deadlineWith(
//                            new ParallelCommandGroup(
//                                    new IntakeCommand(robot.getIntake(), false, false, ()->true),
//                                    new SequentialCommandGroup(
//                                            new StartShooterCommand(robot.getShooter(), false),
//                                            new InstantCommand(() -> {
//                                                robot.getIntake().setHasThreeBalls(false);
//                                            })
//                                    )
//                            )
//
//                        ),
//                new RapidFireShoot(robot.getIntake(), robot.getShooter(), false)
        );
    }
    protected Command park() {
        Path parking = new Path(new BezierLine(
                scoring,
                end
        ));
        parking.setConstantHeadingInterpolation(intakeHeading);

        return new FollowPathCommand(
                follower, parking
        );
    }

    //    protected Point incrementY(Point point, double increment) {
//        return new Point(point.getX(), point.getY() + increment, Point.CARTESIAN);
//    }
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

//        //TODO:fix shooting while moving logic
//        double robotVelocity = Math.abs(follower.getVelocityMagnitude());
//        double additionalTurretAngle = 1.0 * robotVelocity * directionNumber();
//        telemetry.addData("additionalTurretAngle: ", additionalTurretAngle);
//        telemetry.addData("robotVelocity: ", robotVelocity);

        double turretAngleDeg = turretAngle;
        while (turretAngleDeg > 180) turretAngleDeg -= 360;
        while (turretAngleDeg < -180) turretAngleDeg += 360;

        return turretAngleDeg;
    }
}