package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.Robot.RED_ALLIANCE;
import static org.firstinspires.ftc.teamcode.commands.Memory.distanceToGoal;
import static org.firstinspires.ftc.teamcode.opmodes.ManualDrive.BLUE_GOAL_X;
import static org.firstinspires.ftc.teamcode.opmodes.ManualDrive.BLUE_GOAL_Y;
import static org.firstinspires.ftc.teamcode.opmodes.ManualDrive.RED_GOAL_X;
import static org.firstinspires.ftc.teamcode.opmodes.ManualDrive.RED_GOAL_Y;
import static org.firstinspires.ftc.teamcode.subsystems.TurretWithEncoder.MAX_DEGREE;
import static org.firstinspires.ftc.teamcode.subsystems.TurretWithEncoder.MIN_DEGREE;

import android.util.Log;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
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
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.AimUtil;

import java.util.concurrent.TimeUnit;


public abstract class Auto18BallNearBase extends CommandOpMode {
    protected Follower follower;
    protected Robot robot;

    protected static final double FULL_SPEED = 1.0;
    protected static final double MEDIUM_SPEED = 0.95;
    protected static final double SLOW = 0.7;

    // Alliance-specific fields (to be set in derived classes)
    protected Point start;
    protected Point end;
    protected double startHeading;
    protected double endHeading;

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
    protected Point intake1GateOpen;
    protected Point intake3;
    protected Point intake3Control;
    // open gate points
    protected Point scoringControlPoint;
    protected Point gateIntake;
    protected Point gateIntakeControlPoint;
    protected double gateIntakeHeading;

    protected double gateOpeningHeading;
    private double turretFixedAngle;

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
        //FTC(X) -> 72-PY
        //FTC (y) -> P(x) - 72
        startHeading = Math.toRadians(128.8 * directionNumber());
        start = new Point(-47.393, (49.5968) * directionNumber(), Point.CARTESIAN);
        end = new Point(-9.5, 32.0 * directionNumber(), Point.CARTESIAN);
        endHeading = Math.toRadians(180.0 + 90.0 * directionNumber());

        scoring = new Point(-9.5, 14.5 * directionNumber(), Point.CARTESIAN);
//        cycleScoring = new Point(-18.5, 15.0 * directionNumber(), Point.CARTESIAN);
        straightScoringHeading = Math.toRadians(90 * directionNumber());
        scoringHeadingDiagonal = Math.toRadians(65.0 * directionNumber());
//        cycleScoringHeading = Math.toRadians(-45.0 * directionNumbe r());
        intakeHeading = Math.toRadians(180.0 - 90.0 * directionNumber());

//        intake1 = new Point(-1 * (-4.0), 15.0 * directionNumber(), Point.CARTESIAN);
        intake1 = new Point(5.5, 65.0 * directionNumber(), Point.CARTESIAN);
        intake1Control = new Point(18.5, 8.0 * directionNumber(), Point.CARTESIAN);
        // opening the gate after intaking the 2nd row of balls but not gate intaking
        intake1GateOpen = new Point(9.0, 51.0 * directionNumber(), Point.CARTESIAN);
        intake2 = new Point(-12.0, 53.5 * directionNumber(), Point.CARTESIAN);
        intake2Control = new Point (-13.5, 15.0 * directionNumber(), Point.CARTESIAN);
//        intake3 = new Point( -1 * (-27), 17.0 * directionNumber(), Point.CARTESIAN);
        intake3 = new Point (34.0, 54.5 * directionNumber(), Point.CARTESIAN);
        intake3Control = new Point(39.5, 4.0, Point.CARTESIAN);

        // gate intake points
        scoringControlPoint = new Point(-1 * (20.0 - 24.0 - 1.0), directionNumber() * 18.0, Point.CARTESIAN);
        gateIntake = new Point(12.5, 61.2 * directionNumber(), Point.CARTESIAN);
        gateIntakeControlPoint = new Point(13.5, 25 * directionNumber(), Point.CARTESIAN);
//        gateIntakeHeading = Math.toRadians(115.0 * directionNumber());
        gateIntakeHeading = Math.toRadians(119.0 * directionNumber());
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
        Log.d("Follower ", "Heading value: " + Math.toDegrees(follower.getTotalHeading()));
        Log.d("Follower ", "Pose: " + follower.getPose().toString());
        follower.update();
        follower.drawOnDashBoard();
        robot.resetCache();
        Pose robotPos = follower.getPose();
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
        if (targetTurretAngle >= MIN_DEGREE && targetTurretAngle <= MAX_DEGREE) {
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
        toScoring.setConstantHeadingInterpolation(startHeading);

        return new SequentialCommandGroup(
                new InstantCommand(()->{follower.setMaxPower(FULL_SPEED);}),
                new FollowPathCommand(follower, toScoring, true).withTimeout(2000)
                        .deadlineWith(
                                new StartShooterCommand(robot.getShooter(), false),
                                new IntakeCommand(robot.getIntake(), false, false, ()-> true)
                        ),
                new InstantCommand(() -> {
                    telemetry.addData("scoring position: ", "X: " + follower.getPose().getX() + "Y: " + follower.getPose().getY() + "Heading: " + Math.toDegrees(follower.getPose().getHeading()));
                    Log.d("Scoring Position: ", "X: " + follower.getPose().getX() + "Y: " + follower.getPose().getY() + "Heading: " + Math.toDegrees(follower.getPose().getHeading()));
                }),
                new RapidFireShoot(robot.getIntake(), robot.getShooter(), false)
        );
    }

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


        // drive from scoring to intake position
        Path toIntake;

        if (cycle != 2) {
            toIntake = new Path(new BezierCurve(
                    scoring,
                    controlPoint,
                    intakePosition)
            );
            toIntake.setLinearHeadingInterpolation(scoringHeadingDiagonal, intakeHeading, 0.6);
        }else{
            toIntake = new Path(new BezierLine(
                    scoring, intakePosition
            ));
            toIntake.setConstantHeadingInterpolation(intakeHeading);
        }

        // if intaking the 2nd row of balls, open gate after that (but not gate intake)
//        Path gateOpen;
//        gateOpen = new Path(new BezierLine(
//                intake1,
//                intake1GateOpen
//        ));
//        gateOpen.setConstantHeadingInterpolation(intakeHeading);

        Path toScoring;
        // drive from intake position to scoring position
        if (cycle == 1) {
            // special path to avoid hitting the gate while driving
            toScoring = new Path(new BezierCurve(
                    intake1GateOpen,
                    scoringControlPoint,
                    scoring)
            );

            toScoring.setLinearHeadingInterpolation(intakeHeading, scoringHeadingDiagonal, 0.7);
        } else {
            toScoring = new Path(new BezierLine(
                    intakePosition,
                    scoring)
            );
            toScoring.setLinearHeadingInterpolation(intakeHeading, scoringHeadingDiagonal, 0.9);
            toScoring.setZeroPowerAccelerationMultiplier(3.0);
        }

        PathChain scoringPath = new PathChain(toScoring);
        scoringPath.setCallbacks(
                new PathCallback(0, () -> follower.setMaxPower(FULL_SPEED), PathCallback.PARAMETRIC, 0),
                new PathCallback(0.55, () -> follower.setMaxPower(SLOW), PathCallback.PARAMETRIC, 0)
        );

        return new SequentialCommandGroup(
                // drive to intaking
                new InstantCommand(()->{
                    follower.setMaxPower(FULL_SPEED);
                    robot.getIntake().setHasThreeBalls(false);
                }),
                new FollowPathCommand(follower, toIntake).withTimeout(cycle == 3 ? 2200 : cycle == 2 ? 1600 : 2000)
                        .deadlineWith(
                                new IdleShooterCommand(robot.getShooter(), false),
                                new InstantCommand(()->{robot.getIntake().fullIntake();})
                        ),
                new InstantCommand(robot.getIntake()::stopIntake),
                // after intaking 2nd row of balls, open the gate
//                cycle == 1 ? new FollowPathCommand(follower, gateOpen) : new InstantCommand(),
                new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            follower.setMaxPower(MEDIUM_SPEED);
                            Log.d("AutoCheckpoint", "Debugging");
                        }),
                        new FollowPathCommand(follower, scoringPath, true).withTimeout(2000)
                                .deadlineWith(
                                        new IntakeCommand(robot.getIntake(), false, false, ()-> true),
                                        new StartShooterCommand(robot.getShooter(), false)
                                )
                ),
                new InstantCommand(() -> Log.d("AutoCheckpoint", "Debugging")),
                new InstantCommand(() -> {
                    telemetry.addData("scoring position: ", "X: " + follower.getPose().getX() + "Y: " + follower.getPose().getY() + "Heading: " + Math.toDegrees(follower.getPose().getHeading()));
                    Log.d("Scoring Position: ", "X: " + follower.getPose().getX() + "Y: " + follower.getPose().getY() + "Heading: " + Math.toDegrees(follower.getPose().getHeading()));
                }),
//                new InstantCommand(()->{robot.getIntake().deployRest();}),
                new RapidFireShoot(robot.getIntake(), robot.getShooter(), false),
                new InstantCommand(() -> Log.d("AutoCheckpoint", "Debugging"))
        );
    }
    public Command gateCycle() {
        Path toGate = new Path(new BezierCurve(
                scoring,
                gateIntakeControlPoint,
                gateIntake
        ));
        toGate.setLinearHeadingInterpolation(scoringHeadingDiagonal, gateIntakeHeading, 0.9); //end time was 0.9
        toGate.setZeroPowerAccelerationMultiplier(4.5);
        toGate.setPathEndTimeoutConstraint(50);

        Path toScoring = new Path(new BezierCurve(
                gateIntake,
                gateIntakeControlPoint,
                scoring
        ));
        toScoring.setLinearHeadingInterpolation(gateIntakeHeading, scoringHeadingDiagonal, 0.3); //end time was 0.2
        toScoring.setZeroPowerAccelerationMultiplier(3.0);

        PathChain scoringPath = new PathChain(toScoring);
        scoringPath.setCallbacks(
                new PathCallback(0, () -> follower.setMaxPower(FULL_SPEED), PathCallback.PARAMETRIC, 0),
                new PathCallback(0.53, () -> follower.setMaxPower(SLOW), PathCallback.PARAMETRIC, 0)
        );

        return new SequentialCommandGroup(
                new InstantCommand(()->{
                    follower.setMaxPower(FULL_SPEED);
                    robot.getIntake().setHasThreeBalls(false);
                }),
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, toGate).withTimeout(2000),
                        new InstantCommand(()->{
                            Log.d("GateCyclingPosition", "X: " + follower.getPose().getX() + "Y: " + follower.getPose().getY() + "Heading: " + Math.toDegrees(follower.getPose().getHeading()));
                            telemetry.addData("gatecycling position: ", "X: " + follower.getPose().getX() + "Y: " + follower.getPose().getY() + "Heading: " + Math.toDegrees(follower.getPose().getHeading()));
                        })
                ),
                new InstantCommand(() -> Log.d("AutoCheckpoint", "Debugging")),
//                new InstantCommand(()-> {
//                    Log.d("AutonomousPeriod", "X: " + follower.getPose().getX());
//                    Log.d("AutonomousPeriod,", "Y: " + follower.getPose().getY());
//                    Log.d("AutonomousPeriod,", "heading: " + follower.getPose().getHeading());
//                }),
                new ParallelRaceGroup(
                        new IntakeCommand(robot.getIntake(), false, false),
                        new WaitUntilCommand(robot.getIntake()::hasThreeBalls).withTimeout(1200)
                ),

                //artifact detection seems to be failing
//                new IntakeCommand(robot.getIntake(), false, false).withTimeout(1500),

                new FollowPathCommand(follower, scoringPath, true).withTimeout(2000)
                        .deadlineWith(
                                new IntakeCommand(robot.getIntake(), false, false, ()-> true),
                                new StartShooterCommand(robot.getShooter(), false)
                        ),
                new RapidFireShoot(robot.getIntake(), robot.getShooter(), false)
        );
    }
    protected Command park() {
        Path parking = new Path(new BezierLine(
                scoring,
                end
        ));
        parking.setLinearHeadingInterpolation(scoringHeadingDiagonal, intakeHeading);
        parking.setZeroPowerAccelerationMultiplier(4.5);

        return new FollowPathCommand(
                follower, parking
        );
    }

}