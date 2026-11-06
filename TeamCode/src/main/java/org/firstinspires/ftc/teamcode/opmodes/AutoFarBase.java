package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.Robot.RED_ALLIANCE;
import static org.firstinspires.ftc.teamcode.opmodes.ManualDrive.BLUE_GOAL_X;
import static org.firstinspires.ftc.teamcode.opmodes.ManualDrive.BLUE_GOAL_Y;
import static org.firstinspires.ftc.teamcode.opmodes.ManualDrive.RED_GOAL_X;
import static org.firstinspires.ftc.teamcode.opmodes.ManualDrive.RED_GOAL_Y;
import static org.firstinspires.ftc.teamcode.opmodes.ManualDrive.TURRET_MAX_ANGLE;
import static org.firstinspires.ftc.teamcode.opmodes.ManualDrive.TURRET_MIN_ANGLE;
import static org.firstinspires.ftc.teamcode.subsystems.TurretWithEncoder.MAX_DEGREE;
import static org.firstinspires.ftc.teamcode.subsystems.TurretWithEncoder.MIN_DEGREE;

import android.util.Log;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
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
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.AimUtil;

import java.util.concurrent.TimeUnit;

public abstract class AutoFarBase extends CommandOpMode {
    protected Follower follower;
    protected Robot robot;

    protected static final double FULL_SPEED = 1.0;
    protected static final double MEDIUM_SPEED = 0.75;
    protected static final double SLOW = 0.50;
    protected static final double WALL_INTAKE_SPEED = 0.55;

    protected int AprilTagID = 21;

    protected double startHeading;
    protected Point start;

    protected Point scoringInitial;
    protected Point scoring;

    protected Point intakeSpike1;
    protected double intakeStartHeading;
    protected double intakeHeading;
    protected Point intakeSpikeFinal;

    protected Point wallIntakeInitial;
    protected Point wallIntakeFinal;
    protected Point wallIntakeBack;

    protected double intakeOffsetX = 8.0;
    protected double intakeOffsetY;

    protected Point parking;
    protected double parkingHeading;

    protected Timing.Timer timer = new Timing.Timer(30, TimeUnit.SECONDS);
    public static long varWaitTime = 0;

    protected int directionNumber(){
        if(isRedAlliance()){
            return 1;
        }else{
            return -1;
        }
    }

    protected abstract boolean isRedAlliance();
    protected abstract Command scheduleSequence();

    protected void setupAllianceSpecifics() {
        startHeading = Math.toRadians(90.0 * directionNumber());
        start = new Point(63.2, 7.7 * directionNumber(), Point.CARTESIAN);

        scoringInitial = new Point(63.46, 7.55 * directionNumber(), Point.CARTESIAN);
        //TODO: decrease y coordinate of scoring position to decrease movement distance
        scoring = new Point(54.0, (14.5) * directionNumber(), Point.CARTESIAN);

        intakeSpike1 = new Point(32.0, 27.6 * directionNumber(), Point.CARTESIAN);
        intakeStartHeading = Math.toRadians(110.0 * directionNumber());
        intakeHeading = Math.toRadians(90.0 * directionNumber());
        intakeSpikeFinal = new Point(37.5, 58.5 * directionNumber(), Point.CARTESIAN);

        wallIntakeInitial = new Point(64.5, 30.0 * directionNumber(), Point.CARTESIAN);
        wallIntakeFinal = new Point(64.5, (58.75) * directionNumber(), Point.CARTESIAN);
        wallIntakeBack = new Point(64.5, (52.0) * directionNumber(), Point.CARTESIAN);

        intakeOffsetX = -8.0;
        intakeOffsetY = -14.0 * directionNumber();

        parking = new Point(60.0, 36.0 * directionNumber(), Point.CARTESIAN);
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
                scheduleSequence()
        );
    }



    @Override
    public void run() {
        super.run();
        follower.update();
        robot.resetCache();
        Pose robotPos = follower.getPose();
        Log.d("Follower ", "Heading value: " + Math.toDegrees(follower.getTotalHeading()));
        Log.d("Follower ", "Pose: " + robotPos.toString());
        telemetry.addData("Shooter velocity: ", robot.getShooter().getVelocity());
        if (timer.elapsedTime() > 27 && timer.elapsedTime() < 29.9) {
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
        return new SequentialCommandGroup(
                new StartShooterCommand(robot.getShooter(), true),
                // give the shooters some time to spin up
                new WaitUntilCommand(robot.getShooter()::isShooterReady).withTimeout(4000),
                new RapidFireShoot(robot.getIntake(), robot.getShooter(), true)
        );
    }

    protected Command scoreLoadingZone() {
        Path intakeBalls = new Path(new BezierCurve(
                scoring,
                intakeSpike1,
                intakeSpikeFinal
        ));
        intakeBalls.setLinearHeadingInterpolation(intakeStartHeading,intakeHeading);
        intakeBalls.setPathEndTimeoutConstraint(50);
        intakeBalls.setZeroPowerAccelerationMultiplier(4.5);

        Path shootLoadingZone = new Path(new BezierLine(
                intakeSpikeFinal,
                scoring
        ));
        shootLoadingZone.setZeroPowerAccelerationMultiplier(3.5);
        shootLoadingZone.setConstantHeadingInterpolation(intakeHeading);
        shootLoadingZone.setPathEndTimeoutConstraint(300);

        return new org.firstinspires.ftc.teamcode.commands.SequentialCommandGroup(
                new InstantCommand(robot.getIntake()::closeLatch),

                // drive and intake 3rd spike of balls
                new ParallelRaceGroup(
                        new org.firstinspires.ftc.teamcode.commands.SequentialCommandGroup(
                                new InstantCommand(() -> follower.setMaxPower(FULL_SPEED)),
                                new FollowPathCommand(follower, intakeBalls).withTimeout(2500)
                        ),
                        new IntakeCommand(robot.getIntake(), false, false)
                ),

                // drive back and score balls
                new ParallelCommandGroup(
                        new IntakeCommand(robot.getIntake(), false, false, () -> true).withTimeout(1500),
                        new InstantCommand(() -> follower.setMaxPower(FULL_SPEED)),
                        new FollowPathCommand(follower, shootLoadingZone).withTimeout(1500)
                                .deadlineWith(new StartShooterCommand(robot.getShooter(), true))
                )
        );
    }

    // sequence of events:
    // 1. Shoot balls
    // 2. Simultaneously use vision to create an intaking path
    // 3. intake
    // 4. drive back, but don't shoot yet
    protected Command scoreWall(int cycle) {
        Path intakePresets = new Path(new BezierLine(
                scoringInitial, wallIntakeFinal
        ));
        intakePresets.setConstantHeadingInterpolation(intakeHeading);
        intakePresets.setZeroPowerAccelerationMultiplier(4.5);
        intakePresets.setPathEndTimeoutConstraint(50);

        return new org.firstinspires.ftc.teamcode.commands.SequentialCommandGroup(
                new ParallelCommandGroup(
                        new RapidFireShoot(robot.getIntake(), robot.getShooter(), true),
                        new ConditionalCommand(
                                robot.getLimelight().sampleBestBallWindowCommand().withTimeout(400),
                                new InstantCommand(),
                                () -> cycle != 1
                        )
                ),
                new InstantCommand(() -> {Log.d("Auto: ", "after scoring");}),
                new InstantCommand(robot.getIntake()::closeLatch),
                robot.getLimelight().followBestBallWindowToWallCommand(follower).withTimeout(2000)
                    .deadlineWith(new IntakeCommand(robot.getIntake(), false, false, () -> true)),
                new InstantCommand(() -> {Log.d("Auto: ", "after intake");}),
                new ParallelCommandGroup(
                    new IntakeCommand(robot.getIntake(), false, false).withTimeout(1500),
                    new FollowPathCommand(follower, this::createShootWallZone, true).withTimeout(2000),
                    new StartShooterCommand(robot.getShooter(), true)
                )
        );
    }

    protected Command scoreWallInitial() {
        Path intakePresets = new Path(new BezierLine(
            scoringInitial, wallIntakeFinal
        ));
        intakePresets.setConstantHeadingInterpolation(intakeHeading);
        intakePresets.setZeroPowerAccelerationMultiplier(4.5);
        intakePresets.setPathEndTimeoutConstraint(50);

        return new org.firstinspires.ftc.teamcode.commands.SequentialCommandGroup(
            new InstantCommand(robot.getIntake()::closeLatch),
                new ParallelRaceGroup(
                    new org.firstinspires.ftc.teamcode.commands.SequentialCommandGroup(
                        new InstantCommand(() -> follower.setMaxPower(FULL_SPEED)),
                        new FollowPathCommand(follower, intakePresets).withTimeout(1500),
                        // if it's the 3 balls against the wall (i.e. first wall cycle), then wait a bit to make sure they're fully in
                        new WaitCommand(300)
                    ),
                    new IntakeCommand(robot.getIntake(), false, false, () -> true)
                ),
            new InstantCommand(() -> {Log.d("Auto: ", "after intake");}),

            new ParallelCommandGroup(
                new IntakeCommand(robot.getIntake(), false, false).withTimeout(1500),
                new FollowPathCommand(follower, this::createShootWallZone, true).withTimeout(1500),
                new StartShooterCommand(robot.getShooter(), true)
            ),
            new RapidFireShoot(robot.getIntake(), robot.getShooter(), true)
        );
    }

    protected Command park() {
        Path toParking = new Path(new BezierLine(scoring, parking));
        toParking.setConstantHeadingInterpolation(intakeHeading);
        toParking.setZeroPowerAccelerationMultiplier(4.5);
        return new SequentialCommandGroup(
            new RapidFireShoot(robot.getIntake(), robot.getShooter(), true),
            new InstantCommand(() -> {Log.d("Auto: ", "after scoring");}),
            new InstantCommand(robot.getIntake()::closeLatch),
            new FollowPathCommand(follower, toParking)
        );
    }

    protected Point incrementX(Point point, double increment) {
        return new Point(point.getX() + increment, point.getY(), Point.CARTESIAN);
    }
    protected Point incrementY(Point point, double increment){
        return new Point(point.getX(), point.getY() + increment, Point.CARTESIAN);
    }

    private PathChain createShootWallZone() {
        Path path = new Path(new BezierLine(
                new Point(follower.getPose()),
                scoring
        ));

        PathChain toScoring = new PathChain(path);

        path.setConstantHeadingInterpolation(intakeHeading);
        path.setPathEndTimeoutConstraint(300);
        path.setZeroPowerAccelerationMultiplier(3.0);
        toScoring.setCallbacks(
            new PathCallback(0.7, () -> follower.setMaxPower(SLOW), PathCallback.PARAMETRIC, 0) //see if this can be increased
        );
        return toScoring;
    }

    private Path createRunTowardBallGroupPath(double distance) {
        Pose currentPose = follower.getPose();
        double heading = currentPose.getHeading();
        Point startPoint = new Point(currentPose);
        Point targetPoint = new Point(
                currentPose.getX() + distance * Math.cos(heading),
                clamp(60.0,65.0, currentPose.getY() + distance * Math.sin(heading)),
                Point.CARTESIAN
        );

        Path path = new Path(new BezierLine(startPoint, targetPoint));
        path.setConstantHeadingInterpolation(heading);
        path.setPathEndTimeoutConstraint(50);
        path.setZeroPowerAccelerationMultiplier(4.5);
        return path;
    }
    private static double clamp(double min, double max, double input){
        return Math.max(min, Math.min(max, input));
    }
}