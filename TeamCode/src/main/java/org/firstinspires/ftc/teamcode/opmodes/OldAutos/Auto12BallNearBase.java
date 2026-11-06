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
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathCallback;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import java.util.concurrent.TimeUnit;


public abstract class Auto12BallNearBase extends CommandOpMode {
    protected Follower follower;
    protected Robot robot;

    protected static final double FULL_SPEED = 0.95;
    protected static final double MEDIUM_SPEED = 0.75;
    protected static final double SLOW = 0.4;

    // Alliance-specific fields (to be set in derived classes)
    protected Point start;
    protected Point end;
    protected double startHeading;
    protected double endHeading;

    // The initial scoring position is adjusted for drift
    protected Point scoring;
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
    protected Point intake3;
    protected Point intake3Control;
    // open gate points
    protected Point gatePoint;
    protected Point scoringControlPoint;
    protected Point gateIntake;
    protected Point gateIntakeControlPoint;
    protected double gateIntakeHeading;

    protected double gateOpeningHeading;

    protected Timing.Timer timer = new Timing.Timer(30, TimeUnit.SECONDS);

    public static long varWaitTime = 50;

    protected int directionNumber(){
        if (isRedAlliance()){
            return 1;
        }
        return -1;
    }
    protected abstract boolean isRedAlliance();
    private void setupAllianceSpecifics(){
//        startHeading = Math.toRadians(-6.5 + 90 * directionNumber());
//        start = new Point(-63.5, (25.0) * directionNumber(), Point.CARTESIAN);
//        end = new Point(-24.5, 32.0 * directionNumber(), Point.CARTESIAN);
//        endHeading = Math.toRadians(180.0 + 90.0 * directionNumber());
//
//        scoring = new Point(-11.5, 10.0 * directionNumber(), Point.CARTESIAN);
////        cycleScoring = new Point(-18.5, 15.0 * directionNumber(), Point.CARTESIAN);
////        scoringHeading = Math.toRadians(-43.0 * directionNumber());
////        cycleScoringHeading = Math.toRadians(-45.0 * directionNumbe r());
//        intakeHeading = Math.toRadians(180.0 - 90.0 * directionNumber());
//
//        intake1 = new Point(-1 * (-20-.0), 15.0 * directionNumber(), Point.CARTESIAN);
//        intake2 = new Point(-11.5, 10.0 * directionNumber(), Point.CARTESIAN);
//        intake3 = new Point( -1 * (-37.5), 10.0 * directionNumber(), Point.CARTESIAN);
//
//        gatePoint = new Point(-7.0, directionNumber() * (44.5), Point.CARTESIAN);
//        gateOpeningHeading = Math.toRadians(145 * directionNumber());
//        scoringControlPoint = new Point(-1 * (20.0 - 24.0 - 1.0), directionNumber() * 18.0, Point.CARTESIAN);
//        gateIntake = new Point(11.1479, 57.6528 * directionNumber(), Point.CARTESIAN);
//        gateIntakeControlPoint = new Point(18.5, 10 * directionNumber(), Point.CARTESIAN);
//        gateIntakeHeading = Math.toRadians(200 - 90 * directionNumber());


        //FTC(X) -> 72-PX
        //FTC (y) -> P(x) -72
        startHeading = Math.toRadians(-52 * directionNumber());
        start = new Point(-58.06, (47.05) * directionNumber(), Point.CARTESIAN);
        end = new Point(-24.5, 32.0 * directionNumber(), Point.CARTESIAN);
        endHeading = Math.toRadians(180.0 + 90.0 * directionNumber());

        scoring = new Point(-24.5, 10.0 * directionNumber(), Point.CARTESIAN);
//        cycleScoring = new Point(-18.5, 15.0 * directionNumber(), Point.CARTESIAN);
//        scoringHeading = Math.toRadians(-43.0 * directionNumber());
//        cycleScoringHeading = Math.toRadians(-45.0 * directionNumbe r());
        intakeHeading = Math.toRadians(180.0 - 90.0 * directionNumber());

//        intake1 = new Point(-1 * (-4.0), 15.0 * directionNumber(), Point.CARTESIAN);
        intake1 = new Point(9.0, 56.5 * directionNumber(), Point.CARTESIAN);
        intake1Control = new Point(15.0, 8.0 * directionNumber(), Point.CARTESIAN);
        intake2 = new Point(-24.5, 44.5 * directionNumber(), Point.CARTESIAN);
        intake2Control = new Point (-25.0, 15.0 * directionNumber(), Point.CARTESIAN);
//        intake3 = new Point( -1 * (-27), 17.0 * directionNumber(), Point.CARTESIAN);
        intake3 = new Point (36.5, 52.5 * directionNumber(), Point.CARTESIAN);
        intake3Control = new Point(40.5, 4.0, Point.CARTESIAN);

        gatePoint = new Point(2.0, directionNumber() * (47.5), Point.CARTESIAN);
        gateOpeningHeading = Math.toRadians(140.0 * directionNumber());
        scoringControlPoint = new Point(-1 * (20.0 - 24.0 - 1.0), directionNumber() * 18.0, Point.CARTESIAN);
        gateIntake = new Point(-1.1479, 55.6528 * directionNumber(), Point.CARTESIAN);
        gateIntakeControlPoint = new Point(18.5, 10 * directionNumber(), Point.CARTESIAN);
        gateIntakeHeading = Math.toRadians(200 - 90 * directionNumber());

    }
    protected double getIntakeYIncrement(int cycle){
        return cycle == 1 || cycle == 3 ? 28 * directionNumber() : 26.5 * directionNumber();
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

        waitForStart();
        timer.start();

        schedule(
                new SequentialCommandGroup(
                        scorePreload(),
                        intakeAndScore(1),
                        intakeAndScore(2),
                        intakeAndScore(3),
                        gateCycle()
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

        //aiming
        double targetTurretAngle = getTargetTurretAngle();
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
        toScoring.setLinearHeadingInterpolation(startHeading, intakeHeading);

        return new SequentialCommandGroup(
                new FollowPathCommand(follower, toScoring).withTimeout(2000)
                        .deadlineWith(
                                new StartShooterCommand(robot.getShooter(), false)
                        ),
                new InstantCommand(() -> {
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

        if (cycle !=2) {
            toIntake = new Path(new BezierCurve(
                    scoring,
                    controlPoint,
                    intakePosition)
            );
            toIntake.setConstantHeadingInterpolation(intakeHeading);
        }else{
            toIntake = new Path(new BezierLine(
                    scoring, intakePosition
            ));
            toIntake.setConstantHeadingInterpolation(intakeHeading);
        }

        Path toScoring;
        // drive from intake position to scoring position
        if (cycle == 1) {
            // special path to avoid hitting the gate while driving
            toScoring = new Path(new BezierCurve(
                    gatePoint,
                    scoringControlPoint,
                    scoring)
            );

            toScoring.setConstantHeadingInterpolation(intakeHeading);
        } else {
            toScoring = new Path(new BezierLine(
                    intakePosition,
                    scoring)
            );
            toScoring.setConstantHeadingInterpolation(intakeHeading);
        }
        //open gate
        Path openGate = new Path(new BezierLine(
                intakePosition,
                gatePoint
        ));
        openGate.setConstantHeadingInterpolation(gateOpeningHeading);

        openGate.setZeroPowerAccelerationMultiplier(4.5);

        PathChain gatePath = new PathChain(
            openGate
        );
        gatePath.setCallbacks(new PathCallback(
            0.9, () -> {robot.getIntake().stopIntake();}, PathCallback.PARAMETRIC, 0
        ));

        return new SequentialCommandGroup(
                // drive to intaking
                // close purple feeder before intaking so we have the hardware limit
                new InstantCommand(()->{follower.setMaxPower(MEDIUM_SPEED);}),
                new FollowPathCommand(follower, toIntake).withTimeout(2200)
                        .deadlineWith(
                                new IdleShooterCommand(robot.getShooter(), false),
                                new IntakeCommand(robot.getIntake(), false, false)
                        ),

                // intake balls, drive to scoring
                        //new IntakeCommand(robot.getIntake(), robot.getTransfer()),
                new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            follower.setMaxPower(FULL_SPEED);
                        }),
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> {
                                            follower.setMaxPower(MEDIUM_SPEED);
                                        }),
                                        new InstantCommand(() -> Log.d("autonomous", "entered!")),
                                        new FollowPathCommand(follower, gatePath).withTimeout(1000)
//                                                new WaitCommand(100)
                                ),
                                new InstantCommand(() -> {}),
                                () -> cycle == 1
                                ),
                        new InstantCommand(() -> Log.d("AutoCheckpoint", "Debugging")),
//                             new WaitCommand(1500),
                        new InstantCommand(() -> {
                            follower.setMaxPower(FULL_SPEED);
                        }),
                        new InstantCommand(() -> Log.d("AutoCheckpoint", "Debugging")),
                        // drive to scoring & prepare shooter
                        // in the middle of the path (750ms later) stop the intake & transfer
                        new ParallelRaceGroup(
                                new FollowPathCommand(follower, toScoring).withTimeout(2000),
                                new StartShooterCommand(robot.getShooter(), false)
                        )
                ),
                new InstantCommand(() -> Log.d("AutoCheckpoint", "Debugging")),
                new InstantCommand(() -> {
                    Log.d("Scoring Position: ", "X: " + follower.getPose().getX() + "Y: " + follower.getPose().getY() + "Heading: " + Math.toDegrees(follower.getPose().getHeading()));
                }),
                new RapidFireShoot(robot.getIntake(), robot.getShooter(), false),
                new InstantCommand(() -> Log.d("AutoCheckpoint", "Debugging"))
        );
    }
    private Command gateCycle() {
        Path toGate = new Path(new BezierCurve(
                scoring,
                gateIntakeControlPoint,
                gateIntake
        ));
        toGate.setLinearHeadingInterpolation(intakeHeading, gateIntakeHeading);

        Path toScoring = new Path(new BezierCurve(
                gateIntake,
                gateIntakeControlPoint,
                scoring
        ));
        toScoring.setLinearHeadingInterpolation(gateIntakeHeading, intakeHeading);

        return new SequentialCommandGroup(
                new InstantCommand(()->{follower.setMaxPower(FULL_SPEED);}),
                new FollowPathCommand(follower, toGate).withTimeout(2500)
                        .deadlineWith(new IntakeCommand(robot.getIntake(), false, false)),
                new InstantCommand(() -> Log.d("AutoCheckpoint", "Debugging")),
                new InstantCommand(()-> {
                    Log.d("AutonomousPeriod", "X: " + follower.getPose().getX());
                    Log.d("AutonomousPeriod,", "Y: " + follower.getPose().getY());
                    Log.d("AutonomousPeriod,", "heading: " + follower.getPose().getHeading());
                }),

                new FollowPathCommand(follower, toScoring)
                    .deadlineWith(
                            new IntakeCommand(robot.getIntake(), false, false),
                            new StartShooterCommand(robot.getShooter(), false)
                    ),
                new WaitCommand(700),
                new RapidFireShoot(robot.getIntake(), robot.getShooter(), false)
        );
    }
    protected Command park() {
        Path parking = new Path(new BezierLine(
                scoring,
                end
        )
        );
        parking.setLinearHeadingInterpolation(intakeHeading, endHeading, 0.5);

        return new FollowPathCommand(
                follower, parking
        );
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

        return turretAngleDeg+10;
    }
}