//package org.firstinspires.ftc.teamcode.opmodes
//
//import android.util.Log
//import com.arcrobotics.ftclib.command.*
//import com.arcrobotics.ftclib.command.SequentialCommandGroup
//import com.arcrobotics.ftclib.util.Timing
//import org.firstinspires.ftc.teamcode.Robot
//import org.firstinspires.ftc.teamcode.commands.*
//import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower
//import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point
//import java.util.concurrent.TimeUnit
//
//abstract class Near9MotifAutoBase : CommandOpMode() {
//
//    protected lateinit var follower: Follower
//    protected lateinit var robot: Robot
//    protected val FULL_SPEED = 1.0
//    protected val SLOW = 0.19
//
//    protected var AprilTagID = 21
//    protected val timer = Timing.Timer(30, TimeUnit.SECONDS)
//
//    // Properties to override in derived classes
//    protected abstract val start: Point
//    protected abstract val startHeading: Double
//    protected abstract val end: Point
//    protected abstract val endHeading: Double
//
//    protected abstract val scoring: Point
//    protected abstract val cycleScoring: Point
//    protected abstract val scoringHeading: Double
//    protected abstract val cycleScoringHeading: Double
//
//    protected abstract val intakeHeading: Double
//    protected abstract val intake1: Point
//    protected abstract val intake2: Point
//    protected abstract val intake3: Point
//    protected abstract val scoringControlPoint: Point
//
//    override fun initialize() {
//        telemetry.addLine("Initializing...")
//        telemetry.update()
//
//        follower = Follower(hardwareMap)
//        follower.setStartingPose(Pose(start.x, start.y, startHeading))
//        follower.setMaxPower(FULL_SPEED)
//
//        Robot.RED_ALLIANCE = isRedAlliance()
//        Robot.FAR_AUTO = false
//        Robot.AUTO_NOSORTING = false
//        robot = Robot(true, hardwareMap)
//
//        while (opModeInInit()) {
//            AprilTagID = robot.vision.getAprilTagId(Robot.RED_ALLIANCE)
//            telemetry.addLine("AprilTagID: $AprilTagID")
//            Memory.motif = when (AprilTagID) {
//                21 -> "GPP"
//                22 -> "PGP"
//                23 -> "PPG"
//                else -> Memory.motif
//            }
//            telemetry.addLine("Motif: ${Memory.motif}")
//            telemetry.update()
//        }
//
//        waitForStart()
//        timer.start()
//
//        schedule(
//            SequentialCommandGroup(
//                InstantCommand(Runnable { follower.setMaxPower(0.6) }),
//                scorePreload(),
//                InstantCommand({ follower.setMaxPower(FULL_SPEED) }),
//                intakeAndScore(1),
//                intakeAndScore(2),
//                ParallelRaceGroup(
//                    park(),
//                    StopShootingCommand(robot.shooter)
//                )
//            )
//        )
//    }
//
//    override fun run() {
//        super.run()
//        Log.d("Follower", "Heading value: ${Math.toDegrees(follower.totalHeading)}")
//        Log.d("Follower", "Pose: ${follower.pose}")
//        follower.update()
//        follower.drawOnDashBoard()
//        robot.resetCache()
//
//        val time = timer.elapsedTime()
//        if (time > 28.0 && time <= 29.5) {
//            Memory.autoEndPose = follower.pose
//        }
//    }
//
//    private fun scorePreload(): Command {
//        val toScoring = Path(BezierLine(start, scoring))
//        toScoring.setLinearHeadingInterpolation(startHeading, scoringHeading)
//        toScoring.setZeroPowerAccelerationMultiplier(2.5)
//
//        return SequentialCommandGroup(
//            ParallelRaceGroup(
//                ParallelCommandGroup(
//                    FollowPathCommand(follower, toScoring),
//                    InstantCommand({ robot.vision.visionPortal.close() }),
//                ),
//                StartShooterCommand(robot.shooter, false),
//                ParallelCommandGroup(
//                    InstantCommand({
//                        if (Memory.motif[0] == 'P') robot.feeder.feedPurpleArtifact()
//                        else robot.feeder.feedGreenArtifact()
//                    }),
//                    NewIntakeCommand(robot.intake, false, false)
//                )
//            ),
//            InstantCommand({
//                robot.intake.closeLatch()
//            }),
//            WaitCommand(100),
//            NewRapidFireShoot(robot.intake, robot.shooter, false)
//        )
//    }
//
//    protected fun intakeAndScore(cycle: Int): Command {
//        val intakePosition = when (cycle) {
//            1 -> intake1
//            2 -> intake2
//            3 -> intake3
//            else -> intake1
//        }
//
//        val intakeIncr = when (cycle) {
//            1 -> intakeIncrementSmall()
//            2 -> intakeIncrement()
//            3 -> intakeIncrement()
//            else -> intakeIncrement()
//        }
//
//        val toIntake = Path(BezierLine(
//            if (cycle == 1) scoring else cycleScoring,
//            intakePosition))
//        toIntake.setLinearHeadingInterpolation(
//            if (cycle == 1) scoringHeading else cycleScoringHeading,
//            intakeHeading
//        )
//
//        val intakeBalls = Path(BezierLine(
//            intakePosition,
//            incrementY(intakePosition, intakeIncr)))
//        intakeBalls.setConstantHeadingInterpolation(intakeHeading)
//
//        val toScoring = if (cycle == 2) {
//            Path(BezierCurve(
//                incrementY(intakePosition, intakeIncr),
//                scoringControlPoint,
//                cycleScoring))
//                .apply { setLinearHeadingInterpolation(intakeHeading, cycleScoringHeading) }
//        } else {
//            Path(BezierLine(
//                incrementY(intakePosition, intakeIncr),
//                cycleScoring))
//                .apply { setLinearHeadingInterpolation(intakeHeading, cycleScoringHeading) }
//        }
//
//        return SequentialCommandGroup(
//            ParallelRaceGroup(
//                IdleShooterCommand(robot.shooter, false),
//                FollowPathCommand(follower, toIntake)
//            ),
//            ParallelCommandGroup(
//                IntakeCommand(robot.intake, robot.transfer),
//                SequentialCommandGroup(
//                    InstantCommand( { follower.setMaxPower(SLOW) }),
//                    FollowPathCommand(follower, intakeBalls),
//                    InstantCommand({ follower.setMaxPower(FULL_SPEED) }),
//                    // drive to scoring & prepare shooter
//                    // in the middle of the path (750ms later) stop the intake & transfer
//                    ParallelCommandGroup(
//                        ParallelRaceGroup(
//                            FollowPathCommand(follower, toScoring),
//                            StartShooterCommand(robot.shooter, false)
//                        ),
//                        SequentialCommandGroup(
//                            InstantCommand({
//                                if (Memory.motif[0] == 'P') robot.feeder.feedPurpleArtifact()
//                                else robot.feeder.feedGreenArtifact()
//                            }),
//                            WaitCommand(750),
//                            InstantCommand(robot.intake::stopIntake)
//                        )
//                    )
//                )
//            ),
//            WaitCommand(750),
//            InstantCommand({
//                robot.feeder.closeGreenFeeder()
//                robot.feeder.closePurpleFeeder()
//            }),
//            AutoSortingShootCommand(robot.feeder, robot.shooter, robot.intake, robot.transfer, Memory.motif, false)
//        )
//    }
//
//    private fun park(): Command {
//        val parking = Path(BezierLine(cycleScoring, end))
//        parking.setLinearHeadingInterpolation(cycleScoringHeading, endHeading, 0.5)
//        return FollowPathCommand(follower, parking)
//    }
//
//    protected fun incrementY(point: Point, increment: Double) = Point(point.x, point.y + increment, Point.CARTESIAN)
//
//    protected abstract fun intakeIncrement(): Double
//    protected abstract fun intakeIncrementSmall(): Double
//    protected abstract fun isRedAlliance(): Boolean
//}
