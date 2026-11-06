package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.Robot.RED_ALLIANCE;
import static org.firstinspires.ftc.teamcode.commands.Memory.autoEndPose;
import static org.firstinspires.ftc.teamcode.commands.Memory.compensatedAngleToGoal;
import static org.firstinspires.ftc.teamcode.commands.Memory.compensatedDistanceToGoal;
import static org.firstinspires.ftc.teamcode.commands.Memory.distanceToGoal;
import static org.firstinspires.ftc.teamcode.commands.Memory.velocity;
import static org.firstinspires.ftc.teamcode.subsystems.Turret.MAX_DEGREE;
import static org.firstinspires.ftc.teamcode.subsystems.Turret.MIN_DEGREE;

import android.util.Log;
import android.util.Pair;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.DynamicRapidFireShoot;
import org.firstinspires.ftc.teamcode.commands.DynamicStartShooterCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.Memory;
import org.firstinspires.ftc.teamcode.commands.RapidFireShoot;
import org.firstinspires.ftc.teamcode.commands.StartShooterCommand;
import org.firstinspires.ftc.teamcode.commands.TeleopStopShootingCommand;
import org.firstinspires.ftc.teamcode.commands.VoltageSensing;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;
import org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.util.PIDFController;
import org.firstinspires.ftc.teamcode.subsystems.LimelightVision;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.AimUtil;
import org.firstinspires.ftc.teamcode.vision.VisionLocalizer;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.Point;
import org.opencv.objdetect.ArucoDetector;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

@Disabled
@Config
@TeleOp(name = "Z Manual Drive Experimental", group = "Teleop")
public class ManualDrive_Experimental extends CommandOpMode {
    long lastUpdate = System.currentTimeMillis();
    private Robot robot;
    private Follower follower;
    private double prevAngleToGoal = 0.0;
    private double angleToGoal = 0.0;
    private double angleToGate = Math.toRadians(122);
    private VoltageSensing voltageSensing;

    // Goal coordinates for red and blue alliances (in inches, field coordinate system)
    // note: tune these if needed
    public static double RED_GOAL_X = -60.0;
    public static double RED_GOAL_Y = 60.0;
    public static double BLUE_GOAL_X = -60.0;
    public static double BLUE_GOAL_Y = -60.0;

    private Servo led;

    // Auto-aiming state
    private boolean aiming = true;
//    private Limelight3A limelight;

    // Turret limits in degrees relative to robot heading -157.5 to 157.5
    public static final double TURRET_MIN_ANGLE = -157.5;
    public static final double TURRET_MAX_ANGLE = 157.5;

    public static double GATE_INTAKE_HEADING = 119.0;

    // determines whether we will enable position lock
    // set this to true when Driver 2's B button is pressed
    // set this to false once a shooting command button is pressed
    private boolean gateIntakeHeading = false;
    private GamepadEx g1, g2;

    // For limelight relocalization
    private Pose lastPoseForStillCheck = null;
    private long lastMotionTime = 0;
    private long lastRelocalizeTime = 0;

    // robot heading lock PIDs: one for gate intake, the other for continuous aiming
    PIDFController gateHeadingController = new PIDFController(FollowerConstants.headingPIDFCoefficients);
    PIDFController continuousAimingController = new PIDFController(FollowerConstants.headingPIDFCoefficients);

    VisionLocalizer localizer;

    VisionLocalizer.VisionPose3D cameraTransform;
    ArucoDetector detector;
    MatOfDouble distCoeffs;
    Mat cameraMatrix;
    Map<Integer, VisionLocalizer.VisionPose3D> fieldMap;

    private boolean holdPosition = false;

    private double prevoiusTurretAngle = 0;
    private static double testCompensatedAngleToGoal = 0.0;
    public static double angleCompensationMultiplier = 500;

    @Override
    public void initialize() {
        Log.d("ManualDrive::voltageSensing: ", "NEW RUN!");
        telemetry = new MultipleTelemetry(
            telemetry,
            FtcDashboard.getInstance().getTelemetry()
        );
        telemetry.addLine("Initializing...");
        telemetry.update();

        robot = new Robot(false, hardwareMap);
        follower = new Follower(hardwareMap, false);

        // update heading pidf coefficients so it works well in teleop
        FollowerConstants.secondaryHeadingPIDFCoefficients = new CustomPIDFCoefficients(0.7, 0.0, 0.03, 0.0); //p: 3.5, i:
        FollowerConstants.headingPIDFCoefficients = new CustomPIDFCoefficients(0.6, 0.00, 0.08, 0.0); //p: 1.0 d: 0.09

        // --- Vision Setup
        // 1. Setup Field Map from official DECODE constants
        fieldMap = VisionLocalizer.DECODE_FIELD_MAP;

        // 2. Setup Camera Parameters (Calibrated Arducam at 1280x720)
        cameraMatrix = new Mat(3, 3, CvType.CV_64F);
        cameraMatrix.put(0, 0,
            518.5498, 0.0, 627.7407,
            0.0, 516.8600, 375.1764,
            0.0, 0.0, 1.0);

        distCoeffs = VisionLocalizer.getArducamDistCoeffs();

        // Use standard camera transform from VisionLocalizer
        cameraTransform = VisionLocalizer.ARDUCAM_TRANSFORM;

        localizer = new VisionLocalizer(fieldMap, cameraMatrix, distCoeffs, cameraTransform);
        // --- Vision Setup ^-^
//        limelight = robot.getLimelight().getLimelightCamera();
//        limelight.setPollRateHz(100);
//        limelight.start();

        //gate intake heading stuffs
//        if(RED_ALLIANCE){
//            GATE_INTAKE_HEADING = 123.1;
//        } else {
//            GATE_INTAKE_HEADING = -124.2;
//        }

        Pose startPose;
        if (autoEndPose != null) {
            // convert our coordinate system to the standard FTC coordinate system 😅
            startPose = autoEndPose;
        }else{
            startPose = new Pose(0.0, 0.0, 0.0);
        }
        telemetry.addData("Auto End Pose: ", "X: " + startPose.getX() + "Y: " + startPose.getY() + "Heading: " + startPose.getHeading());

        follower.setPose(startPose);
        follower.update();

        lastPoseForStillCheck = follower.getPose();

        led = hardwareMap.get(Servo.class, "led");

        // create the gamepad objects
        g1 = new GamepadEx(gamepad1);
        g2 = new GamepadEx(gamepad2);

        follower.startTeleopDrive();

        telemetry.addLine("Ready!");
        telemetry.addData("Alliance Color: ", RED_ALLIANCE ? "Red" : "Blue");
        Log.d("Alliance Color: ", RED_ALLIANCE ? "Red" : "Blue");
        telemetry.update();

        // all code above this happens during init
        // the robot will now wait until start is pressed before doing anything below here.

        // create all gamepad driver controls
        driver1Controls();
        driver2Controls();

        waitForStart();

        robot.startTele();
    }
    /*
        DRIVER 1 CONTROLS:
        A - Intake toggle
        Y - Reverse Intake toggle
        X - Rapid fire shoot (near)
        B - Rapid fire shoot (far) / Limelight relocalize (when held)
        RIGHT_STICK_BUTTON - Reset heading
        START - Corner relocalization
        BACK - Change alliance color
        LEFT BUMPER - dynamic start shooter
        RIGHT BUMPER - gate intake heading
        TRIGGERS - Drive turning
        DPAD_LEFT: stop shooter motor
        DPAD_RIGHT: gate intake heading

        DRIVER 2 CONTROLS
        DPAD_LEFT: stop shooter motor
        LEFT BUMPER - start shooter (near)
        RIGHT BUMPER - start shooter (far)
        DPAD RIGHT - dynamic start shooter command
        A - set turret to face forward and hold its position
        Y - disable
        B - LL relocalize
        START - Change alliance color
     */

    private void driver1Controls() {
        // A - Intake
        new GamepadButton(
            g1, GamepadKeys.Button.A)
            .toggleWhenPressed(
                new SequentialCommandGroup(
                    new InstantCommand(() -> {
                        led.setPosition(0.0);
                    }),
                    new InstantCommand(() -> {Log.d("ManualDrive::Gamepad1:A: ", "pressed once");}),
                    new IntakeCommand(robot.getIntake(), false, false, ()-> true)
                )
            );

        // Y - Reverse Intake
        new GamepadButton(
            g1, GamepadKeys.Button.Y)
            .toggleWhenPressed(
                new SequentialCommandGroup(
                    new InstantCommand(() -> {Log.d("ManualDrive::Gamepad1:Y: ", "pressed once");}),
                    new IntakeCommand(robot.getIntake(), true, false)
                )
            );

        // X - Dynamic rapid fire shoot
        new GamepadButton(
            g1, GamepadKeys.Button.X)
            .whenPressed(
                new SequentialCommandGroup(
                    new InstantCommand(() -> {
                        Log.d("ManualDrive::Gamepad1:DPAD_UP: ", "pressed once");
                        Log.d("ManualDrive::Gamepad1:DPAD_UP: ", "motor velocity: " + robot.getShooter().getVelocity());
                    }),
                    new ParallelCommandGroup(
                        new DynamicRapidFireShoot(robot.getIntake(), robot.getShooter()),
                        new SequentialCommandGroup(
                            new WaitCommand(400),
                            new InstantCommand(() -> {
                                led.setPosition(0.555);
                            }),
                            new WaitCommand(500),
                            new InstantCommand(() -> {
                                led.setPosition(0.0);
                            })
                        )
                    ),
                    new InstantCommand(() -> {
                        gateIntakeHeading = false;
                        holdPosition = false;
                    })
                )
            );

        // B - Rapid fire shoot (far) (Note: Limelight relocalize is handled in run() via gamepad1.b check)
        new GamepadButton(
            g1, GamepadKeys.Button.B)
            .whenPressed(
                new SequentialCommandGroup(
                    new InstantCommand(() -> {
                        Log.d("ManualDrive::Gamepad1:B: ", "pressed once");
                        Log.d("ManualDrive::Gamepad1:B: ", "motor velocity: " + robot.getShooter().getVelocity());
                    }),
                    new RapidFireShoot(robot.getIntake(), robot.getShooter(), true),
                    new InstantCommand(() -> {
                        gateIntakeHeading = false;
                        holdPosition = false;
                    })
                )
            );

        // RIGHT_STICK_BUTTON - Reset heading
        new GamepadButton(
            g1, GamepadKeys.Button.RIGHT_STICK_BUTTON)
            .whenPressed(
                new InstantCommand(() -> {
                    Pose pose = follower.getPose();
                    follower.setPose(new Pose(pose.getX(), pose.getY(), RED_ALLIANCE ? Math.toRadians(90) : Math.toRadians(-90)));
                })
            );

        // START - Gate relocalization
        new GamepadButton(
            g1, GamepadKeys.Button.START)
            .whenPressed(
                new SequentialCommandGroup(
                    new InstantCommand(() -> {
                        follower.setPose(RED_ALLIANCE ?
                            new Pose(-7.3, 53.35, Math.toRadians(90))
                            : new Pose(-7.3, -53.35, Math.toRadians(-90)));
                    }),
                    new InstantCommand(() -> {
                        Log.d("ManualDrive::Corner Relocalization: ",
                            "Current Pose: X: " + follower.getPose().getX()
                                + " Y: " + follower.getPose().getY()
                                + " Heading: " + follower.getPose().getHeading());
                        gamepad1.rumble(250);
                        gamepad2.rumble(250);
                    })
                )
            );

        new GamepadButton(
            g1, GamepadKeys.Button.BACK)
            .whenPressed(
                new InstantCommand(() -> {
                    if (RED_ALLIANCE) {
                        RED_ALLIANCE = false;
                    } else {
                        RED_ALLIANCE = true;
                    }
                }
                )
            );

        // DPAD_UP - Rapid fire shoot (near)
//        new GamepadButton(
//            g1, GamepadKeys.Button.DPAD_UP)
//            .whenPressed(
//                new SequentialCommandGroup(
//                    new InstantCommand(() -> {
//                        Log.d("ManualDrive::Gamepad1:X: ", "pressed once");
//                        Log.d("ManualDrive::Gamepad1:X: ", "motor velocity: " + robot.getShooter().getVelocity());
//                    }),
//                    new RapidFireShoot(robot.getIntake(), robot.getShooter(), false),
//                    new InstantCommand(() -> {
//                        gateIntakeHeading = false;
//                        holdPosition = false;
//                    })
//                )
//            );

        // DPAD_DOWN - automatic gate intake heading
//        new GamepadButton(
//            g1, GamepadKeys.Button.DPAD_DOWN)
//            .whenPressed(
//                new InstantCommand(() -> {
//                        Log.d("ManualDrive::Gamepad1:gateIntakeHeading ", "heading = " + follower.getPose().getHeading());
//                        Pose currentPose = follower.getPose();
//                        follower.holdPoint(new BezierPoint(new org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point(currentPose.getX(), currentPose.getY(), org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point.CARTESIAN)),
//                            angleToGate);
//                    }).withTimeout(500)
//            );

//        new GamepadButton(
//            g2, GamepadKeys.Button.BACK).
//            whenPressed(
//                new InstantCommand(() -> {
//                    cameraRelocalize();
//                }
//                )
//            );


        new GamepadButton(
            g1, GamepadKeys.Button.LEFT_BUMPER)
            .whenPressed(
                new SequentialCommandGroup(
                    new DynamicStartShooterCommand(robot.getShooter()),
                    new InstantCommand(()-> {
                        Log.d("ManualDrive::Gamepad1:LEFT_BUMPER: ", "pressed once");
                        Log.d("ManualDrive::Gamepad1:LEFT_BUMPER: ", "motor velocity: " + robot.getShooter().getVelocity());
                    })
                )
            );

        new GamepadButton(
            g1, GamepadKeys.Button.RIGHT_BUMPER)
            .whenPressed(
                new ParallelCommandGroup(
                    new InstantCommand(() -> {
                        gateIntakeHeading = true;
                        Log.d("ManualDrive::DPAD_RIGHT: ", "True");
                        led.setPosition(0.0);
                    }),
                    new IntakeCommand(robot.getIntake(), false, false, () -> true)
                )
            );
        new GamepadButton(
            g1, GamepadKeys.Button.DPAD_LEFT)
            .whenPressed(
                new SequentialCommandGroup(
                    new InstantCommand(()->{Log.d("ManualDrive::Gamepad1:DPAD_LEFT", "pressed once");}),
                    new TeleopStopShootingCommand(robot.getShooter())
                )
            );

        new GamepadButton(
            g1, GamepadKeys.Button.DPAD_RIGHT)
            .whenPressed(
                new InstantCommand(() -> {
                    gateIntakeHeading = true;
                    Log.d("ManualDrive::DPAD_RIGHT: ", "True");
                }
                )
            );
    }

    private void driver2Controls() {
        new GamepadButton(
            g2, GamepadKeys.Button.DPAD_LEFT)
            .whenPressed(
                new SequentialCommandGroup(
                    new InstantCommand(()->{Log.d("ManualDrive::Gamepad2:DPAD_LEFT", "pressed once");}),
                    new TeleopStopShootingCommand(robot.getShooter())
                )
            );
        new GamepadButton(
            g2, GamepadKeys.Button.DPAD_RIGHT)
            .whenPressed(
                new SequentialCommandGroup(
                    new InstantCommand(()-> {Log.d("ManualDrive::Gamepad2:DPAD_UP", "pressed once");}),
                    new DynamicStartShooterCommand(robot.getShooter())
                )
            );

//        new GamepadButton(
//            g2, GamepadKeys.Button.DPAD_UP)
//            .whenPressed(
//                new InstantCommand(() -> {
//                    holdPosition = true;
//                    Log.d("Angle error to goal: ", "" +(Math.abs(wrapRad(compensatedAngleToGoal) - wrapRad(follower.getPose().getHeading()))));
//                })
//            );

        new GamepadButton(
            g2, GamepadKeys.Button.LEFT_BUMPER)
            .whenPressed(
                new SequentialCommandGroup(
                    new StartShooterCommand(robot.getShooter(), false),
                    new InstantCommand(()-> {
                        Log.d("ManualDrive::Gamepad2:LEFT_BUMPER: ", "pressed once");
                        Log.d("ManualDrive::Gamepad2:LEFT_BUMPER: ", "motor velocity: " + robot.getShooter().getVelocity());
                    })
                )
            );

        new GamepadButton(
            g2, GamepadKeys.Button.RIGHT_BUMPER)
            .whenPressed(
                new SequentialCommandGroup(
                    new StartShooterCommand(robot.getShooter(), true),
                    new InstantCommand(()-> {
                        Log.d("ManualDrive::Gamepad2:RIGHT_BUMPER: ", "pressed once");
                        Log.d("ManualDrive::Gamepad2:RIGHT_BUMPER: ", "motor velocity: " + robot.getShooter().getVelocity());
                    })
                )
            );
        new GamepadButton(
            g2, GamepadKeys.Button.A)
            .whenPressed(
                new SequentialCommandGroup(
                    new InstantCommand(()-> {aiming = false;}),
                    new InstantCommand(()->{
                        Log.d("ManualDrive::Gamepad2:A: ", "pressed once");
                        robot.getTurret().setForward();
                    })
                )
            );

        new GamepadButton(
            g2, GamepadKeys.Button.Y)
            .whenPressed(
                new InstantCommand(()->{
                    Log.d("ManualDrive::Gamepad2:Y: ", "pressed once");
                    aiming = true;
                })
            );

        new GamepadButton(
            g2, GamepadKeys.Button.X)
            .whenPressed(
                new SequentialCommandGroup(
                    new InstantCommand(() -> {
//                                    follower.setPose(Robot.RED_ALLIANCE ?
//                                            new Pose(72 - 8.5, -72 + 11.25, Math.toRadians(-90))
//                                            : new Pose(72 - 8.5, 72 - 11.25, Math.toRadians(90)));
//                                    follower.setPose(Robot.RED_ALLIANCE ?
//                                        new Pose(72 - 11.25, -72 + 8.5, Math.toRadians(0))
//                                        : new Pose(72 - 11.25, 72 - 8.5, Math.toRadians(0)));
//                                    follower.setPose(Robot.RED_ALLIANCE ?
//                                        new Pose(58.475, -62.84, Math.toRadians(0))
//                                        : new Pose(58.475, 62.84, Math.toRadians(0)));
//                                    follower.setPose(Robot.RED_ALLIANCE ?
//                                        new Pose(72 - 12.0, -72 + 7.0, Math.toRadians(0))
//                                        : new Pose(72 - 12.0, 72 - 7.0, Math.toRadians(0)));
                        follower.setPose(RED_ALLIANCE ?
                            new Pose(70.75 - 11.0, -70.75 + 8.0, Math.toRadians(0))
                            : new Pose(70.75 - 11.0, 70.75 - 8.0, Math.toRadians(0)));
                    }),
                    new InstantCommand(() -> {
                        Log.d("ManualDrive::Corner Relocalization: ",
                            "Current Pose: X: " + follower.getPose().getX()
                                + " Y: " + follower.getPose().getY()
                                + " Heading: " + follower.getPose().getHeading());
                    })
                )
            );
//        new GamepadButton(
//                g2, GamepadKeys.Button.X)
//                .whenPressed(
//                        new SequentialCommandGroup(
//                                new InstantCommand(()-> {aiming = false;}),
//                                new InstantCommand(()->{
//                                    Log.d("ManualDrive::Gamepad2:X: ", "pressed once");
//                                    robot.getTurret().holdPosition();
//                                })
//                        )
//                );

        new GamepadButton(
            g2, GamepadKeys.Button.B)
            .whenPressed(
                new InstantCommand(()->{
                    forceLimelightRelocalize();
                })
            );

        new GamepadButton(
            g2, GamepadKeys.Button.START)
            .whenPressed(
                new InstantCommand(() -> {
                    if (RED_ALLIANCE) {
                        RED_ALLIANCE = false;
                    } else {
                        RED_ALLIANCE = true;
                    }
                }
                )
            );
    }
    @Override
    public void run() {
        super.run();
        Log.d("ManualDrive::Shootervelocity: ", "motorvelocity: " + robot.getShooter().getVelocity());

        // Turning
        double turn = (gamepad1.left_trigger - gamepad1.right_trigger) * 0.6;

        // have driver turning override the automatic gate intake heading hold
        if (turn != 0) {
            gateIntakeHeading = false;
        }

        // driving controls
        if (gateIntakeHeading) {
            // Automatically turns the robot to have the correct gate intake heading
            Log.d("ManualDrive::gateintakeHeading: ", "gate intake heading: " + GATE_INTAKE_HEADING + " current heading: " + follower.getPose().getHeading());
            double error = wrapRad(Math.toRadians(GATE_INTAKE_HEADING) * (RED_ALLIANCE ? 1 : -1) - follower.getPose().getHeading());
            gateHeadingController.setCoefficients(FollowerConstants.headingPIDFCoefficients);
            Log.d("ManualDrive::gateintakeHeading: ", "error : " + error);
            gateHeadingController.updateError(error);
            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, gateHeadingController.runPIDF(), true);
        } /*else if (holdPosition) {
            // Automatically turns the robot to aim to the goal, used if turret isn't working
//            double error = wrapRad(compensatedAngleToGoal) - wrapRad(follower.getPose().getHeading());
//            continuousAimingController.setCoefficients(FollowerConstants.headingPIDFCoefficients);
//            continuousAimingController.updateError(error);
//            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, continuousAimingController.runPIDF(), true);
        } */else {
            // normal driving without any heading lock assistance
            Log.d("ManualDrive::setTeleopMovementVectors: ", "gateIntakeHeading: " + gateIntakeHeading + "forward: " + -gamepad1.left_stick_y + "strafe: " + -gamepad1.left_stick_x);
            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, turn, true);
        }
        follower.update();
        robot.resetCache();

        long currentTime = System.currentTimeMillis();
        telemetry.addData("Loop Rate (Hz)", 1000.0 / (double) (currentTime - lastUpdate));
        lastUpdate = currentTime;

        Log.d("ManualDrive::voltageSensing", "singularity: " + voltageSensing.readVoltage());

        if (robot.getShooter().getTargetVelocity() != 0 && Math.abs(robot.getShooter().getVelocity() - robot.getShooter().getTargetVelocity()) <= 40) {
            led.setPosition(0.444);
        } else /* Intake 3 ball detection */if (robot.getIntake().hasThreeBalls()) {
            Log.d("ManualDrive::hasThreeBalls: ", "true");
            gamepad1.rumble(250);
            gamepad2.rumble(250);

            // the name is a little misleading
            // the robot still has 3 balls at this point, but since it's done intaking it's gonna score them
            // so the next time we use the intake, the robot won't have 3 balls anymore.
//            robot.getIntake().setHasThreeBalls(false);
            led.setPosition(0.611);
            gateIntakeHeading = false;
            telemetry.addData("has 3 balls: ", robot.getIntake().hasThreeBalls());
        } else {
            led.setPosition(0.0);
        }

        telemetry.addData("Shooter velocity: ", robot.getShooter().getVelocity());

        // Current pose data
        Pose poseData = follower.getPose();
        telemetry.addData("x: ", poseData.getX());
        telemetry.addData("y: ", poseData.getY());
        telemetry.addData("r (deg): ", Math.toDegrees(poseData.getHeading()));

        // ==========================================
        // AUTO-AIMING + SHOOTING WHILE MOVING LOGIC
        // ==========================================
        telemetry.addData("Aiming Mode", aiming);

        // Toggle auto-aiming with driver 2's A button
//        if (gamepad2.left_stick_button) {
//            if (aiming) {
//                Log.d("ManualDrive:AimingMode:", " " + aiming);
//                robot.getTurret().holdPosition();
//                aiming = false;
//            } else {
//                Log.d("ManualDrive:AimingMode:", " " + aiming);
//                aiming = true;
//            }
//        }

        double goalX = RED_ALLIANCE ? RED_GOAL_X : BLUE_GOAL_X;
        double goalY = RED_ALLIANCE ? RED_GOAL_Y : BLUE_GOAL_Y;
        // Calculate angle to goal and target turret position
        AimUtil.AimResult aim = AimUtil.getAngleToGoal(poseData, goalX, goalY);
        distanceToGoal = aim.distance;
        double targetTurretAngle = Turret.turretAngleToServoDegrees(
            AimUtil.getTurretAngle(aim.angleDeg, poseData.getHeading()));

        if (System.currentTimeMillis() - lastUpdate > 50) {
            // TEST PURPOSES ONLY!
            double distanceFromAngleVelocity = 0.0;
            // Field-relative angle to goal
            if (Math.abs(prevAngleToGoal) < 0.0001) {
                prevAngleToGoal = aim.angleDeg;
            } else {
                prevAngleToGoal = angleToGoal;
                angleToGoal = aim.angleDeg;
                distanceFromAngleVelocity = (angleToGoal - prevAngleToGoal)
                    / (System.currentTimeMillis() - lastUpdate);
                telemetry.addData("prev angle to goal: ", prevAngleToGoal);
                telemetry.addData("angle to goal: ", angleToGoal);
                telemetry.update();
                Log.d("ManualDrive::updateGoalGeometryFromPose: ", "prevAngleToGoal: " + prevAngleToGoal);
                Log.d("ManualDrive::updateGoalGeometryFromPose: ", "angleToGoal: " + angleToGoal);
                Log.d("ManualDrive::updateGoalGeometryFromPose: ", "distanceFromAngleVelocity: " + distanceFromAngleVelocity);
            }
            testCompensatedAngleToGoal = angleToGoal + distanceFromAngleVelocity * angleCompensationMultiplier;
            telemetry.addData("distance from angle velocity: ", distanceFromAngleVelocity);
            telemetry.update();
            lastUpdate = System.currentTimeMillis();
        }
        // TODO: Warning! Check if above code works

        // Calculate velocity of robot
        velocity = follower.getVelocity();
        Pair<Double, Double> velocityCompensation = velocityCompensation(distanceToGoal, velocity);
        compensatedDistanceToGoal = velocityCompensation.first;
        compensatedAngleToGoal = velocityCompensation.second;
        telemetry.addData("compensated Distance to goal (deg): ", compensatedDistanceToGoal);
        telemetry.addData("compensated angle to goal (deg): ", testCompensatedAngleToGoal);
        // TODO: WARNING! Change this back if shooting while moving doesn't work
        targetTurretAngle = Turret.turretAngleToServoDegrees(
            // apparently that's 0?
            AimUtil.getTurretAngle(testCompensatedAngleToGoal, poseData.getHeading()));

        telemetry.addData("Velocity: ", velocity);
        telemetry.addData("Angle to goal (deg): ", aim.angleDeg);
        telemetry.addData("Target turret angle (deg): ", targetTurretAngle);
        telemetry.addData("Current turret angle (deg): ", robot.getTurret().getTurretAngle());

        // Apply auto-aiming if enabled
        if (aiming) {
            telemetry.addData("Status: ", "AIMING");
            // Check if target angle is within turret range
            // TODO: targetTurretAngle is the angle to the goal w/o velocity compensation
            // TODO: compensatedAngleToGoal is angle WITH velocity compensation
            // test whether compensatedAngleToGoal works
            if (targetTurretAngle >= MIN_DEGREE && targetTurretAngle <= MAX_DEGREE) {
                if (Math.abs(targetTurretAngle - prevoiusTurretAngle) > 0.5) {
                    robot.getTurret().setTurretAngle(targetTurretAngle);
                    prevoiusTurretAngle = targetTurretAngle;
                }
            } else {
                // Target is out of turret range indicate this to driver
                // note: implement something that doesn't something for this problem to solve it automatically?
                telemetry.addData("WARNING: ", "Target out of turret range!");
                gamepad1.rumble(100);
                led.setPosition(0.280);
            }
            //adjusts position of turret to target angle
        }

        if(!(targetTurretAngle >= MIN_DEGREE && targetTurretAngle <= MAX_DEGREE) && !robot.getIntake().hasThreeBalls()){
            led.setPosition(0.0);
        }

        // ==========================================
        // LIMELIGHT RELOCALIZATION
        // ==========================================
//        if (gamepad1.b) {
//            forceLimelightRelocalize();
//            telemetry.addData("STATUS: ", "Relocalizing...");
//        }

        telemetry.update();
    }

    // Uses time-of-flight recursion and robot's velocity to calculate
    // the distance from the goal and robot angle to the goal for shooting on the move
    public Pair<Double, Double> velocityCompensation(double distance, Vector velocity) {
        Pose robotPose = follower.getPose();

        double goalX = RED_ALLIANCE ? RED_GOAL_X : BLUE_GOAL_X;
        double goalY = RED_ALLIANCE ? RED_GOAL_Y : BLUE_GOAL_Y;

        double newGoalX, newGoalY;
        double newDistance = 0;
        double angleToGoalDeg = 0;
        double newTimeOfFlight;
        telemetry.addData("Robot velocity x: ", velocity.getXComponent());
        telemetry.addData("Robot velocity y: ", velocity.getYComponent());

        double initialTimeOfFlight = robot.getShooter().timeOfFlight(distance);
        //iterates so that the goal position and time of flight converges
        for (int i = 0; i < 5; i++) {
            // Treat the goal as moving relative to the robot.
            // Calculate the new goal position.
            newGoalX = (goalX - velocity.getXComponent() * initialTimeOfFlight);
            newGoalY = (goalY - velocity.getYComponent() * initialTimeOfFlight);

            // calculate the new distance
            double dx = newGoalX - robotPose.getX();
            double dy = newGoalY - robotPose.getY();

            //calculate new compensated distance
            telemetry.addData("dx: ", dx);
            telemetry.addData("dy: ", dy);
            newDistance = Math.hypot(dx, dy);

            // calculate the new angle
            angleToGoalDeg = Math.toDegrees(Math.atan2(dy, dx));
            while (angleToGoalDeg < 0) angleToGoalDeg += 360;
            while (angleToGoalDeg >= 360) angleToGoalDeg -= 360;

            //updates time of flight with new estimated distance
            newTimeOfFlight = robot.getShooter().timeOfFlight(newDistance);
            telemetry.addData("newDistance: ", newDistance);

            // check if the time of flight sufficiently converges
            //TODO: check whether a different threshold works better
            if (Math.abs(newTimeOfFlight - initialTimeOfFlight) < 0.01) {
                break;
            }

            //update initial time of flight so that the next loop uses the new estimate
            initialTimeOfFlight = newTimeOfFlight;
        }

        return new Pair<>(newDistance, angleToGoalDeg);
    }

    /**
     * Wraps an angle in radians to the range [-PI, PI].
     */
    private static double wrapRad(double r) {
        while (r > Math.PI) r -= 2.0 * Math.PI;
        while (r < -Math.PI) r += 2.0 * Math.PI;
        return r;
    }

    /**
     * Gets the robot's pose from the Limelight using MT2.
     * @return The Pose from Limelight, or null if unavailable
     */
    private Pose getLimelightPose() {
        LimelightVision ll = robot.getLimelight();
        if (ll == null) {
            Log.d("ManualDrive::Relocalize", "Limelight is null");
            return null;
        }

        if (!ll.isTargetValid()) {
            Log.d("ManualDrive::Relocalize", "Limelight target is not valid");
            return null;
        }

        // Get field position using MT2 with current robot heading
        double robotYawDeg = Math.toDegrees(wrapRad(follower.getPose().getHeading()));
        Log.d("ManualDrive::Relocalize", "Heading given to limelight: " + robotYawDeg);

        Pose3D fieldPose = ll.getFieldPositionMT2(robotYawDeg);

        if (fieldPose == null) {
            Log.d("ManualDrive::Relocalize", "Field pose is null");
            return null;
        }

        double x = fieldPose.getPosition().x;
        double y = fieldPose.getPosition().y;
        double headingDegrees = fieldPose.getOrientation().getYaw(AngleUnit.DEGREES);

        // Limelight returns position in meters, convert to inches
        // note: may need to adjust coordinate system mapping?
        Log.d("ManualDrive::Relocalize", "Returning new pose: (" + x * 39.3700787 + ", " + y * 39.3700787 + ", " + headingDegrees + ")");

        return new Pose(x * 39.3700787, y * 39.3700787, follower.getPose().getHeading());
    }
    //
//    /**
//     * Forces a Limelight relocalization updating the follower's pose.
//     */
    private void forceLimelightRelocalize() {
        Pose poseNow = follower.getPose();

        Pose llPose = null;
        try {
            llPose = getLimelightPose();
        } catch (Exception e) {
            Log.d("ManualDrive::Relocalize", "Exception while reading limelight pose: " + e.getMessage());
        }

        if (llPose == null) {
            lastPoseForStillCheck = poseNow;
            Log.d("ManualDrive::Relocalize", "No limelight pose available");
            return;
        }

        // Log the pose information
        Log.d("ManualDrive::Relocalize", "llPose: X: " + llPose.getX() + " Y: " + llPose.getY() + " Heading: " + llPose.getHeading());
        telemetry.addLine("llPose: X: " + llPose.getX() + " Y: " + llPose.getY() + " Heading: " + Math.toDegrees(llPose.getHeading()));
        telemetry.addLine("poseNow: X: " + poseNow.getX() + " Y: " + poseNow.getY() + " Heading: " + Math.toDegrees(poseNow.getHeading()));

        // Update follower pose with Limelight data
        follower.setPose(llPose);
        Log.d("ManualDrive::Relocalize", "Accepted limelight pose. x=" + llPose.getX() + " y=" + llPose.getY() + " h=" + llPose.getHeading());

        telemetry.addData("Relocalized", "Success!");

        // Rumble to confirm relocalization
        gamepad1.rumble(250);
        gamepad2.rumble(250);
    }

    /**
     * Wraps an angle in degrees to the range [0, 360).
     */
    private static double wrapAngle(double r) {
        while (r >= 360) r -= 360;
        while (r < 0) r += 360;
        return r;
    }

    // Use custom camera vision algorithm to relocalize
    private void cameraRelocalize() {
        List<VisionLocalizer.Detection> detections = new ArrayList<>();
        // Center is roughly (640, 360)
        Point[] corners = {
            new Point(590, 310), new Point(690, 310),
            new Point(690, 410), new Point(590, 410)
        };
        detections.add(new VisionLocalizer.Detection(RED_ALLIANCE ? 20 : 24, corners));

        double imuYaw = follower.getPose().getHeading(); // Facing straight ahead (Field Relative)
        VisionLocalizer.VisionPose3D robotPose = localizer.estimateCameraPoseConstrained(detections, imuYaw);

        Pose relocalizedPose = new Pose(robotPose.x, robotPose.y, imuYaw);

        if (relocalizedPose != null) {
            System.out.println("Robot Position Estimated Successfully!");
            Log.d("ManualDrive::Relocalize", "Accepted limelight pose. x=" + relocalizedPose.getX() + " y=" + relocalizedPose.getY() + " h=" + relocalizedPose.getHeading()
                + " CurrentPose: X: " + relocalizedPose.getX() + " Y: " + relocalizedPose.getY() + " Heading: " + relocalizedPose.getHeading());
        } else {
            System.out.println("Failed to estimate robot position.");
        }
    }
}
