package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Math.abs;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.commands.Memory;
import org.firstinspires.ftc.teamcode.commands.VoltageSensing;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;

import com.arcrobotics.ftclib.util.InterpLUT;

@Config
public class Shooter extends SubsystemBase {
    private final MotorEx motor;
    private final MotorEx motor2;
    private final ServoEx lid; //adjustable lid

    private VoltageSensing voltageSensing;

    // whether shooter motors spin during periodic function
    private boolean enabled = false;

    private enum SHOOTING_MODE {
        NEAR, FAR, DYNAMIC
    }

    SHOOTING_MODE mode = SHOOTING_MODE.NEAR;

    // tune on dashboard :)
    // CHANGED FROM 1120 DUE TO NEW RAPID FIRE SHOOT WAYS
    // TODO: FIND NEW POWER DUE TO INTAKE BOOSTING SHOOTING POWER
    //TODO: FIND DESIRED DIRECTION FOR MOTORS
    public static double NEAR_SHOOTING_POWER = 1080; // from the close launch zone

    public static double NEAR_IDLE_SHOOTING_POWER = 1160;

    public static double MEDIUM_SHOOTING_POWER = 1340;

    public static double FAR_SHOOTING_POWER = 1400; // from the far launch zone
    public static double FAR_IDLE_SHOOTING_POWER = 1420;

    public static int FAR_AUTO_SHOOTING_POWER = 2560;
    private final static double LID_MIN_ANGLE = 0.0;

    // look-up table of distance vs. shooter power
    InterpLUT shooterPowerLut;
    InterpLUT lidPowerLut;

    // TODO: TUNE LID POSITIONS
    // TODO: Add back lid code once it starts working
    public static double LID_NEAR = 30.0; //changed for autos, used to be 20.0
    public static double LID_MEDIUM = 80.0;
    public static double LID_FAR = 75.0 /*46.0*/;
    public static double LID_MAX_ANGLE = 280.0;

    private final static double LID_ROTATE = 1.0;

    private double previous_lid = 0.0;

    // This is set to true once all balls have been shot
    // This should always be set to false in the init function of StartShooterCommand
    private boolean stopShooting = false;

    // TODO: Temporary testing!!!
    public static double kP = 0.004; /* 0.004 */ /*0.003*/
    public static double kI = 0.0;
    public static double kD = 0.00005;
    public static double kS = /* 0.0752141 0.072496 /* 0.0134004 */0.0139836/*0.00571348 /*0.0565076 /*0.0683723 /*0.0350483/*0.0170949/*0.0317261 /*0.0254536 */;
    public static double kV = /*0.000378611 /* 0.000370647 /* 0.000396351 */ 0.000412688 /* 0.000419694 /*0.000374306 /*0.000388617 /*0.000376634/*0.000403674 /*0.000400193 */;
    public static double kA = 0.0;
    private PIDFController pidf;

    private long lastTime = 0;
    private double lastVelocity = 0.0;
    private double targetVelocity = 0.0;
    private double lidCompensationFactor = 3.0;
    private double farLidAngle = 81.0;

    private double NOMINAL_VOLTAGE = 12.0;

    // Once the motor velocity reaches the target velocity, the correctVelocityCount is incremented
    // Once this is 3 then we can shoot the ball

    // Distance: 50. Power: 980. Lid angle: 10      Time: 0.515 secs (0.44 + 0.516 + 0.59)/3
    // Distance: 67. Power: 1040. Lid angle: 20     Time: 0.59 secs (0.64 + 0.6 + 0.53)/3
    // Distance: 83.7. Power: 1120. Lid angle: 40   Time: 0.746 (0.81 + 0.73 + 0.7)/3
    // Distance: 96.1. Power: 1220. Lid angle: 60.  Time: 0.75 (0.75 + 0.82 + 0.68)/3
    // Distance: 109.4. Power: 1240 Lid angle: 75   Time: 0.846 (0.79 + 0.88 + 0.87)/3
    // Distance: 128.3. Power: 1440 Lid angle: 110  Time: 0.8167 (0.79 + 0.8 + 0.86)/3
    // Distance: 144.5. Power: 1480 Lid angle: 110  Time: 0.833 (0.8 + 0.84 + 0.86)/3


    // New hoods
    // TODO: Update time
    // Distance: 50. Power: 980. Lid angle: 10      Time: 0.515 secs (0.44 + 0.516 + 0.59)/3
    // Distance: 67. Power: 1060. Lid angle: 20     Time: 0.59 secs (0.64 + 0.6 + 0.53)/3
    // Distance: 83.7. Power: 1140. Lid angle: 40   Time: 0.746 (0.81 + 0.73 + 0.7)/3
    // Distance: 96.1. Power: 1220 / 1240. Lid angle: 60.  Time: 0.75 (0.75 + 0.82 + 0.68)/3
    // Distance: 109.4. Power: 1240 Lid angle: 75   Time: 0.846 (0.79 + 0.88 + 0.87)/3
    // Distance: 128.3. Power: 1420 Lid angle: 80  Time: 0.8167 (0.79 + 0.8 + 0.86)/3
    // Distance: 144.5. Power: 1460 Lid angle: 85  Time: 0.833 (0.8 + 0.84 + 0.86)/3

    // New belt shooter
    // TODO: Update time
    // Distance: 50. Power: 980. Lid angle: 10      Time: 0.515 secs (0.44 + 0.516 + 0.59)/3
    // Distance: 67. Power: 1060. Lid angle: 20     Time: 0.59 secs (0.64 + 0.6 + 0.53)/3
    // Distance: 83.7. Power: 1140. Lid angle: 40   Time: 0.746 (0.81 + 0.73 + 0.7)/3
    // Distance: 96.1. Power: 1220 / 1240. Lid angle: 60.  Time: 0.75 (0.75 + 0.82 + 0.68)/3
    // Distance: 109.4. Power: 1240 Lid angle: 75   Time: 0.846 (0.79 + 0.88 + 0.87)/3
    // Distance: 128.3. Power: 1420 Lid angle: 110  Time: 0.8167 (0.79 + 0.8 + 0.86)/3
    // Distance: 144.5. Power: 1460 Lid angle: 115  Time: 0.833 (0.8 + 0.84 + 0.86)/3

    // Belt shooter w/ looser tension
    // TODO: Update time
    // Distance: 50. Power: 980. Lid angle: 10      Time: 0.515 secs (0.44 + 0.516 + 0.59)/3
    // Distance: 67. Power: 1080. Lid angle: 20     Time: 0.59 secs (0.64 + 0.6 + 0.53)/3
    // Distance: 83.7. Power: 1140. Lid angle: 40   Time: 0.746 (0.81 + 0.73 + 0.7)/3
    // Distance: 96.1. Power: 1200. Lid angle: 60.  Time: 0.75 (0.75 + 0.82 + 0.68)/3
    // Distance: 109.4. Power: 1300 Lid angle: 67   Time: 0.846 (0.79 + 0.88 + 0.87)/3
    // Distance: 128.3. Power: 1460 Lid angle: 110  Time: 0.8167 (0.79 + 0.8 + 0.86)/3
    // Distance: 144.5. Power: 1500 Lid angle: 115  Time: 0.833 (0.8 + 0.84 + 0.86)/3

    // Revert back to chain
    // TODO: Update time
    // Distance: 36.5 Power: 960 Lid Angle: 0        Time: need to tune ! ! !
    // Distance: 50. Power: 1020. Lid angle: 10      Time: 0.515 secs (0.44 + 0.516 + 0.59)/3
    // Distance: 67. Power: 1100. Lid angle: 20     Time: 0.59 secs (0.64 + 0.6 + 0.53)/3
    // Distance: 83.7. Power: 1140. Lid angle: 40   Time: 0.746 (0.81 + 0.73 + 0.7)/3
    // Distance: 96.1. Power: 1220. Lid angle: 55.  Time: 0.75 (0.75 + 0.82 + 0.68)/3
    // Distance: 109.4. Power: 1240 Lid angle: 70   Time: 0.846 (0.79 + 0.88 + 0.87)/3
    // Distance: 128.3. Power: 1400 Lid angle: 75  Time: 0.8167 (0.79 + 0.8 + 0.86)/3
    // Distance: 144.5. Power: 1480 Lid angle: 80  Time: 0.833 (0.8 + 0.84 + 0.86)/3

    //retuning chain shooter (need to tune)
    // Distance: 36.5 Power: 960 Lid Angle: 0   Time:
    // Distance: 50. Power: 1020. Lid angle: 10   Time:
    // Distance: 67. Power: 1100. Lid angle: 20   Time:
    // Distance: 82.9, Power: 1180, Lid angle: 30   Time:
    // Distance: 83.7. Power: 1140. Lid angle: 40   Time:
    // Distance: 96.1. Power: 1220. Lid angle: 55.   Time:
    // Near: velocity = 0.0000454983x^{4}-0.0118585x^{3}+1.10853x^{2}-39.48181x+1420.2418
    // Near: hoodAngle = y=-0.00000254325x^{4}+0.000973085x^{3}-0.114874x^{2}+6.02439x-109.67563
    //// far
    // Distance: 121.5, Power: 1380 Lid angle: 76  Time:
    // Distance: 127.3  Power: 1420  Lid angle: 77   Time:
    // Distance: 132.5 Power: 1440 Lid angle: 80   Time:
    // Distance: 137.2 Power: 1480 Lid angle: 83  Time:
    // Distance: 144 Power: 1500  Lid angle: 82   Time: 0.863 secs
    // Distance: 145.5 Power: 1520  Lid angle: 86  Time:
    // Far: velocity = 0.000268737x^{4}-0.142375x^{3}+28.21024x^{2}-2471.90611x+82072.9071
    // Far: hoodAngle = 81 //TODO: tune

    //retuning chain shooter (4/23)
    // Distance: 36.5 Power: 960 Lid Angle: 0
    // Distance: 50. Power: 1020. Lid angle: 10 Time: (0.55 + 0.66 + 0.56)/3 = 0.59
    // Distance: 67.3. Power: 1120. Lid angle: 21 Time: (0.62 + 0.61)/2 = 0.615
    // Distance: 83.7. Power: 1160. Lid angle: 40 Time:
    // Distance: 96.1. Power: 1220. Lid angle: 55. Time:
    // Distance: 121.5, Power: 1380 Lid angle: 76 Time:
    // Distance: 127.3  Power: 1420  Lid angle: 78 Time:
    // Distance: 132.5 Power: 1440 Lid angle: 80 Time:
    // Distance: 137.2 Power: 1480 Lid angle: 83 Time:
    // Distance: 145.5 Power: 1520  Lid angle: 86 Time:
    // Power: y=-0.00000809761x^{4}+0.00307829x^{3}-0.400865x^{2}+25.68798x+417.019
    // Lid Angle: y=0.000000471513x^{4}-0.000261425x^{3}+0.0447912x^{2}-2.03525x+27.25824
    private int correctVelocityCount = 0;

    //done for figuring out hood position based on velocity
    //TODO: add more points to this array
    private static final double[] VELOCITY_POINTS = {
        960, 1020, 1100, 1140, 1220, 1240, 1420, 1480,
    };

    private static final double[] HOOD_ANGLE_POINTS = {
        0, 10, 20, 40, 55, 70, 75, 80
    };

    public Shooter(final HardwareMap hMap) {
        motor = new MotorEx(hMap, "shooter");
        motor2 = new MotorEx(hMap, "motorTop");
        motor.setRunMode(Motor.RunMode.RawPower);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        // TODO: determine if motor2 needs to be reversed
        motor2.setRunMode(Motor.RunMode.RawPower);
        motor2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        motor.setInverted(false);
        motor2.setInverted(false);
        motor.encoder.setDirection(MotorEx.Direction.REVERSE);
        lid = new SimpleServo(hMap, "hoodServo", LID_MIN_ANGLE, LID_MAX_ANGLE);
        pidf = new PIDFController(kP, kI, kD, 0);
        stopShooting = false;
        previous_lid = 0.0;
    }

    // TODO: Remove these logs once I don't need them anymore!!!!!!!!!!
    public void periodic() {
        if (!enabled) {
            return;
        }
        switch (mode) {
            case NEAR:
                shootArtifactNearPower();
                lid.turnToAngle(LID_NEAR);
                break;
            case FAR:
                shootArtifactFarPower();
                lid.turnToAngle(LID_FAR);
                break;
            case DYNAMIC:
                // TODO: Warning! If smth with shooting goes wrong, check if it's smth to do with shooting while moving
                shootArtifactAdjustablePower(Memory.compensatedDistanceToGoal /*Memory.distanceToGoal*/);
                break;
        }
    }

    public void init() {
        motor.set(0.0);
        motor2.set(0.0);
        lid.turnToAngle(LID_FAR);
    }

    // adjustable shooting power to shoot from anywhere in field
    public void shootArtifactAdjustablePower(double distance) {
//        double power = shooterPowerLut.get(distance);
//        double angle = lidPowerLut.get(distance);
        double power = calculateShooterPower(distance);
        double angle = calculateLidPower(distance);
//        angle *= (getVelocity() / targetVelocity) * lidCompensationFactor;
        double adjusted_angle = MathFunctions.clamp(angle, 0, 130);
        if (abs(adjusted_angle - previous_lid) > 0.01)
        {
            lid.turnToAngle(adjusted_angle);
            previous_lid = adjusted_angle;
        }

        Log.d("Shooter::shootArtifactAdjustablePower: ", "Before rounding: " + power);
        power = nearestMultipleOf20((int) power);
        setVelocity(power);
        Log.d("Shooter::shootArtifactAdjustablePower: ", "After rounding: " + power);
    }

    //todo: get a new name 😭
    public void shootArtifactDynamicPower(double distance, boolean far) {
        double power, angle;
        if(far){
            power = calculateFarShooterPower(distance);
            angle = calculateFarLidPower(distance);
        }else{
            power = calculateNearShooterPower(distance);
            angle = calculateNearLidAngle(distance);
        }

//        angle *= (getVelocity() / targetVelocity) * lidCompensationFactor * angle;
        double adjusted_angle = MathFunctions.clamp(angle, 0, 130);
        if (abs(adjusted_angle - previous_lid) > 0.01)
        {
            lid.turnToAngle(adjusted_angle);
            previous_lid = adjusted_angle;
        }

        Log.d("Shooter::shootArtifactDynamicPower: ", "Before rounding: " + power);
        power = nearestMultipleOf20((int) power);
        setVelocity(power);
        Log.d("Shooter::shootArtifactDynamicPower: ", "After rounding: " + power);
    }

    // calculates the time it takes for the ball to enter the goal
    // based on the robot distance and velocity relative to the goal
    public double timeOfFlight(double distance) {
        double x = distance;
        // the logistic equations was gotten from a Desmos regression of distance v. time of flight
//        return (0.858095)/(1 + Math.pow(2.718, -(0.0397648 * distance - 1.662)));
        return (-0.323229) /
            (1 + Math.pow(2.718, 0.102793 * distance - 8.38917)) + 0.893429;
    }

    //default shooting power
    // no time constraint -- will be controlled with waitCommands() instead
    public void shootArtifactNearPower() {
        setVelocity(NEAR_SHOOTING_POWER);
        Log.d("Shooter::shootArtifactNearPower: ", "" + NEAR_SHOOTING_POWER);
    }

    public void shootArtifactMediumPower() {
        setVelocity(MEDIUM_SHOOTING_POWER);
        Log.d("Shooter::shootArtifactMediumPower: ", "" + MEDIUM_SHOOTING_POWER);
    }

    public void shootArtifactFarPower() {
        setVelocity(FAR_SHOOTING_POWER);
        Log.d("Shooter::shootArtifactFarPower: ", "" + FAR_SHOOTING_POWER);
    }

    public void shooterIdleNearVelocity() {
        setVelocity(NEAR_IDLE_SHOOTING_POWER);
        Log.d("Shooter::shooterIdleNearVelocity: ", "" + NEAR_IDLE_SHOOTING_POWER);
    }

    public void shooterIdleFarVelocity() {
        setVelocity(FAR_IDLE_SHOOTING_POWER);
        Log.d("Shooter::shooterIdleFarVelocity: ", "" + FAR_IDLE_SHOOTING_POWER);
    }

    public void shooterOff() {
        setVelocity(500);
    }

    public void shooterOffTele() {
        motor.set(0.0);
        motor2.set(0.0);
        enabled = false;
    }

    public void shooterNegative() {
        motor.set(-0.3);
        motor.set(-0.3);
    }

    //set lid to specific angle from vision method
    public void setLidAngle(double angle) {
        lid.turnToAngle(angle);
    }
    public double getLidAngle(){return lid.getAngle();}

    // turn the lid to shoot from the near position
    public void lidNear() {
        lid.turnToAngle(LID_NEAR);
    }

    public void lidMid() {
        lid.turnToAngle(LID_MEDIUM);
    }

    // turn the lid to shoot from the far position
    public void lidFar() {
        lid.turnToAngle(LID_FAR);
    }

    //increment lid angle by set degrees higher
    public void lidHigher() {
        lid.rotateBy(LID_ROTATE);
    }

    //incremenet lid angle by set degrees lower
    public void lidLower() {
        lid.rotateBy(-LID_ROTATE);
    }

    /**
     * Call this in your loop to update motor power based on target velocity
     * @param targetVelocity target in ticks per second
     */
    public void setVelocity(double targetVelocity) {
        this.targetVelocity = targetVelocity;
        double currentVelocity = motor.getVelocity();
        Log.d("Shooter::setVelocity", "target velocity: " + targetVelocity);
        Log.d("Shooter::setVelocity::motor_velocity", "current_velocity:" + currentVelocity);
        Log.d("Shooter::setVelocity", "voltage: " + voltageSensing.readVoltage());

        double dt = 0.001;  // default small value

        long now = System.nanoTime();
        if (lastTime != 0) {
            dt = (now - lastTime) / 1e9;
        }
        // temporary workaround,
        // set acceleration to 0 since we do not use kA at all
        double acceleration = 0.0;
        lastVelocity = currentVelocity;
        lastTime = now;

        // PID output
//        double pidOutput = pidf.calculate(currentVelocity, this.targetVelocity);
        double pidOutput = (this.targetVelocity - currentVelocity) * kP;

        if (pidOutput < 0.05 && currentVelocity < targetVelocity) {
            pidOutput = 0.05;
        }

        // Feedforward
        double ff = kS * Math.signum(this.targetVelocity) + kV * (this.targetVelocity) + kA * acceleration;
        ff *= (NOMINAL_VOLTAGE) / (voltageSensing.readVoltage());
        double power = pidOutput + ff;

        // Clamp power between specified value to prevent it from initially going to a too high speed
        double maxPower = 1.0;
        power = Math.max(-maxPower, Math.min(maxPower, power));

        motor.set(power);
        motor2.set(power);
        Log.d("Shooter::setVelocity", "target_velocity: " + targetVelocity +
            " current_velocity: " + currentVelocity +
            " pidOutput: " + pidOutput +
            " feedForward: " +  ff +
            " power: " + power);
        Log.d("ShootThreeCommand::setVelocity", "set power: " + power);
    }

    public double getVelocity() {
        double velocity = motor.getVelocity();
        // failsafe: if one motor encoder cable is unplugged, then use the other one.
//        if (velocity == 0) {
//            return motor2.getVelocity();
//        }
//        Log.d("motor velocity: ", "" + velocity /* + "motor 2 velocity: " + motor2.getVelocity() */);
        return velocity;
    }

    // TODO: This error margin is tight; it takes a considerable amount of time for the shooter to get within this error margin
    // If the tolerance is instead 20, then the shooter will be more inconsistent and risk shooting balls out the goal
    // Work on ways to get it reliable without having to wait for so long, may need a two-motor shooter.
    public boolean isShooterReady() {
        boolean isReady = abs(getVelocity() - targetVelocity) < 10;
        //boolean isReady = (Math.abs(getVelocity() - targetVelocity) < 10);
        Log.d("Shooter::isShooterReady ", "" + isReady);
//        return correctVelocityCount >= 3;
        return isReady;
    }

    public boolean teleIsShooterReading() {
        boolean isReady = abs(getVelocity() - targetVelocity) < 30;
        //boolean isReady = (Math.abs(getVelocity() - targetVelocity) < 10);
        Log.d("Shooter::isShooterReady ", "" + isReady);
//        return correctVelocityCount >= 3;
        return isReady;
    }

    public void setStopShooting(boolean config) {
        stopShooting = config;
    }

    public boolean getStopShooting() {
        return stopShooting;
    }

    public void resetVelocityCount() {
        correctVelocityCount = 0;
    }

    public void stopMotor() {
        motor.set(0);
        motor2.set(0);
    }

    //DYNAMIC SHOOTING LOGIC
    // use a regression to calculate shooter power based on distance
    private double calculateShooterPower(double distance) {
//        return 5.92352 * Math.pow(10, -7) * Math.pow(distance, 4) - 0.0000206631 * Math.pow(distance, 3)
//            - 0.0189864 * Math.pow(distance, 2) + 6.49696 * distance + 745.92233;
        return -0.0000108187 * Math.pow(distance, 4) + 0.00406683 * Math.pow(distance, 3)
            - 0.527027 * Math.pow(distance, 2) + 32.20504 * distance + 302.38847;
    }

    private double calculateLidPower(double distance) {
//        return 5.19509 * Math.pow(10, -7) * Math.pow(distance, 4) - 0.000313641 * Math.pow(distance, 3)
//            + 0.0551793 * Math.pow(distance, 2) - 2.73186 * distance + 41.5961;
        return 0.00000135997 * Math.pow(distance, 4) - 0.000601653 * Math.pow(distance, 3)
            + 0.0889588 * Math.pow(distance, 2) - 4.28205 * distance + 65.47308;
    }

    private double calculateNearShooterPower(double distance){
        // Near: velocity = 0.0000454983x^{4}-0.0118585x^{3}+1.10853x^{2}-39.48181x+1420.2418
        return clamp(300.0, 1200.0, 0.0000454983 * Math.pow(distance, 4) - 0.0118585 * Math.pow(distance, 3)
            + 1.10853 * Math.pow(distance, 2) - 39.48181 * distance + 1420.2418);
    }
    // Near: hoodAngle = y=-0.00000254325x^{4}+0.000973085x^{3}-0.114874x^{2}+6.02439x-109.67563
    private double calculateNearLidAngle(double distance){
        return clamp(0.0, 60.0, -0.00000254325 * Math.pow(distance, 4) + 0.000973085 * Math.pow(distance, 3)
            - 0.114874 * Math.pow(distance, 2) + 6.02439 * distance - 109.67563);
    }
    private double calculateFarShooterPower(double distance){
        // Far: velocity = 0.000268737x^{4}-0.142375x^{3}+28.21024x^{2}-2471.90611x+82072.9071
        return clamp(1200, 1600, 0.000268737 * Math.pow(distance, 4) - 0.142375 * Math.pow(distance, 3)
            + 28.21024 * Math.pow(distance, 2) - 2471.90611 * distance + 82072.9071);
    }
    private double calculateFarLidPower(double distance){
        return farLidAngle; //81, TODO: tune
    }
    public static double hoodAngleFromVelocity(double flywheelVelocity) {
        // Clamp below range (if velocity is below 960)
        if (flywheelVelocity <= VELOCITY_POINTS[0]) {
            return clampAngle(HOOD_ANGLE_POINTS[0]);
        }

        // Clamp above range (if velocity is above 1480)
        int lastPosition = VELOCITY_POINTS.length - 1;
        if (flywheelVelocity >= VELOCITY_POINTS[lastPosition]) {
            return clampAngle(HOOD_ANGLE_POINTS[lastPosition]);
        }

        // Find the interval and interpolate
        //finds which points the velocity is between (i.e. between 1420 and 1480)
        for (int i = 0; i < lastPosition; i++) {
            //velocity is between floor and ceiling velocities
            double floorVelocity = VELOCITY_POINTS[i];
            double ceilingVelocity = VELOCITY_POINTS[i + 1];

            if (flywheelVelocity >= floorVelocity && flywheelVelocity <= ceilingVelocity) {
                //angle is between floor and ceiling angles
                double floorHoodAngle = HOOD_ANGLE_POINTS[i];
                double ceilingHoodAngle = HOOD_ANGLE_POINTS[i + 1];

                //t tells you how far between the two points you are
                double t = (flywheelVelocity - floorVelocity) / (ceilingVelocity - floorVelocity);

                //final angle
                double angle = floorHoodAngle + t * (ceilingHoodAngle - floorHoodAngle);

                //ensures that the angle is between 0 and 90
                return clampAngle(angle);
            }
        }

        // Should never get here, but just in case. returns smallest hood angle
        return clampAngle(HOOD_ANGLE_POINTS[0]);
    }
    private static double clamp(double min, double max, double input){
        return Math.max(min, Math.min(max, input));
    }
    private static double clampAngle(double angleDeg) {
        return Math.max(0.0, Math.min(90.0, angleDeg));
    }

    // shooter velocities are always a multiple of 20 for some reason
    private int nearestMultipleOf20(int number) {
        int lowerBound = (number / 20) * 20;
        int upperBound = lowerBound + 20;
        if (abs(upperBound - number) < abs(lowerBound - number)) {
            return upperBound;
        } else {
            return lowerBound;
        }
    }

    public void setShooterEnabled(boolean set) {
        enabled = set;
    }

    public void setShooterFar(boolean set) {
        if (set) {
            mode = SHOOTING_MODE.FAR;
        } else {
            mode = SHOOTING_MODE.NEAR;
        }
    }

    public void enableDynamic(boolean set) {
        if (set) {
            mode = SHOOTING_MODE.DYNAMIC;
        }
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }
    /*
     * Start shooter motor is too fast (inconsistently fast though)
     * Ball doesn't shoot, so need to wait until timeout occurs
     * Gate doesn't open
     * Shooter ends prematurely, slows to an idle speed
     * Figure out left trigger and right trigger
     */
}
