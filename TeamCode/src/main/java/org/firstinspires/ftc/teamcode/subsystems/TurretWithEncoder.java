package org.firstinspires.ftc.teamcode.subsystems;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import static java.lang.Thread.sleep;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

@Config
public class TurretWithEncoder extends SubsystemBase {
    public final SimpleServo turretRight;
    public final SimpleServo turretLeft;
    public final SimpleServo turretBack;

    private final MotorEx encoder;

    double offset = 0;

    public static double kP = 0.01;
    public static double kD = 0.01;
    public static double kI = 0.01;
    public static double MIN_CORRECTION = 2.0;
    private double integral = 0;

    public static boolean USE_LIMITS = true;
    // Turret rotation limits: ±165 degrees from center (0 = aligned with robot heading)
    // In servo terms: 165 is center, 0 is -165 deg, 330 is +165 deg
    public static double MIN_DEGREE = 0;       // -157.5 degrees from robot heading
    public static double CENTER_DEGREE = 195;   // 0 degrees (aligned with robot heading)
    public static double MAX_DEGREE = 315;      // +157.5 degrees from robot heading

    // Teleop starts with turret aligned with robot heading
    public static double TELEOP_INIT_DEGREE = CENTER_DEGREE;
    public static double AUTO_INIT_DEGREE = CENTER_DEGREE;

    public static int LOAD_RED_DEGREE = 55; //shooting pos after getting balls from hp in loading zone
    public static int NEAR_RED_SHOOTING_DEGREE = 280; //shooting pos from far side facing forward
    public static int FAR_RED_SHOOTING_DEGREE = 55; //shooting while facing red loading zone

    public static double LOAD_BLUE_DEGREE = 306.95;
    public static int NEAR_BLUE_SHOOTING_DEGREE = 80;
    public static double FAR_BLUE_SHOOTING_DEGREE = 306.95; //shooting while facing towards blue loading zone

    public static int POS_TOLERANCE_DEGREE = 5;

    private double targetDegree = 0;
    private boolean holdPosition = false;

    private double lastError = 0;
    private double lastTime = 0;

    private double correction;
    private double newAngle;
    public static double USE_PID_ERROR_RANGE = 8.0;

    //private Encoder overflowEncoder;

    public TurretWithEncoder(HardwareMap hMap) {
        turretRight = new SimpleServo(hMap, "turretRight", MIN_DEGREE, MAX_DEGREE);
        turretLeft = new SimpleServo(hMap, "turretLeft", MIN_DEGREE, MAX_DEGREE);
        turretBack = new SimpleServo(hMap, "turretBack", MIN_DEGREE, MAX_DEGREE);
        encoder = new MotorEx(hMap, "intakeRight");
        //overflowEncoder = new Encoder(hMap.get(DcMotorEx.class, "intakeRight"));
//        overflowEncoder.getDirection().

    }
    //turning right, turretLeft: (-), turretRight: (-)
    //turning left, turretLeft: (+), turretRight: (+)
    //note: positions between servos might differ, will need to be tuned later on
    //facing back is 0.5, facing leftFront is 0.0, facing rightFront is 1.0, facing front is currently out of range

    @Override
    public void periodic() {
        update(targetDegree);
    }

    //initial position should be facing backwards
    public void init(boolean auto) {
        if(!auto){
            setTargetDegree(TELEOP_INIT_DEGREE);}
        else{
            setTargetDegree(AUTO_INIT_DEGREE);}

        // wait 200 ms for the turret to turn to its centered position,
        // then apply an offset to the encoder.

//        long time = System.currentTimeMillis();
//        while ((System.currentTimeMillis() - time) < 500) {
//            Log.d("Turret Encoder::", "waiting" + Log.d("Turret Encoder::", "waiting " + System.currentTimeMillis()););
//        }

        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        resetEncoder();
    }

    public void resetEncoder() {
        double rawTicks = encoder.getCurrentPosition();
        double initialAngleReading = -(rawTicks * 360 / 8192) * 2 / 9;
        offset = CENTER_DEGREE - initialAngleReading;
        Log.d("Turret Encoder::offset", "" + offset);
    }

    // TODO: convert encoder ticks to degrees.
    public double encoderTicksToDegrees(double ticks) {
        // first convert ticks to degrees
        // then convert encoder degrees to the actual turret angle (90:20 gear ratio)
        // then add the offset
        double degrees = -(ticks * 360 / 8192) * 2 / 9 + offset;
        Log.d("Turret Encoder::ticks", "" + ticks);
        Log.d("Turret Encoder::degrees", "" + degrees);
        return degrees;
    }

    public void turn(double turnAmount){
        holdPosition = false;
        double currentAngle = turretRight.getAngle();
        double targetAngle = currentAngle + turnAmount;
        update(targetAngle);
        Log.d("turret", "turnAmount = " + turnAmount + ", target = " + targetAngle);
    }

    //clamped to limits if enabled
    public void update(double degree) {
        if (USE_LIMITS) degree = clamp(degree);
        targetDegree = degree;
        double error = targetDegree - getTurretAngle();
        integral += error;
        integral = Math.max(-50, Math.min(50, integral));
//        double dt = 0.001;
//        if (lastTime != 0) {
//            dt = (System.currentTimeMillis() - lastTime) / 1e9;
//        }
        correction = error * kP + integral * kI/*+ (error - lastError) / dt * kD*/;
        if (Math.abs(correction) < MIN_CORRECTION && Math.abs(correction) > 0.5) {
            correction = MIN_CORRECTION * Math.signum(correction);
        }

        if (Math.abs(error) > USE_PID_ERROR_RANGE) {
            correction = 0;
        }
        Log.d("Turret Encoder::", "correction: " + correction + "Proportional: " + error * kP + "Integral: " + integral + " " + integral * kI);
//        double currentServoPos = (turretRight.getAngle() + turretLeft.getAngle() + turretBack.getAngle())/3;
        newAngle = Range.clip(targetDegree + correction, 0, 315);
        turretRight.turnToAngle(newAngle);
        turretLeft.turnToAngle(newAngle);
        turretBack.turnToAngle(newAngle);
        Log.d("TURRET::setTargetDegree", "target = " + targetDegree);
    }

    // set turret angle WITHOUT using PID
    public void setTargetDegree(double ticks) {
        if (USE_LIMITS) ticks = clamp(ticks);
        targetDegree = ticks;
        turretRight.turnToAngle(targetDegree);
        turretLeft.turnToAngle(targetDegree);
        turretBack.turnToAngle(targetDegree);
        Log.d("TURRET::setTargetDegree", "target = " + targetDegree);
    }

    /**
     * Convert the turret-relative angle to servo degrees, 0 = robot heading
     * @param turretAngle angle relative to the robot (-157.5 to +157.5 degrees, 0 = forward)
     * @return The servo angle in degrees from 0 to 315
     */
    public static double turretAngleToServoDegrees(double turretAngle) {
        return CENTER_DEGREE + turretAngle; //CENTER_DEGREE = 157.5, MIN_DEGREE = 0, MAX_DEGREE = 325
    }

    /**
     * Convert servo degrees to turret-relative angle.
     * @param servoDegrees the servo angle 0 to 315
     * @return the angle relative to the robot
     * basically opposite of what's above
     */
    public static double servoDegreesToTurretAngle(double servoDegrees) {
        return servoDegrees - CENTER_DEGREE;
    }

    /**
     * Set the turret to a specific angle relative to the robot heading.
     * @param turretAngle the angle in degrees -157.5 to 157.5
     */
    public void setTurretAngle(double turretAngle) {
        holdPosition = false;
        double servoDegrees = turretAngleToServoDegrees(turretAngle);
        update(servoDegrees);
        Log.d("Turret::setTurretAngle", "turretAngle=" + turretAngle + " servoDegrees=" + servoDegrees);
    }

    /**
     * Get the current turret angle relative to the robot heading.
     * @return the angle in degrees -157.5 to 157.5
     */
    public double getTurretAngle() {
        return encoderTicksToDegrees(encoder.getCurrentPosition());
    }

    public double getCurrentTicks() {
        return turretRight.getAngle();
    }
    //if v is too big then use hi, if it's too low then use lo
    private static double clamp(double v) {
        return Math.max(MIN_DEGREE, Math.min(MAX_DEGREE, v));
    }

    public double getNewAngle(){
        return newAngle;
    }

    public double getCorrection() {
        return correction;
    }
}
