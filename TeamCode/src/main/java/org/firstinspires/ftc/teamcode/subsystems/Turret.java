package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;

public class Turret extends SubsystemBase {
    public final SimpleServo turretRight;
    public final SimpleServo turretLeft;
//    public final SimpleServo turretBack;
//    private final AnalogInput turretEncoder;

    public static double kP = 0.01;

    public static boolean USE_LIMITS = true;
    // Turret rotation limits: ±165 degrees from center (0 = aligned with robot heading)
    // Increasing the angle makes turret turn counterclockwise
    // Decreasing the angle makes turret turn clockwise
    // In servo terms: 165 is center, 0 is -165 deg, 330 is +165 deg
    public static double MIN_DEGREE = 0;       // -157.5 degrees from robot heading
    public static double CENTER_DEGREE = 195;   // 0 degrees (aligned with robot heading)
    public static double FACING_FORWARD_DEGREE = 15;
    public static double MAX_DEGREE = 320;      // +157.5 degrees from robot heading

    public static double ENCODER_MAX_VOLTAGE = 3.3;
    
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

    // angle
    // 285: 92 degrees counterclockwise
    // 195: straight
    // 230: 38 counterclockwise
    // 260: 65.5 counterclockwise

    public Turret(HardwareMap hMap) {
        turretRight = new SimpleServo(hMap, "turretRight", MIN_DEGREE, MAX_DEGREE);
        turretLeft = new SimpleServo(hMap, "turretLeft", MIN_DEGREE, MAX_DEGREE);
//        turretBack = new SimpleServo(hMap, "turretBack", MIN_DEGREE, MAX_DEGREE);
//        turretEncoder = hMap.get(AnalogInput.class, "turretEncoder");
    }

    //returns absolute angle
//    public double getAbsoluteEncoderAngle() {
//        return (turretEncoder.getVoltage() / ENCODER_MAX_VOLTAGE) * 360.0;
//    }

    //ensures that the target reaches its desired position
    public void ensureTargetPosition() {
//        double currentDegree = getAbsoluteEncoderAngle();
//        while (Math.abs(currentDegree - targetDegree) > POS_TOLERANCE_DEGREE){
//            if (currentDegree > targetDegree) {
//                setTargetDegree(currentDegree - POS_TOLERANCE_DEGREE);
//            }
//            else {
//                setTargetDegree(currentDegree + POS_TOLERANCE_DEGREE);
//            }
//        }
//        Log.d("Turret", "EncoderPosition: " + currentDegree + ", Target: " + targetDegree);

    }

    public void setForward(){
        setTargetDegree(CENTER_DEGREE);
        holdPosition();
    }
    //initial position should be facing backwards
    public void init(boolean auto) {
        if(!auto){
            setTargetDegree(TELEOP_INIT_DEGREE);}
        else{
            setTargetDegree(AUTO_INIT_DEGREE);}
    }
    public void hitTargetPosition(double targetDegree){
        double currentTurretAngle = getTurretAngle();
        while(Math.abs(targetDegree - currentTurretAngle) > POS_TOLERANCE_DEGREE){
            if(targetDegree-currentTurretAngle > POS_TOLERANCE_DEGREE){
                setTargetDegree(currentTurretAngle + POS_TOLERANCE_DEGREE);
            }
            else{
                setTargetDegree(currentTurretAngle - POS_TOLERANCE_DEGREE);
            }
        }
    }
    public void turn(double turnAmount){
        holdPosition = false;
        double currentAngle = turretRight.getAngle();
        double targetAngle = currentAngle + turnAmount;
        setTargetDegree(targetAngle);
        Log.d("turret", "turnAmount = " + turnAmount + ", target = " + targetAngle);
    }

    public void goToLoad(boolean redAlliance) {
        holdPosition = false;
        if (redAlliance) {
            targetDegree = LOAD_RED_DEGREE;
            setTargetDegree(targetDegree);
        }
        else {
            targetDegree = LOAD_BLUE_DEGREE;
            setTargetDegree(targetDegree);
        }
        Log.d("turret", "target = " + targetDegree);
    }

    public void goToNearShot(boolean redAlliance) {
        holdPosition = false;
        if (redAlliance) {
            targetDegree = NEAR_RED_SHOOTING_DEGREE;
            setTargetDegree(targetDegree);
        }
        else {
            targetDegree = NEAR_BLUE_SHOOTING_DEGREE;
            setTargetDegree(targetDegree);
        }
        Log.d("turret", "target = " + targetDegree);
    }

    public void goToFarShot(boolean redAlliance) {
        if (redAlliance) {
            targetDegree = FAR_RED_SHOOTING_DEGREE;
            setTargetDegree(targetDegree);
        }
        else {
            targetDegree = FAR_BLUE_SHOOTING_DEGREE;
            setTargetDegree(targetDegree);
        }
        Log.d("Turret::goToFarShot", "target=" + targetDegree);
    }

    //clamped to limits if enabled
    public void setTargetDegree(double ticks) {
        if (USE_LIMITS) ticks = clamp(ticks);
        targetDegree = ticks;
        turretRight.turnToAngle(targetDegree  - 4.0);
        turretLeft.turnToAngle(targetDegree + 4.0 );
//        turretBack.turnToAngle(targetDegree);
        Log.d("TURRET::setTar getDegree", "target = " + targetDegree);
    }

    public void holdCurrentPosition() {
        setTargetDegree(getCurrentTicks());
    }

    /**
     * Enable hold position mode, turret maintains current position.
     */
    public void holdPosition() {
        holdPosition = true;
        holdCurrentPosition();
    }

    //stop output to motor
    public void disable() {
        holdPosition = false;
        turretLeft.disable();
        turretRight.disable();
//        turretBack.disable();
    }

    public boolean isEnabled() {
        return holdPosition;
    }
    
    /**
     * Convert the turret-relative angle to servo degrees, 0 = robot heading
     * @param turretAngle angle relative to the robot (-157.5 to +157.5 degrees, 0 = forward)
     * @return The servo angle in degrees from 0 to 315
     */
    public static double turretAngleToServoDegrees(double turretAngle) {
        return wrapDegree(FACING_FORWARD_DEGREE + turretAngle); //Forward facing degree = 15, MIN_DEGREE = 0, MAX_DEGREE = 315
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
        setTargetDegree(turretAngle);
        Log.d("Turret::setTurretAngle", "turretAngle=" + turretAngle);
    }
    
    /**
     * Get the current turret angle relative to the robot heading.
     * @return the angle in degrees -157.5 to 157.5
     */
    public double getTurretAngle() {
        return servoDegreesToTurretAngle(getCurrentTicks());
    }

    public double getTargetDegree() {
        return targetDegree;
    }

    public double getCurrentTicks() {
        return turretRight.getAngle();
    }

    public boolean atTarget() {
        return Math.abs(getCurrentTicks() - targetDegree) <= POS_TOLERANCE_DEGREE;
    }

    @Override
    public void periodic() {
//        double current = getCurrentTicks();
//        if(current < MIN_DEGREE || current > MAX_DEGREE) {
//            setTargetDegree(clamp(current));
//        }
    }

    //if v is too big then use hi, if it's too low then use lo
    private static double clamp(double v) {
        return Math.max(MIN_DEGREE, Math.min(MAX_DEGREE, v));
    }

    private static int clampInt(int v, int lo, int hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static double wrapDegree(double r) {
        if (r < 0) {
            r += 360;
        } else if (r > 360) {
            r -= 360;
        }
        return r;
    }
}
