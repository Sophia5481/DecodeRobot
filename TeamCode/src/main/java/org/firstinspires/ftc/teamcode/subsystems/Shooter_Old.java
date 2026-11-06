//package org.firstinspires.ftc.teamcode.subsystems;
//
//import android.util.Log;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.arcrobotics.ftclib.command.SubsystemBase;
//import com.arcrobotics.ftclib.hardware.ServoEx;
//import com.arcrobotics.ftclib.hardware.SimpleServo;
//import com.arcrobotics.ftclib.hardware.motors.Motor;
//import com.arcrobotics.ftclib.hardware.motors.MotorEx;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//@Config
//public class Shooter_Old extends SubsystemBase {
//    private final MotorEx motor;
//    private final ServoEx lid; //adjustable lid
//
//    private final static double DEFAULT_SHOOTING_POWER = 0.6; //default shooting power
//    private final static double LID_MIN_ANGLE = 0.0;
//    private final static double LID_MAX_ANGLE = 90.0;
//
//    private final static double LID_ROTATE = 1.0;
//    public static boolean shooterFull = false;
//
//    public static double kP = 0.001;
//    public final static double kI = 0.0;
//    public final static double kD = 0.0;
//    public final static double kS = 0.096507;
//    public final static double kV = 0.00042;
//    public final static double kA = 0.0;
//
//    public Shooter(final HardwareMap hMap) {
//        motor = new MotorEx(hMap, "shooter");
//        // motor.setRunMode(Motor.RunMode.RawPower);
//        motor.setRunMode(Motor.RunMode.VelocityControl);
//        motor.setVeloCoefficients(kP, kI, kD);
//        motor.setFeedforwardCoefficients(kS, kV, kA);
//        lid = new SimpleServo(hMap, "adjustableLid", LID_MIN_ANGLE, LID_MAX_ANGLE);
//    }
//
//    public void init() {
//        motor.set(0.0);
//        lid.turnToAngle(15.0);
//    }
//
//    // inputtable motor power from vision method
//    public void shootArtifactWithPower(double shooting_power) {
//        motor.set(shooting_power);
//        shooterFull = false;
//    }
//
//    //default shooting power
//    // no time constraint -- will be controlled with waitCommands() instead
//    public void shootArtifactDefaultPower() {
//        motor.set(DEFAULT_SHOOTING_POWER);
//        Log.d("motor velocity", "" + motor.getVelocity());
//    }
//
//    public void shooterOff() {
//        motor.set(0);
//    }
//
//    //set lid to specific angle from vision method
//    public void setLidAngle(double angle) {
//        lid.turnToAngle(angle);
//    }
//
//    //increment lid angle by set degrees higher
//    public void lidHigher() {
//        lid.rotateBy(LID_ROTATE);
//    }
//
//    //incremenet lid angle by set degrees lower
//    public void lidLower() {
//        lid.rotateBy(-LID_ROTATE);
//    }
//
//    public void setRunMode(Motor.RunMode mode) {
//        motor.setRunMode(mode);
//    }
//
//    public void setVelocity(double velocity) {
//        motor.setVelocity(velocity);
//    }
//
//    public void set(double velocity) {
//        motor.set(velocity);
//    }
//
//    public double getVelocity() {
//        return motor.getVelocity();
//    }
//
//    public MotorEx getMotor() {
//        return motor;
//    }
//}
