package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import android.util.Log;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Intake extends SubsystemBase {
//    private final MotorEx intakeRight;
    private final MotorEx intakeLeft;
    private final MotorEx intakeRight;
    private final SimpleServo latch;

    private final SimpleServo deploy;
    private final static double INTAKE_FULL = 1.0;
    private final static double INTAKE_REVERSE = -0.6;
    private final static double INTAKE_ON = 0.9;
    private final static double INTAKE_SLOW = 0.85;
    private final static double INTAKE_REVERSE_SLOW = -0.30;
    private final static double INTAKE_OFF = 0.0;
    private final static double LATCH_MIN_DEGREE = 0.0; //open
    private final static double LATCH_MAX_DEGREE = 360; //closed
    private final static double LATCH_OPEN = 195;
    private final static double LATCH_CLOSED = 245;

    private final static double DEPLOY_REST_DEGREE = 58.0; //is up
    private final static double DEPLOY_ACTIVE_DEGREE = 66.0; //active

    private static boolean isIntaking = false;

    // sensors for detecting balls in the intake
    private AnalogInput firstBallDetector;
    private AnalogInput secondBallDetector;
    private AnalogInput secondBallDetector2;
    private AnalogInput thirdBallDetector;

    // Sensor scale: 3.3V corresponds to 1000 mm
    private static final double MAX_VOLTS = 3.3;
    private static final double MAX_DISTANCE_MM = 1000.0;

    private boolean hasThreeBalls = false;

    public Intake(HardwareMap hMap) {
        intakeLeft = new MotorEx(hMap, "intakeLeft"); //negative if intaking
        intakeLeft.setRunMode(Motor.RunMode.RawPower);
        intakeLeft.setInverted(true);
        intakeRight = new MotorEx(hMap, "intakeRight"); //positive if intaking
        intakeRight.setRunMode(Motor.RunMode.RawPower);
        intakeRight.setInverted(false);
        latch = new SimpleServo(hMap, "latch", LATCH_MIN_DEGREE,LATCH_MAX_DEGREE);
        deploy = new SimpleServo(hMap, "deploy", 0, 90);
        firstBallDetector = hMap.get(AnalogInput.class, "firstAnalogSensor");
        secondBallDetector = hMap.get(AnalogInput.class, "secondAnalogSensor");
        secondBallDetector2 = hMap.get(AnalogInput.class, "secondAnalogSensor2");
        thirdBallDetector = hMap.get(AnalogInput.class, "thirdAnalogSensor");
    }
    public void init() {
        isIntaking = false;
        stopIntake();
    }
    public boolean isIntaking(){
        return isIntaking;
    }
    public void slowIntake(){
        isIntaking = true;
        deployIntake();
        closeLatch();
        intakeRight.set(INTAKE_SLOW);
        intakeLeft.set(INTAKE_SLOW);
    }
    public void deployRest(){deploy.turnToAngle(DEPLOY_REST_DEGREE);}

    public void deployIntake(){deploy.turnToAngle(DEPLOY_ACTIVE_DEGREE);}
    public void fullIntake(){
        isIntaking = true;
        deployIntake();
        closeLatch();
        intakeRight.set(INTAKE_FULL);
        intakeLeft.set(INTAKE_FULL);
    }
    public void reverseSlowIntake(){
        isIntaking = true;
        deployIntake();
        closeLatch();
        intakeRight.set(INTAKE_REVERSE_SLOW);
        intakeLeft.set(INTAKE_REVERSE_SLOW);
    }
    public void startIntake() {
        isIntaking = true;
        deployIntake();
        closeLatch();
        intakeRight.set(INTAKE_ON);
        intakeLeft.set(INTAKE_ON);
    }
    public void reverseIntake(){
        isIntaking = true;
        deployIntake();
        closeLatch();
        intakeRight.set(INTAKE_REVERSE);
        intakeLeft.set(INTAKE_REVERSE);
    }
    
    public void stopIntake(){
        isIntaking = false;
        closeLatch();
        intakeRight.set(INTAKE_OFF);
        intakeLeft.set(INTAKE_OFF);
    }

    public void stopTransfer() {
        closeLatch();
        intakeLeft.set(INTAKE_OFF);
    }

    public void closeLatch(){latch.turnToAngle(LATCH_CLOSED);}
    public void openLatch(){latch.turnToAngle(LATCH_OPEN);}

    // Code for detecting 3 balls
    public boolean hasFirstArtifact() {
        double volts = firstBallDetector.getVoltage();

        // Convert voltage to distance in millimeters (linear mapping)
        double distanceMM = (volts / MAX_VOLTS) * MAX_DISTANCE_MM;
        Log.d("Distance sensor 1st ball: ", "distance: " + distanceMM);
        return distanceMM < 90.0;
    }

    public boolean hasSecondArtifact() {
        double volts = secondBallDetector.getVoltage();

        // Convert voltage to distance in millimeters (linear mapping)
        double distanceMM = (volts / MAX_VOLTS) * MAX_DISTANCE_MM;
        Log.d("Distance sensor 2nd ball: ", "distance: " + distanceMM);

        double volts2 = secondBallDetector2.getVoltage();

        // Convert voltage to distance in millimeters (linear mapping)
        double distanceMM2 = (volts2 / MAX_VOLTS) * MAX_DISTANCE_MM;
        Log.d("Distance sensor 2nd ball 2: ", "distance: " + distanceMM2);
        return (distanceMM < 70.0) || (distanceMM2 < 70.0);
//        return (distanceMM < 70.0);
    }

    public boolean hasThirdArtifact() {
        double volts = thirdBallDetector.getVoltage();

        // Convert voltage to distance in millimeters (linear mapping)
        double distanceMM = (volts / MAX_VOLTS) * MAX_DISTANCE_MM;
        Log.d("Distance sensor 3nd ball: ", "distance: " + distanceMM);
        return distanceMM < 70.0;
    }

    public void setHasThreeBalls(boolean set) {
        hasThreeBalls = set;
    }

    public boolean hasThreeBalls() {
        return hasThreeBalls;
    }

    public double getTransferVelocity() {
        double velocity = intakeLeft.getVelocity();
        Log.d("Intake", "transferVelocity:" + velocity);
        return velocity;
    }
}
