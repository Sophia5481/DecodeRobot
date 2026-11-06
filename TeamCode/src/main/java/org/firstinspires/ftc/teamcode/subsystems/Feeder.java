//package org.firstinspires.ftc.teamcode.subsystems;
//
//import android.util.Log;
//
//import com.arcrobotics.ftclib.command.SubsystemBase;
//import com.arcrobotics.ftclib.hardware.ServoEx;
//import com.arcrobotics.ftclib.hardware.SimpleServo;
//import com.qualcomm.robotcore.hardware.AnalogInput;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.teamcode.Robot;
//
//public class Feeder extends SubsystemBase {
//    private final ServoEx greenServo;
//    private final ServoEx purpleServo;
//    private final ServoEx shovel;
//    private final DistanceSensor distanceSensor;
//
////    private final DistanceSensor distanceSensor;
//
////    private AnalogInput laserAnalog;
//
//    // Sensor scale: 3.3V corresponds to 1000 mm
//    private static final double MAX_VOLTS = 3.3;
//    private static final double MAX_DISTANCE_MM = 1000.0;
//
//    private final static double GREEN_MIN_ANGLE = 0.0;
//    // TODO: This value if not tuned can cause shooting failures!
//    // If the latch is opened too much, then it can't keep the ball centered in the shovel
//    // and when the robot tries to shoot, the ball gets pushed to the side and stuck on the latch!
//    private final static double GREEN_CLOSED = 54.0;
//    private final static double GREEN_OPEN = 39.0;
//    private final static double GREEN_MAX_ANGLE = 90.0;
//
//    private final static double PURPLE_MIN_ANGLE = 0.0;
//    private final static double PURPLE_CLOSED = 45.0;
//    private final static double PURPLE_OPEN = 56.0;
//    private final static double PURPLE_MAX_ANGLE = 90.0;
//
//    private final static double SHOVEL_MIN_ANGLE = 0.0;
//    private final static double SHOVEL_DOWN = 40;
//    // this angle allows balls from transfer to get into the shovel and contact the shooter wheel at the same time
//    private final static double SHOVEL_RAPID_FIRE = 45.0;
//
//    // sometimes the first ball doesn't shoot correctly, so the shovel needs to raise a lil bit higher
//    // however, the second and third balls will be fine, so then the shovel should move back to
//    // the SHOVEL_RAPID_FIRE position.
//    private final static double SHOVEL_FIRST_RAPID_FIRE = 46.5;
//
//    // this angle reliably brings the ball in contact with the shooter wheel
//    private final static double SHOVEL_UP = 47.0;
//    private final static double SHOVEL_MAX_ANGLE = 90.0;
////    ElapsedTime timer;
//
//    public Feeder(HardwareMap hMap) {
//        greenServo = new SimpleServo(hMap, "latchRight", GREEN_MIN_ANGLE, GREEN_MAX_ANGLE);
//        purpleServo = new SimpleServo(hMap, "latchLeft", PURPLE_MIN_ANGLE, PURPLE_MAX_ANGLE);
//        shovel = new SimpleServo(hMap, "shovel", SHOVEL_MIN_ANGLE, SHOVEL_MAX_ANGLE);
//        distanceSensor = hMap.get(DistanceSensor.class, "feederSensor");
//
////        laserAnalog = hMap.get(AnalogInput.class, "laserAnalogInput");
//    }
//
//    public void init(boolean auto) {
//        greenServo.turnToAngle(GREEN_CLOSED);
//        if (auto) {
//            if (Robot.AUTO_NOSORTING) {
//                purpleServo.turnToAngle(PURPLE_OPEN);
//            }
//            else {
//                purpleServo.turnToAngle(PURPLE_CLOSED);
//            }
//        } else {
//            purpleServo.turnToAngle(PURPLE_CLOSED /*PURPLE_OPEN*/);
//        }
////        purpleServo.turnToAngle(PURPLE_CLOSED);
////        if (auto) {
////            greenServo.turnToAngle(GREEN_CLOSED);
////        } else {
////            greenServo.turnToAngle(GREEN_OPEN);
////        }
//        shovel.turnToAngle(SHOVEL_DOWN);
//    }
//
//    //feed one green artifact up to shooter
//    public void feedGreenArtifact() {
//        greenServo.turnToAngle(GREEN_OPEN);
//    }
//
//    public void closeGreenFeeder() {
//        greenServo.turnToAngle(GREEN_CLOSED);
//    }
//
//    //feed one purple artifact up to shooter
//    public void feedPurpleArtifact() {
//        purpleServo.turnToAngle(PURPLE_OPEN);
//    }
//
//    public void closePurpleFeeder() {
//        purpleServo.turnToAngle(PURPLE_CLOSED);
//    }
//
//    public void setShovelUp() {
//        shovel.turnToAngle(SHOVEL_UP);
//    }
//
//    public void shovelRapidFire() {
//        shovel.turnToAngle(SHOVEL_RAPID_FIRE);
//    }
//
//    public void shovelFirstRapidFire() {
//        shovel.turnToAngle(SHOVEL_FIRST_RAPID_FIRE);
//    }
//
//    public void setShovelDown() {
//        shovel.turnToAngle(SHOVEL_DOWN);
//    }
//
//    public boolean shooterFull() {
//        double distance = distanceSensor.getDistance(DistanceUnit.INCH);
//        Log.d("ShootThreeCommand: Feeder::shooterFull ", "" + distance + " " + (distance < 3.4));
//        return distance < 3.0;
//
////        // Read sensor voltage (0.0–3.3V)
////        double volts = laserAnalog.getVoltage();
////
////        // Convert voltage to distance in millimeters (linear mapping)
////        double distance = (volts / MAX_VOLTS) * MAX_DISTANCE_MM / 25.4;
////        Log.d("ShootThreeCommand: Feeder::shooterFull ", "" + distance + " " + (distance < 4.0));
////        return distance < 4.0;
//    }
//}
