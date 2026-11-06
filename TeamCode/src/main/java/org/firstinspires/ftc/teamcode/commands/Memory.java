package org.firstinspires.ftc.teamcode.commands;

import android.util.Pair;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;

public class Memory {
    // stores variables from auto that are used in teleop
    public static boolean ENDED_AUTO = false;

    public static Pose autoEndPose = new Pose(/*-48.0*/ - 72.0 + 17.5/2, /*48.0*/ 63, Math.toRadians(90));

    public static String motif = "PGP";

    // These variables are updated in ManualDrive and are used for shooting while moving
    public static double distanceToGoal;
    public static double compensatedDistanceToGoal;

    public static double compensatedAngleToGoal;

    public static Vector velocity;
    public static Vector acceleration;

    public static double accelerationDouble;
}
