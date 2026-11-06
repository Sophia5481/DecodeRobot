package org.firstinspires.ftc.teamcode.opmodes.OldAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name = "Red Near 12 Ball Auto 🏖️", group = "Auto")
public class Red12BallAuto extends Auto12BallNearBase {

    @Override
    protected boolean isRedAlliance() {
        return true;
    }

//    @Override
//    protected void setupAllianceSpecifics() {
//        // Points definitions from original file
////        startHeading = Math.toRadians(180.0 - 52.0);
//        start = new Point(58.06, -46.05, Point.CARTESIAN);
//        end = new Point(24.5, -32.0, Point.CARTESIAN);
//        endHeading = Math.toRadians(180.0 - 90.0);
//
//        scoring = new Point(24.5, -12.0, Point.CARTESIAN);
//        cycleScoring = new Point(24.5, -12.0, Point.CARTESIAN);
//        scoringHeading = Math.toRadians(180.0 - 43.0);
//        cycleScoringHeading = Math.toRadians(180 - 45.0);
//
//        intakeHeading = Math.toRadians(180.0 + 90.0);
//        intake1 = new Point(20.0 - 24.0 - 1.0, -20.0, Point.CARTESIAN);
//        intake2 = new Point(20.0, -16.0, Point.CARTESIAN);
//        intake3 = new Point(20.0 - 24.0 * 2, -21.5, Point.CARTESIAN);
//
//        // open gate points
//        gatePoint = new Point(20.0 - 24.0 - 1.0 + 3.0, -(22.0 + 31.5 - 10.0), Point.CARTESIAN);
//        gatePoint2 = new Point(20.0 - 24.0 - 1.0 + 7.0, -(22.0 + 31.5 - 12.0), Point.CARTESIAN);
//        gatePoint3 = new Point(20.0 - 24.0 - 1.0 + 10.0, -(22.0 + 31.5 - 6.0), Point.CARTESIAN);
//        scoringControlPoint = new Point(20.0 - 24.0 - 1.0, -18.0, Point.CARTESIAN);
//
//        gateOpeningHeading = 180.0 + 20.0;
//    }
//
//    @Override
//    protected double getIntakeYIncrement(int cycle) {
//        // Original logic: cycle == 1 || cycle == 3 ? -31.5 : -29
//        return cycle == 1 || cycle == 3 ? -31.5 : -29;
//    }
}