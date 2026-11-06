//package org.firstinspires.ftc.teamcode.opmodes.OldAutos
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point
//
//@Autonomous(name = "Red Near Auto 🧧", group = "Auto")
//class Near9MotifRedAuto : Near9MotifAutoBase() {
//
//    // After converting to pedro coordinate system, add 72.0 to both x and y
//    // The center of the field is now (72, 72), not (0, 0)
//    override val start = Point(58.06, -46.05, Point.CARTESIAN)
//    override val startHeading = Math.toRadians(180.0 - 52.0)
//    override val end = Point(24.5, -32.0, Point.CARTESIAN)
//    override val endHeading = Math.toRadians(180.0 - 90.0)
//
//    // The initial scoring position is adjusted for drift
//    override val scoring = Point(24.5, -12.0, Point.CARTESIAN)
//    // The scoring point adjusted for drift after intaking
//    override val cycleScoring = Point(24.5, -12.0, Point.CARTESIAN)
//    override val scoringHeading = Math.toRadians(180.0 - 43.0)
//    override val cycleScoringHeading = Math.toRadians(180.0 - 45.0)
//
//    override val intakeHeading = Math.toRadians(180.0 + 90.0)
//    override val intake1 = Point(20.0, -18.0, Point.CARTESIAN)
//    override val intake2 = Point(20.0 - 24.0 - 1.0, -22.0, Point.CARTESIAN)
//    override val intake3 = Point(20.0 - 24.0 * 2 + 2.0, -18.0, Point.CARTESIAN)
//    override val scoringControlPoint = Point(20.0 - 24.0 - 1.0, -18.0, Point.CARTESIAN)
//
//    override fun intakeIncrement() = -30.5
//    override fun intakeIncrementSmall() = -28.0
//    override fun isRedAlliance() = true
//}
