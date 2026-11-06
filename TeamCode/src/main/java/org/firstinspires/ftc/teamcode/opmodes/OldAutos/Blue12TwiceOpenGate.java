package org.firstinspires.ftc.teamcode.opmodes.OldAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name = "Blue Near 12 Twice Open Gate🚪🐬", group = "Auto")
public class Blue12TwiceOpenGate extends Auto12NearTwiceOpenBase {

    @Override
    protected boolean isRedAlliance() {
        return false;
    }


    @Override
    protected double getIntakeYIncrement(int cycle) {
        return cycle == 1 || cycle == 2 ? 28.5 : 26;
    }

    @Override
    protected double getGatePoint2YIncrement(int cycle) {
        return cycle == 2 ? -3.0 : 0.0;
    }
}