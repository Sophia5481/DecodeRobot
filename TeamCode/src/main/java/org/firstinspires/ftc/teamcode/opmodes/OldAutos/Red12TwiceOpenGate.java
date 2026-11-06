package org.firstinspires.ftc.teamcode.opmodes.OldAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name = "Red Near 12 Twice Open Gate🚪❗", group = "Auto")
public class Red12TwiceOpenGate extends Auto12NearTwiceOpenBase {

    @Override
    protected boolean isRedAlliance() {
        return true;
    }

    @Override
    protected double getIntakeYIncrement(int cycle) {
        return cycle == 1 || cycle == 2 ? -28.5 : -26;
    }

    @Override
    protected double getGatePoint2YIncrement(int cycle) {
        return cycle == 2 ? 3.0 : 0.0;
    }
}