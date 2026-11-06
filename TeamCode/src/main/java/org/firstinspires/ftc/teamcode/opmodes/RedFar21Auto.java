package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Far 21 Auto: 1 SPIKE, 5 HP 🦞️️", group = "Auto")
public class RedFar21Auto extends Auto21FarBase {

    @Override
    protected boolean isRedAlliance() {
        return true;
    }
}
