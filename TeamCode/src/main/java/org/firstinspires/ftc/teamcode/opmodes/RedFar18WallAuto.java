package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name = "Red Far 15 Wall Auto HP ONLY 🐢❤️", group = "Auto")
public class RedFar18WallAuto extends Auto18FarWallBase {

    @Override
    protected boolean isRedAlliance() {
        return true;
    }
}