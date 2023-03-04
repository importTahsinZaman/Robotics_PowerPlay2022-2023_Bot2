package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConstants {
    public static final double LIFTPOSITIONCOEFFICIENT = 0.009;
    public static final int LIFTPOSITIONTOLERANCE = 35;

    public static final int BOTTOMLIFTPOSITION = 15; //Not included in preset heights
    public static final int GROUNDJUNCTIONPOSITION = 50;
    public static final int LOWJUNCTIONPOSITION = 100;
    public static final int MIDJUNCTIONPOSITION = 300;
    public static final int HIGHJUNCTIONPOSITION = 4050;
    public static final int TOPLIFTPOSITION = 4100; //Not included in preset heights

    public static final double LEFTSERVOOPENPOSITION = .68;
    public static final double LEFTSERVOCLOSEPOSITION = .47;

    public static final double RIGHTSERVOOPENPOSITION = .32;
    public static final double RIGHTSERVOCLOSEPOSITION = .53;

    public static final int LIFTMULTIPLIER = 30; //The multiplier on gamepad trigger values for manual lift movement
}
