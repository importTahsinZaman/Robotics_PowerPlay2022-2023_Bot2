package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConstants {
    public static  double LIFT_POSITION_COEFFICIENT = 0.009;
    public static  int LIFT_POSITION_TOLERANCE = 35;

    public static  int BOTTOM_LIFT_POSITION = 15; //Not included in preset heights
    public static  int GROUND_JUNCTION_POSITION = 50;
    public static  int LOW_JUNCTION_POSITION = 100;
    public static  int MID_JUNCTION_POSITION = 300;
    public static  int HIGH_JUNCTION_POSITION = 4050;
    public static  int TOP_LIFT_POSITION = 4100; //Not included in preset heights

    public static  double LEFT_SERVO_OPEN_POSITION = .68;
    public static  double LEFT_SERVO_CLOSE_POSITION = .47;

    public static  double RIGHT_SERVO_OPEN_POSITION = .32;
    public static  double RIGHT_SERVO_CLOSE_POSITION = .53;

    public static  int LIFT_MULTIPLIER = 30; //The multiplier on gamepad trigger values for manual lift movement
}
