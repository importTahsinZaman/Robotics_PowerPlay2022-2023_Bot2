package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public class AutonConstants {
    public static int STRAFE_TO_ZONE2_DISTANCE = 30;

    public static int ZONE2_TO_ZONE1_DISTANCE = 26; //inches
    public static int ZONE2_TO_ZONE3_DISTANCE = 26; //inches

    public static double WAIT_AT_JUNCTION_TIME = 0.3; //Seconds
    public static double WAIT_AT_ZONE2_TIME = 0.3; //Seconds

    //Blue Left
    public static Pose2d BLUE_LEFT_START_POSITION = new Pose2d(36, -60, Math.toRadians(-180));
    public static Vector2d BLUE_LEFT_JUNCTION_POSITION = new Vector2d(34, 0);
    public static Pose2d BLUE_LEFT_SIDE_STACK_POSITION = new Pose2d(36, -60, Math.toRadians(-180)); //Not set
    public static Vector2d BLUE_LEFT_ZONE2_POSITION = new Vector2d(-30, -38);

    //Blue Right
    public static Pose2d BLUE_RIGHT_START_POSITION = new Pose2d(-36, 60, Math.toRadians(0));
    public static Vector2d BLUE_RIGHT_JUNCTION_POSITION = new Vector2d(-33.8, 0);
    public static Pose2d BLUE_RIGHT_SIDE_STACK_POSITION = new Pose2d(36, -60, Math.toRadians(-180)); //Not set
    public static Vector2d BLUE_RIGHT_ZONE2_POSITION = new Vector2d(-30, -38);

    //Red Left
    public static Pose2d RED_LEFT_START_POSITION = new Pose2d(-30, -70, Math.toRadians(0));
    public static Vector2d RED_LEFT_JUNCTION_POSITION = new Vector2d(36, -63);
    public static Pose2d RED_LEFT_SIDE_STACK_POSITION = new Pose2d(36, -60, Math.toRadians(-180)); //Not set
    public static Vector2d RED_LEFT_ZONE2_POSITION = new Vector2d(-30, -38);

    //Red Right
    public static Pose2d RED_RIGHT_START_POSITION = new Pose2d(30, -60, Math.toRadians(-180));
    public static Vector2d RED_RIGHT_JUNCTION_POSITION = new Vector2d(30, -2);
    public static Pose2d RED_RIGHT_SIDE_STACK_POSITION = new Pose2d(36, -60, Math.toRadians(-180)); //Not set
    public static Vector2d RED_RIGHT_ZONE2_POSITION = new Vector2d(30, -38);
}
