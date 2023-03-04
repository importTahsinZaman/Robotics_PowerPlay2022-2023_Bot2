package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public class AutonConstants {
    public static int ZONE2_TO_ZONE1_DISTANCE = 25; //inches
    public static int ZONE2_TO_ZONE3_DISTANCE = 25; //inches

    public static double WAIT_AT_JUNCTION_TIME = 0.3; //Seconds
    public static double WAIT_AT_ZONE2_TIME = 0.3; //Seconds

    //Blue Left
    public static Pose2d BLUE_LEFT_START_POSITION = new Pose2d(36, -60, Math.toRadians(-180));
    public static Vector2d BLUE_LEFT_JUNCTION_POSITION = new Vector2d(34, 0);
    public static Pose2d BLUE_LEFT_SIDE_STACK_POSITION = new Pose2d(36, -60, Math.toRadians(-180)); //Not set
    public static Vector2d BLUE_LEFT_ZONE1_POSITION = new Vector2d(36, 35);
    public static Pose2d BLUE_LEFT_ZONE2_POSITION = new Pose2d(36, -60, Math.toRadians(-180)); //Not set
    public static Pose2d BLUE_LEFT_ZONE3_POSITION = new Pose2d(36, -60, Math.toRadians(-180)); //Not set

    //Blue Right
    public static Pose2d BLUE_RIGHT_START_POSITION = new Pose2d(-36, 60, Math.toRadians(0));
    public static Vector2d BLUE_RIGHT_JUNCTION_POSITION = new Vector2d(-33.8, 0);
    public static Pose2d BLUE_RIGHT_SIDE_STACK_POSITION = new Pose2d(36, -60, Math.toRadians(-180)); //Not set
    public static Vector2d BLUE_RIGHT_ZONE1_POSITION = new Vector2d(-35.6, 35.3);
    public static Pose2d BLUE_RIGHT_ZONE2_POSITION = new Pose2d(36, -60, Math.toRadians(-180)); //Not set
    public static Pose2d BLUE_RIGHT_ZONE3_POSITION = new Pose2d(36, -60, Math.toRadians(-180)); //Not set

    //Red Left
    public static Pose2d RED_LEFT_START_POSITION = new Pose2d(-35, -60, Math.toRadians(0));
    public static Vector2d RED_LEFT_JUNCTION_POSITION = new Vector2d(-33.8, 0);
    public static Pose2d RED_LEFT_SIDE_STACK_POSITION = new Pose2d(36, -60, Math.toRadians(-180)); //Not set
    public static Vector2d RED_LEFT_ZONE1_POSITION = new Vector2d(-35, -35);
    public static Pose2d RED_LEFT_ZONE2_POSITION = new Pose2d(36, -60, Math.toRadians(-180)); //Not set
    public static Pose2d RED_LEFT_ZONE3_POSITION = new Pose2d(36, -60, Math.toRadians(-180)); //Not set

    //Red Right
    public static Pose2d RED_RIGHT_START_POSITION = new Pose2d(36, -60, Math.toRadians(-180));
    public static Vector2d RED_RIGHT_JUNCTION_POSITION = new Vector2d(34, 0);
    public static Pose2d RED_RIGHT_SIDE_STACK_POSITION = new Pose2d(36, -60, Math.toRadians(-180)); //Not set
    public static Vector2d RED_RIGHT_ZONE1_POSITION = new Vector2d(36, -35);
    public static Pose2d RED_RIGHT_ZONE2_POSITION = new Pose2d(36, -60, Math.toRadians(-180)); //Not set
    public static Pose2d RED_RIGHT_ZONE3_POSITION = new Pose2d(36, -60, Math.toRadians(-180)); //Not set
}
