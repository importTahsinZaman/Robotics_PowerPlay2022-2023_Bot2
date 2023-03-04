/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.AutonConstants.BLUE_LEFT_JUNCTION_POSITION;
import static org.firstinspires.ftc.teamcode.AutonConstants.BLUE_LEFT_START_POSITION;
import static org.firstinspires.ftc.teamcode.AutonConstants.BLUE_LEFT_ZONE1_POSITION;
import static org.firstinspires.ftc.teamcode.AutonConstants.BLUE_RIGHT_JUNCTION_POSITION;
import static org.firstinspires.ftc.teamcode.AutonConstants.BLUE_RIGHT_START_POSITION;
import static org.firstinspires.ftc.teamcode.AutonConstants.BLUE_RIGHT_ZONE1_POSITION;
import static org.firstinspires.ftc.teamcode.AutonConstants.RED_LEFT_JUNCTION_POSITION;
import static org.firstinspires.ftc.teamcode.AutonConstants.RED_LEFT_START_POSITION;
import static org.firstinspires.ftc.teamcode.AutonConstants.RED_LEFT_ZONE1_POSITION;
import static org.firstinspires.ftc.teamcode.AutonConstants.RED_RIGHT_JUNCTION_POSITION;
import static org.firstinspires.ftc.teamcode.AutonConstants.RED_RIGHT_START_POSITION;
import static org.firstinspires.ftc.teamcode.AutonConstants.RED_RIGHT_ZONE1_POSITION;
import static org.firstinspires.ftc.teamcode.AutonConstants.ZONE2_TO_ZONE1_DISTANCE;
import static org.firstinspires.ftc.teamcode.AutonConstants.ZONE2_TO_ZONE3_DISTANCE;
import static org.firstinspires.ftc.teamcode.RobotConstants.LIFT_POSITION_COEFFICIENT;
import static org.firstinspires.ftc.teamcode.RobotConstants.LIFT_POSITION_TOLERANCE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class Autonomous extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    //Tag IDs of sleeve
    int ZONE_1 = 3;
    int ZONE_2 = 6;
    int ZONE_3 = 9;

    AprilTagDetection tagOfInterest = null;

    SampleMecanumDrive drive;
    private Motor lLift, rLift;
    private MotorGroup lift;
    private Servo leftServo, rightServo;
    private int liftPosition;

    //These are set up such that default is left blue:
    boolean left, blue = true;
    boolean right, red = false;

    @Override
    public void runOpMode()
    {
        //                  ROBOT SETUP
        drive = new SampleMecanumDrive(hardwareMap);

        lLift = new Motor(hardwareMap, "lLift",Motor.GoBILDA.RPM_312);
        rLift = new Motor(hardwareMap, "rLift", Motor.GoBILDA.RPM_312);

        lLift.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rLift.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        rLift.setInverted(true);

        rLift.resetEncoder();
        lLift.resetEncoder();

        lift = new MotorGroup(lLift, rLift);

        lift.setRunMode(Motor.RunMode.PositionControl);
        lift.setPositionCoefficient(LIFT_POSITION_COEFFICIENT);
        lift.setPositionTolerance(LIFT_POSITION_TOLERANCE);

        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");

        //              ROBOT SETUP ENDED

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            telemetry.addLine("Press (X) for Blue or (B) for Red");
            telemetry.addLine("Press (Y) for Left or (A) for Right");

            if (gamepad1.x){
                blue = true;
                red = false;
            }else if (gamepad1.b){
                red = true;
                blue = false;
            }

            if (gamepad1.y){
                left = true;
                right = false;
            }else if (gamepad1.a){
                right = true;
                left = false;
            }

            if (blue && left){
                telemetry.addLine("Robot is on Blue Left");
            }else if (blue && right){
                telemetry.addLine("Robot is on Blue Right");
            }else if (red && left){
                telemetry.addLine("Robot is on Red Left");
            }else if (red & right){
                telemetry.addLine("Robot is on Red Right");
            }

            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == ZONE_1 || tag.id == ZONE_2 || tag.id == ZONE_3)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }


        if (blue && right){
            blueRight();
        }else if (red && left){
            redLeft();
        }else if (red && right){
            redRight();
        }else{
            blueLeft();
        }


        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {sleep(20);}
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
    void sleep(int duration){
        try {
            Thread.sleep(duration);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    void blueLeft(){
        TrajectorySequence traj = drive.trajectorySequenceBuilder(BLUE_LEFT_START_POSITION)
                .strafeTo(BLUE_LEFT_JUNCTION_POSITION)
                .waitSeconds(.8)
                .strafeTo(BLUE_LEFT_ZONE1_POSITION)
                .waitSeconds(.8)
                .build();
        drive.followTrajectorySequence(traj);

        Trajectory parkTraj = null;

        if(tagOfInterest.id == ZONE_1){
            parkTraj = drive.trajectoryBuilder(new Pose2d())
                    .back(ZONE2_TO_ZONE1_DISTANCE)
                    .build();
            drive.followTrajectory(parkTraj);
        }else if (tagOfInterest.id == ZONE_3){
            parkTraj = drive.trajectoryBuilder(new Pose2d())
                    .forward(ZONE2_TO_ZONE3_DISTANCE)
                    .build();
            drive.followTrajectory(parkTraj);
        }
    }


    void blueRight(){
        TrajectorySequence traj = drive.trajectorySequenceBuilder(BLUE_RIGHT_START_POSITION)
                .strafeTo(BLUE_RIGHT_JUNCTION_POSITION)
                .waitSeconds(.8)
                .strafeTo(BLUE_RIGHT_ZONE1_POSITION)
                .waitSeconds(.8)
                .build();
        drive.followTrajectorySequence(traj);

        Trajectory parkTraj = null;

        if(tagOfInterest.id == ZONE_1){
            parkTraj = drive.trajectoryBuilder(new Pose2d())
                    .forward(ZONE2_TO_ZONE1_DISTANCE)
                    .build();
            drive.followTrajectory(parkTraj);
        }else if (tagOfInterest.id == ZONE_3){
            parkTraj = drive.trajectoryBuilder(new Pose2d())
                    .back(ZONE2_TO_ZONE3_DISTANCE)
                    .build();
            drive.followTrajectory(parkTraj);
        }
    }

    void redLeft(){
        TrajectorySequence traj = drive.trajectorySequenceBuilder(RED_LEFT_START_POSITION)
                .strafeTo(RED_LEFT_JUNCTION_POSITION)
                .waitSeconds(.8)
                .strafeTo(RED_LEFT_ZONE1_POSITION)
                .waitSeconds(.8)
                .build();
        drive.followTrajectorySequence(traj);

        Trajectory parkTraj = null;

        if(tagOfInterest.id == ZONE_1){
            parkTraj = drive.trajectoryBuilder(new Pose2d())
                    .back(ZONE2_TO_ZONE1_DISTANCE)
                    .build();
            drive.followTrajectory(parkTraj);
        }else if (tagOfInterest.id == ZONE_3){
            parkTraj = drive.trajectoryBuilder(new Pose2d())
                    .forward(ZONE2_TO_ZONE3_DISTANCE)
                    .build();
            drive.followTrajectory(parkTraj);
        }
    }

    void redRight(){
        TrajectorySequence traj = drive.trajectorySequenceBuilder(RED_RIGHT_START_POSITION)
                .strafeTo(RED_RIGHT_JUNCTION_POSITION)
                .waitSeconds(.8)
                .strafeTo(RED_RIGHT_ZONE1_POSITION)
                .waitSeconds(.8)
                .build();
        drive.followTrajectorySequence(traj);

        Trajectory parkTraj = null;

        if(tagOfInterest.id == ZONE_1){
            parkTraj = drive.trajectoryBuilder(new Pose2d())
                    .forward(ZONE2_TO_ZONE1_DISTANCE)
                    .build();
            drive.followTrajectory(parkTraj);
        }else if (tagOfInterest.id == ZONE_3){
            parkTraj = drive.trajectoryBuilder(new Pose2d())
                    .back(ZONE2_TO_ZONE3_DISTANCE)
                    .build();
            drive.followTrajectory(parkTraj);
        }
    }
}