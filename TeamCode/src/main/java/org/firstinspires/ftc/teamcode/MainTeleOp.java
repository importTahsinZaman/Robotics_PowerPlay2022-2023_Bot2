package org.firstinspires.ftc.teamcode;

import static com.arcrobotics.ftclib.gamepad.GamepadEx.*;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.RobotConstants.BOTTOMLIFTPOSITION;
import static org.firstinspires.ftc.teamcode.RobotConstants.GROUNDJUNCTIONPOSITION;
import static org.firstinspires.ftc.teamcode.RobotConstants.LEFTSERVOCLOSEPOSITION;
import static org.firstinspires.ftc.teamcode.RobotConstants.LEFTSERVOOPENPOSITION;
import static org.firstinspires.ftc.teamcode.RobotConstants.LIFTMULTIPLIER;
import static org.firstinspires.ftc.teamcode.RobotConstants.LIFTPOSITIONCOEFFICIENT;
import static org.firstinspires.ftc.teamcode.RobotConstants.LIFTPOSITIONTOLERANCE;
import static org.firstinspires.ftc.teamcode.RobotConstants.LOWJUNCTIONPOSITION;
import static org.firstinspires.ftc.teamcode.RobotConstants.MIDJUNCTIONPOSITION;
import static org.firstinspires.ftc.teamcode.RobotConstants.HIGHJUNCTIONPOSITION;
import static org.firstinspires.ftc.teamcode.RobotConstants.RIGHTSERVOCLOSEPOSITION;
import static org.firstinspires.ftc.teamcode.RobotConstants.RIGHTSERVOOPENPOSITION;
import static org.firstinspires.ftc.teamcode.RobotConstants.TOPLIFTPOSITION;

@TeleOp(name="MainOpMode")
public class MainTeleOp extends LinearOpMode {

    private Motor fL, fR, bL, bR;
    private MecanumDrive m_drive;
    private GamepadEx driverController1;
    private GamepadEx driverController2;

    private Motor lLift, rLift;
    private MotorGroup lift;

    private int liftPosition;

    private final int[] LIFTPOSITIONS = new int[]{ GROUNDJUNCTIONPOSITION, LOWJUNCTIONPOSITION, MIDJUNCTIONPOSITION, HIGHJUNCTIONPOSITION }; //MUST BE LEAST TO GREATEST

    private Servo leftServo, rightServo;

    @Override
    public void runOpMode() throws InterruptedException{
        driverController1 = new GamepadEx(gamepad1);
        driverController2 = new GamepadEx(gamepad2);

        fL = new Motor(hardwareMap, "fL"); //port 0
        fR = new Motor(hardwareMap, "fR"); //port 1
        bL = new Motor(hardwareMap, "bL"); //port 2
        bR = new Motor(hardwareMap, "bR"); //port 3

        fL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        fR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        bL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        bR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        m_drive = new MecanumDrive(fL, fR, bL, bR);

        lLift = new Motor(hardwareMap, "lLift", Motor.GoBILDA.RPM_312);
        rLift = new Motor(hardwareMap, "rLift", Motor.GoBILDA.RPM_312);

        lLift.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rLift.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        rLift.setInverted(true);

        rLift.resetEncoder();
        lLift.resetEncoder();

        lift = new MotorGroup(lLift, rLift);

        lift.setRunMode(Motor.RunMode.PositionControl);
        lift.setPositionCoefficient(LIFTPOSITIONCOEFFICIENT);
        lift.setPositionTolerance(LIFTPOSITIONTOLERANCE);

        liftPosition = lLift.getCurrentPosition();

        Arrays.sort(LIFTPOSITIONS);

        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");

        waitForStart();
        while(opModeIsActive()){

             if(driverController2.getButton(GamepadKeys.Button.Y) || driverController2.getButton(GamepadKeys.Button.A)){
                 int distance = Math.abs(LIFTPOSITIONS[0] - liftPosition);
                 int idx = 0;
                 for(int c = 1; c < LIFTPOSITIONS.length; c++){
                     int cdistance = Math.abs(LIFTPOSITIONS[c] - liftPosition);
                     if(cdistance < distance){
                         idx = c;
                         distance = cdistance;
                     }
                 }
                 if (driverController2.getButton(GamepadKeys.Button.Y)) {
                     if (idx == LIFTPOSITIONS.length -1){
                         liftPosition = LIFTPOSITIONS[idx];
                     }else if (LIFTPOSITIONS[idx] > liftPosition){
                         liftPosition = LIFTPOSITIONS[idx];
                     }else if (LIFTPOSITIONS[idx] < liftPosition){
                         liftPosition = LIFTPOSITIONS[idx+1];
                     }
                 }else if(driverController2.getButton(GamepadKeys.Button.A)){
                     if (idx == 0){
                         liftPosition = LIFTPOSITIONS[0];
                     }else if (LIFTPOSITIONS[idx] < liftPosition){
                         liftPosition = LIFTPOSITIONS[idx];
                     }else if (LIFTPOSITIONS[idx] > liftPosition){
                         liftPosition = LIFTPOSITIONS[idx-1];
                     }
                 }
            }else{
                 liftPosition += gamepad2.right_trigger * LIFTMULTIPLIER;
                 liftPosition -= gamepad2.left_trigger * LIFTMULTIPLIER;
             }

            if (liftPosition > TOPLIFTPOSITION){
                liftPosition = TOPLIFTPOSITION;
            }else if (liftPosition <= BOTTOMLIFTPOSITION){
                liftPosition = BOTTOMLIFTPOSITION;
            }

            lift.setTargetPosition(liftPosition);


            if(lift.atTargetPosition()){
                lift.set(0);
            }else{
                lift.set(1);
            }

            if(gamepad2.x){
                leftServo.setPosition(LEFTSERVOCLOSEPOSITION);
                rightServo.setPosition(RIGHTSERVOCLOSEPOSITION);
            }else if(gamepad2.b){
                leftServo.setPosition(LEFTSERVOOPENPOSITION);
                rightServo.setPosition(RIGHTSERVOOPENPOSITION);
            }

            telemetry.addData("Target Position:", liftPosition);
            telemetry.addData("Position Left:", lLift.getCurrentPosition());
            telemetry.addData("Position Right:", rLift.getCurrentPosition());

            telemetry.addData("leftServo Position:", leftServo.getPosition());
            telemetry.addData("rightServo Position", rightServo.getPosition());

            telemetry.update();

            m_drive.driveRobotCentric(driverController1.getLeftX(), driverController1.getLeftY(), driverController1.getRightX());


        }
    }
}
