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

@TeleOp(name="MainOpMode")
public class MainTeleOp extends LinearOpMode {

    private Motor fL, fR, bL, bR;
    private MecanumDrive m_drive;
    private GamepadEx driverController1;
    private GamepadEx driverController2;

    private Motor lLift, rLift;
    private MotorGroup lift;

    private int liftPosition;

    private int topLiftPosition = 4500;
    private int bottomLiftPosition = 15;

    int[] LIFTPOSITIONS = new int[]{ 300, 800, 1300 }; //MUST BE LEAST TO GREATEST

    private Servo leftServo, rightServo;

    @Override
    public void runOpMode() throws InterruptedException{
        driverController1 = new GamepadEx(gamepad1);
        driverController2 = new GamepadEx(gamepad2);

        fL = new Motor(hardwareMap, "fL"); //port 0
        fR = new Motor(hardwareMap, "fR"); //port 1
        bL = new Motor(hardwareMap, "bL"); //port 2
        bR = new Motor(hardwareMap, "bR"); //port 3

        fL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        m_drive = new MecanumDrive(fL, fR, bL, bR);

        lLift = new Motor(hardwareMap, "lLift", Motor.GoBILDA.RPM_312);
        rLift = new Motor(hardwareMap, "rLift", Motor.GoBILDA.RPM_312);

        lLift.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rLift.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        rLift.setInverted(true);

        rLift.resetEncoder();
        lLift.resetEncoder();

        lift = new MotorGroup(lLift, rLift);

        PIDFController pidf = new PIDFController(0.02, 0,0,0.6);

        lift.setRunMode(Motor.RunMode.VelocityControl);
        lift.setVeloCoefficients(0.01, 0.5, 0.001);
        lift.setFeedforwardCoefficients(0.92, 0.47);

        liftPosition = lLift.getCurrentPosition();

        Arrays.sort(LIFTPOSITIONS);

        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");

        waitForStart();
        while(opModeIsActive()){
            if (driverController2.getButton(GamepadKeys.Button.RIGHT_BUMPER)){
                    liftPosition += 30;
            }else if (driverController2.getButton(GamepadKeys.Button.LEFT_BUMPER)){
                    liftPosition -= 30;
            } else if(driverController2.getButton(GamepadKeys.Button.Y)){
                if (liftPosition < LIFTPOSITIONS[2] && liftPosition > LIFTPOSITIONS[1]){
                    liftPosition = LIFTPOSITIONS[2];
                }else if (liftPosition < LIFTPOSITIONS[1] && liftPosition > LIFTPOSITIONS[0]){
                    liftPosition = LIFTPOSITIONS[1];
                }else {
                    liftPosition = LIFTPOSITIONS[0];
                }
            }else if(driverController2.getButton(GamepadKeys.Button.A)){
                if (liftPosition > LIFTPOSITIONS[0] && liftPosition < LIFTPOSITIONS[1]){
                    liftPosition = LIFTPOSITIONS[0];
                }else if (liftPosition > LIFTPOSITIONS[1] && liftPosition < LIFTPOSITIONS[2]){
                    liftPosition = LIFTPOSITIONS[1];
                }else {
                    liftPosition = LIFTPOSITIONS[2];
                }
            }

            if (liftPosition > topLiftPosition){
                liftPosition = topLiftPosition;
            }else if (liftPosition < bottomLiftPosition){
                liftPosition = bottomLiftPosition;
            }


            double output = pidf.calculate(
                        lLift.getCurrentPosition(), liftPosition
            );
            lift.set(output);

            //4268
            //7450

            telemetry.addData("Desired SetPoint", liftPosition);

            telemetry.addData("Position Left:", lLift.getCurrentPosition());
            telemetry.addData("Position Right:", rLift.getCurrentPosition());

            telemetry.addData("leftServo Position:", leftServo.getPosition());
            telemetry.addData("rightServo Position", rightServo.getPosition());

            telemetry.update();

            m_drive.driveRobotCentric(driverController1.getLeftX(), driverController1.getLeftY(), driverController1.getRightX());


        }
    }
}
