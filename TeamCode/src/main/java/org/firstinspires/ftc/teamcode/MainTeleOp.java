package org.firstinspires.ftc.teamcode;

import static com.arcrobotics.ftclib.gamepad.GamepadEx.*;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="MainOpMode")
public class MainTeleOp extends LinearOpMode {

    private Motor fL, fR, bL, bR;
    private MecanumDrive m_drive;
    private GamepadEx driverController1;
    private GamepadEx driverController2;

    private Motor lLift, rLift;
    private MotorGroup lift;

    private int liftPosition;

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

        lift = new MotorGroup(lLift, rLift);

        lift.setRunMode(Motor.RunMode.VelocityControl);
//        lift.setPositionCoefficient(0.3);
//        lift.setPositionTolerance(30);

//        liftPosition = lLift.getCurrentPosition();

        waitForStart();
        while(opModeIsActive()){
//            if(driverController1.getButton(GamepadKeys.Button.Y)){
//                liftPosition += 10;
//            }else if (driverController1.getButton(GamepadKeys.Button.A)){
//                liftPosition -= 10;
//            }
//
//            lift.setTargetPosition(liftPosition);
//            lift.set(0.7);

            if (driverController1.getButton(GamepadKeys.Button.Y)){
                lift.set(1);
            }else if (driverController1.getButton(GamepadKeys.Button.A)){
                lift.set(-1);
            }else{
                lift.set(0);
            }

            //4268
            //7450

            telemetry.addData("Position Left:", lLift.getCurrentPosition());
            telemetry.addData("Position Right:", rLift.getCurrentPosition());

            telemetry.update();

            m_drive.driveRobotCentric(driverController1.getLeftX(), driverController1.getLeftY(), driverController1.getRightX());


        }
    }
}
