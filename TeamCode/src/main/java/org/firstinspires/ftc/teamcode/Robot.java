package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Robot {
    HardwareMap hwMap = null;
    // Declare OpMode members.
    public ElapsedTime period = new ElapsedTime();

    public DcMotor armRightMotor = null;
    public DcMotor armLeftMotor = null;
    public Servo leftFinger = null;
    public Servo rightFinger = null;
    public DcMotor motorFrontLeft = null;
    public DcMotor motorBackLeft = null;
    public DcMotor motorFrontRight = null;
    public DcMotor motorBackRight = null;

    BHI260IMU imu;
    IMU.Parameters IMUParameters;

    public void init(HardwareMap ahwmap){
        hwMap = ahwmap;

        //  Get all drive motors
        motorFrontLeft = hwMap.get(DcMotor.class, "motorFrontLeft");
        motorBackLeft = hwMap.get(DcMotor.class, "motorBackLeft");
        motorFrontRight = hwMap.get(DcMotor.class, "motorFrontRight");
        motorBackRight = hwMap.get(DcMotor.class, "motorBackRight");

        // Left side drive motors are forward
        motorFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        // Reverse the right side drive motors
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        //Set all drive motors to zero power brake
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Set all drive motors to zero power
        setAllDrivePower(0);
        //Set all drive motors to run without encoders
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Get arm motors
        armRightMotor = hwMap.get(DcMotor.class, "armRight");
        armLeftMotor = hwMap.get(DcMotor.class, "armLeft");

        //Set arm motors to work in tandem
//        armRightMotor.setDirection(DcMotor.Direction.FORWARD);
//        armLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        //Set arm run without encoder
        armRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Set arm motors brake mode
        armRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Get claw servos
        leftFinger = hwMap.get(Servo.class, "left_finger");
        rightFinger = hwMap.get(Servo.class, "right_finger");

        //Set claw left finger to work in tandem w/ right finger
        leftFinger.setDirection(Servo.Direction.REVERSE);

        //Reset Left/Right finger positions
        leftFinger.setPosition(0);
        rightFinger.setPosition(0);

        IMUParameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        // Get IMU
        imu = hwMap.get(BHI260IMU.class, "imu");
        imu.initialize(IMUParameters);
    }

    public void setAllDrivePower(double p){
        setDrivePower(p,p,p,p);
    }

    public void setDrivePower(double lF, double rF, double lB, double rB){
        motorFrontLeft.setPower(lF);
        motorBackLeft.setPower(lB);
        motorFrontRight.setPower(rF);
        motorBackRight.setPower(rB);
    }
}
