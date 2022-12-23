/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Iterative OpMode", group="Iterative Opmode")
public class BasicOpMode_Iterative extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor armRightMotor = null;
    private DcMotor armLeftMotor = null;
    private Servo leftFinger = null;
    private Servo rightFinger = null;
    private DcMotor motorFrontLeft = null;
    private DcMotor motorBackLeft = null;
    private DcMotor motorFrontRight = null;
    private DcMotor motorBackRight = null;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        armRightMotor = hardwareMap.get(DcMotor.class, "armRight");
        armLeftMotor = hardwareMap.get(DcMotor.class, "armLeft");

        armRightMotor.setDirection(DcMotor.Direction.FORWARD);
        armLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        armRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFrontLeft = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorFrontRight = hardwareMap.get(DcMotor.class, "motorFrontRight");
        motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight");

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);



        leftFinger = hardwareMap.get(Servo.class, "left_finger");
        rightFinger = hardwareMap.get(Servo.class, "right_finger");

        leftFinger.setDirection(Servo.Direction.REVERSE);


        leftFinger.setPosition(0);
        rightFinger.setPosition(0);

        autonomous();

        runtime.reset();

        telemetry.addData("Status", "Initialized");
    }
    public void cSleep (int time){
        try {
            Thread.sleep(time);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public void stopMovement() {
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }

    public void moveForward(double speed){
        motorFrontLeft.setPower(speed);
        motorFrontRight.setPower(speed);
        motorBackLeft.setPower(speed);
        motorBackRight.setPower(speed);
    }

    public void moveBackward(double speed){
        motorFrontLeft.setPower(-speed);
        motorFrontRight.setPower(-speed);
        motorBackLeft.setPower(-speed);
        motorBackRight.setPower(-speed);
    }

    public void moveRight(double speed){
        motorFrontLeft.setPower(speed);
        motorFrontRight.setPower(-speed);
        motorBackLeft.setPower(-speed);
        motorBackRight.setPower(speed);
    }

    public void moveLeft(double speed){
        motorFrontLeft.setPower(-speed);
        motorFrontRight.setPower(speed);
        motorBackLeft.setPower(speed);
        motorBackRight.setPower(-speed);
    }

    public void raiseArm(double power){
        armRightMotor.setDirection(DcMotor.Direction.FORWARD);
        armLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        if(armRightMotor.getCurrentPosition()>=155){
            armRightMotor.setPower(0.3);
            armLeftMotor.setPower(0.3);
        }else{
            armRightMotor.setPower(power);
            armLeftMotor.setPower(power);
        }
    }

    public void dropArm(){
        armRightMotor.setDirection(DcMotor.Direction.REVERSE);
        armLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        armRightMotor.setPower(0.5);
        armLeftMotor.setPower(0.5);
    }

    public void restArm(){
        armRightMotor.setPower(0);
        armLeftMotor.setPower(0);
    }

    public void openClaw(){
        leftFinger.setPosition(0.162);
        rightFinger.setPosition(0.162);
    }

    public void closeClaw(){
        leftFinger.setPosition(0);
        rightFinger.setPosition(0);
    }

    public void forwardScore() {
        closeClaw();
        raiseArm(.8);
        cSleep(1650);
        moveForward(.5);
        cSleep(600);
        stopMovement();
    }

    public void returnDock(){
        dropArm();
        cSleep(300);
        restArm();
        openClaw();
        moveBackward(.5);
        cSleep(600);
        stopMovement();
    }

    public void autonomous(){
        closeClaw();
    moveForward(.5);
    cSleep(1300);
    stopMovement();
    raiseArm(.8);
}

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        closeClaw();
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        if (gamepad1.b){
            openClaw();
        }else if (gamepad1.x){
            closeClaw();
        }

        if (gamepad1.y){
            raiseArm(1);
        }
        else if(gamepad1.a){
            dropArm();
        }
        else{
            restArm();
        }

        while (armRightMotor.isBusy()){
            telemetry.addData("Status", "Running Arm");
            telemetry.update();
        }

        if (gamepad1.dpad_up){
            forwardScore();
        }else if (gamepad1.dpad_down){
            returnDock();
        }


        double y = -gamepad1.left_stick_y; // This is reversed.
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        motorFrontLeft.setPower(frontLeftPower*.7);
        motorBackLeft.setPower(backLeftPower*.7);
        motorFrontRight.setPower(frontRightPower*.7);
        motorBackRight.setPower(backRightPower*.7);

        telemetry.addData("Arm Pos:", armRightMotor.getCurrentPosition());

        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
