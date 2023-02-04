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

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

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
    private Robot robot = new Robot();
    private ElapsedTime runtime = new ElapsedTime();
    private Gyro gyro;
    private double initialAngle = 0;

    float[] hsvValues = {0f, 0f, 0f};
    final float[] values = hsvValues;
    final double SCALE_FACTOR = 255;

    public double clawOpenTime = 0;

    private double dPadMovementSpeed = 0.1;

    int blueReading = 0;
    int redReading = 0;

    @Override
    public void init() {
        robot.init(hardwareMap);
        gyro  = new Gyro(robot);
        gyro.resetAngle();
        initialAngle = robot.imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;

        runtime.reset();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    public void raiseArm(double power){
        if(robot.armRightMotor.getCurrentPosition()>=155){
            robot.armRightMotor.setPower(0.3);
            robot.armLeftMotor.setPower(-0.3);
        }else{
            robot.armRightMotor.setPower(power);
            robot.armLeftMotor.setPower(-power);
        }
    }

    public void dropArm(){
        robot.armRightMotor.setPower(-0.5);
        robot.armLeftMotor.setPower(0.5);
    }

    public void restArm(){
        robot.armRightMotor.setPower(0);
        robot.armLeftMotor.setPower(0);
    }

    public void openClaw(){
        robot.leftFinger.setPosition(0.162);
        robot.rightFinger.setPosition(0.162);
    }

    public void closeClaw(){
        robot.leftFinger.setPosition(0);
        robot.rightFinger.setPosition(0);
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
        boolean clawDetected = false;
        double speedMultiplier = 0.4;


        Color.RGBToHSV((int)(robot.sensorColor.red() * SCALE_FACTOR),
                    (int) (robot.sensorColor.green() * SCALE_FACTOR),
                    (int) (robot.sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);

        if (gamepad1.b || (robot.sensorDistance.getDistance(DistanceUnit.CM) < 0.639)&& (robot.sensorColor.red() > robot.sensorColor.blue())
                && (robot.sensorColor.green() > robot.sensorColor.blue()) && (robot.sensorColor.green() > robot.sensorColor.red())){
            clawOpenTime = runtime.time();
            openClaw();

        }else if (gamepad1.x || (robot.sensorDistance2.getDistance(DistanceUnit.CM) < 5.07 && runtime.time() >= clawOpenTime + .75 &&
                 gamepad1.right_trigger>0)){
            clawDetected = true;
            closeClaw();
        }

        if(gamepad1.y){
            raiseArm(1);
        }else if (gamepad1.a){
            dropArm();
        }else{
            restArm();
        }

//        if(gamepad1.dpad_left){
//            gyro.turnToPID(initialAngle+90);
//        }else if (gamepad1.dpad_right){
//            gyro.turnToPID(initialAngle-90);
//        }else if (gamepad1.dpad_up){
//            gyro.turnToPID(initialAngle);
//        }else if (gamepad1.dpad_down){
//            gyro.turnToPID(initialAngle+180);
//        }

        if (gamepad1.left_trigger > 0){
            speedMultiplier = 1;
        }

        //Movement code starts here

        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
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

        if (!clawDetected) {
            if (gamepad1.dpad_left) {
                robot.setDrivePower(-dPadMovementSpeed, dPadMovementSpeed, dPadMovementSpeed, -dPadMovementSpeed);
            } else if (gamepad1.dpad_right) {
                robot.setDrivePower(dPadMovementSpeed, -dPadMovementSpeed, -dPadMovementSpeed, dPadMovementSpeed);
            } else if (gamepad1.dpad_up) {
                robot.setAllDrivePower(dPadMovementSpeed);
            } else if (gamepad1.dpad_down) {
                robot.setAllDrivePower(-dPadMovementSpeed);
            } else {
                robot.motorFrontLeft.setPower(frontLeftPower * speedMultiplier);
                robot.motorBackLeft.setPower(backLeftPower * speedMultiplier);
                robot.motorFrontRight.setPower(frontRightPower * speedMultiplier);
                robot.motorBackRight.setPower(backRightPower * speedMultiplier);
            }
        }else if (frontLeftPower == 0 && frontRightPower ==0 && backRightPower ==0 && backLeftPower ==0){
            robot.setAllDrivePower(0);
        }

        blueReading = robot.sensorColor2.blue();
        redReading = robot.sensorColor2.red();

        //Movement code ends here

        telemetry.addData("x: ", robot.imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle);
        telemetry.addData("y: ", robot.imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle);
        telemetry.addData("z: ", robot.imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
        telemetry.addData("Arm Pos: ",robot.armRightMotor.getCurrentPosition());
        telemetry.addData("Distance(cm): ", String.format(Locale.US, "%.02f", robot.sensorDistance.getDistance(DistanceUnit.CM)));
        telemetry.addData("Alpha: ", robot.sensorColor.alpha());
        telemetry.addData("Red: ", robot.sensorColor.red());
        telemetry.addData("Green: ", robot.sensorColor.green());
        telemetry.addData("Blue: ", robot.sensorColor.blue());
        telemetry.addData("Hue: ", hsvValues[0]);
        telemetry.addData("Distance(cm) Sensor 2: ", String.format(Locale.US, "%.02f", robot.sensorDistance2.getDistance(DistanceUnit.CM)));
        telemetry.addData("Red2: ", robot.sensorColor2.red());
        telemetry.addData("Green2: ", robot.sensorColor2.green());
        telemetry.addData("Blue2: ", robot.sensorColor2.blue());
        telemetry.addData("RunTime:", runtime.time());
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}