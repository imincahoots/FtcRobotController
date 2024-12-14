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

package org.firstinspires.ftc.teamcode.disabled;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.custom.Lift;

/*
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
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

/*
welcome!

This is the first iteration of what will be considered the main TeleOp program for the 2024-25
into the deep season. This is an updated version of the code that includes all basic functions of the robot
including drivetrain, wrist servo, intake, arm actuation, and arm elevation. The code right now is pretty messy,
but this is the first iteration of the program where I have made an attempt to condense it. I started with the
linear slides, so you can compare this program with the previous version called "ITDDualGamepadV2"
The logic for the linear slides is mostly done in the file "Lift", so for any confusion look there
good luck
p.s. I will do better commenting in future programs, I just wrote this one at like 9pm when I was exhausted
lacked time

Last updated on 10/24/2024

 */

@TeleOp
@Disabled
public class ITDMainTeleOp extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    DcMotor frontLeftMotor = null;
    DcMotor backLeftMotor = null;
    DcMotor frontRightMotor = null;
    DcMotor backRightMotor = null;
    CRServo crServoRubberWheel = null;
    Servo wristServo = null;
    DcMotor armMotor = null;
    int positionArmMotor = 0;
    double frontLeftPower = 0;
    double backLeftPower = 0;
    double frontRightPower = 0;
    double backRightPower = 0;
    Lift myLift;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters

        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        crServoRubberWheel = hardwareMap.crservo.get("crServoRubberWheel");
        wristServo = hardwareMap.servo.get("wristServo");
        armMotor = hardwareMap.dcMotor.get("armMotor");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        myLift = new Lift(hardwareMap);     // New instance of the "lift" class

        positionArmMotor = armMotor.getCurrentPosition();
        armMotor.setTargetPosition(positionArmMotor);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
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
        runtime.reset();
        //makes the arm motor and linear slide motors hold their positions when start is pressed
        positionArmMotor = armMotor.getCurrentPosition();
        armMotor.setTargetPosition(positionArmMotor);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        myLift.holdBottom();    // Hang on!

    }
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        // set variables for controller inputs
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;
        double ly2 = gamepad2.left_stick_y;
        double ry2 = gamepad2.right_stick_y;
        boolean a_button = gamepad2.a;
        boolean b_button = gamepad2.b;
        double leftTriggerRaw = gamepad2.left_trigger;
        double rightTriggerRaw = gamepad2.right_trigger;
        boolean dpadUp1 = gamepad1.dpad_up;
        boolean dpadDown1 = gamepad1.dpad_down;
        boolean dpadLeft1 = gamepad1.dpad_left;
        boolean dpadRight1 = gamepad1.dpad_right;
        double speedModifier = 1-(gamepad1.left_trigger*0.8);

        // Does math and calibrates the trigger positions to be inline to the servo's requirements
        double trigger = 0.5+(leftTriggerRaw*0.5)-(rightTriggerRaw*0.5);

        // sets wrist servo's position equal to what we just calculated
        wristServo.setPosition(trigger);

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        // TLDR mecanum wheel drive math
        // this is also an if statement. basically, it just does the 90 degree movements that
        // owen wanted when the left stick is not being used
        
        if (!( y == 0 ) || !( x == 0 ) || !( rx == 0)) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        frontLeftPower = (y + x - rx) / denominator;
        backLeftPower = (y - x + rx) / denominator;
        frontRightPower = (y - x - rx) / denominator;
        backRightPower = (y + x + rx) / denominator;
        }
        else if (dpadUp1 || dpadDown1 || dpadLeft1 || dpadRight1){
          if (dpadUp1) {
            frontLeftPower = 1;
            backLeftPower = 1;
            frontRightPower = 1;
            backRightPower = 1;
          }
          else if (dpadDown1){
            frontLeftPower = -1;
            backLeftPower = -1;
            frontRightPower = -1;
            backRightPower = -1;   
          }
          
          if (dpadRight1) {
            frontLeftPower = 1;
            backLeftPower = -1;
            frontRightPower = 1;
            backRightPower = -1;
          }
          else if (dpadLeft1) {
            frontLeftPower = -1;
            backLeftPower = 1;
            frontRightPower = -1;
            backRightPower = 1;
          }
        }else{
            frontLeftPower = 0;
            backLeftPower = 0;
            frontRightPower = 0;
            backRightPower = 0;
        }

        frontLeftMotor.setPower(frontLeftPower*speedModifier);
        backLeftMotor.setPower(backLeftPower*speedModifier);
        frontRightMotor.setPower(frontRightPower*speedModifier);
        backRightMotor.setPower(backRightPower*speedModifier);

        //input for continuous rotation servo with rubber wheel
        if (a_button && b_button) {
            crServoRubberWheel.setPower(0);
        }
        else if (a_button){
            crServoRubberWheel.setPower(-1);
        }
        else if (b_button){
            crServoRubberWheel.setPower(1);
        }
        else {
            crServoRubberWheel.setPower(0);
        }
        //arm motor
        if (ly2 == 0){
          armMotor.setTargetPosition(positionArmMotor);
          armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else {
          armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
          armMotor.setPower(ly2);
          positionArmMotor = armMotor.getCurrentPosition();
        }
       //linear slide
        myLift.moveSlide(-ry2);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("LeftTrigger", gamepad1.left_trigger);

        telemetry.addData("Positions", "left (%d), right (%d)",
                myLift.getLeftPos(), myLift.getRightPos());
        telemetry.addData("Powers", "left (%.2f), right (%.2f)",
                myLift.getLeftPower(), myLift.getRightPower());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
