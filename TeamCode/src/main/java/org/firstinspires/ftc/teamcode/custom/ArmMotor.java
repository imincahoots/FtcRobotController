package org.firstinspires.ftc.teamcode.custom;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ArmMotor {
    // initialize variables

    public DcMotor armMot = null;
    private int armMotPos = 0;
    private int armMotPosMax = 0;
    private int armMotPosMin = 0;
    private boolean setPointReached = false;
    //resets encoders, sets mins and maxes
    public ArmMotor(HardwareMap hwMap) {
        armMot = hwMap.dcMotor.get("armMotor");
        armMot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMot.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotPosMin = armMot.getCurrentPosition();
        armMotPosMax = armMot.getCurrentPosition() + 300;


    }

    public void armMotStickControl(double cmd){
        if (cmd == 0){
            armMot.setTargetPosition(armMotPos);
            armMot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else {
            armMot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armMot.setPower(-cmd*0.75);
            armMotPos = armMot.getCurrentPosition();
        }
    }
    public void armMotHoldPos(int pos){
        armMot.setPower(1);
        armMot.setTargetPosition(pos);
        armMot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    // sets the target position of the arm motor to the given value
    public boolean  armGoToAngle(int targetTicks) {
        armMot.setPower(1);
        armMot.setTargetPosition(targetTicks);
        armMot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (targetTicks == armMot.getCurrentPosition()) {
            return true;
        } else {
            return false;
        }

    }
    public void pidControl(int armMotPos, int targetPos){
        /*

         * Proportional Integral Derivative Controller

         */
        double Kp = 0.000455;
        double Ki = 0;
        double Kd = 0;
        double integralSum = 0;
        double lastError = 0;
        armMot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotPos = armMot.getCurrentPosition();
        if (armMotPos == targetPos){
            setPointReached = true;
            armMotPos = armMot.getCurrentPosition();
        }else{
            setPointReached = false;
            armMotPos = armMot.getCurrentPosition();
        }

// Elapsed timer class from SDK, please use it, it's epic
        ElapsedTime timer = new ElapsedTime();

        while (setPointReached == false) {
            // calculate the error
            double error = targetPos - armMotPos;

            // rate of change of the error
            double derivative = (error - lastError) / timer.seconds();

            // sum of all error over time
            integralSum = integralSum + (error * timer.seconds());

            double out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

            armMot.setPower(out);

            lastError = error;

            // reset the timer for next time
            timer.reset();

            if (armMotPos == targetPos){
                setPointReached = true;
                armMotPos = armMot.getCurrentPosition();
            }else{
                setPointReached = false;
                armMotPos = armMot.getCurrentPosition();
            }

        }
    }
}
