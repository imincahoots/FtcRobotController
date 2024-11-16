package org.firstinspires.ftc.teamcode.custom;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {

    public DcMotor linearSlideMotorLeft = null;
    public DcMotor linearSlideMotorRight = null;
    private int positionLinearSlideMotorLeft = 0;
    private int positionLinearSlideMotorMinLeft = 0;
    private int positionLinearSlideMotorMaxLeft = 0;
    private int positionLinearSlideMotorRight = 0;
    private int positionLinearSlideMotorMinRight = 0;
    private int positionLinearSlideMotorMaxRight = 0;
    private int positionLinearSlideMotorAvg = 0;
    
    //Constructor
    public Lift(HardwareMap hwMap) {
        linearSlideMotorLeft = hwMap.dcMotor.get("linearSlideMotorLeft");
        linearSlideMotorRight = hwMap.dcMotor.get("linearSlideMotorRight");
        linearSlideMotorRight.setDirection(DcMotorSimple.Direction.FORWARD);
        linearSlideMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        positionLinearSlideMotorMinLeft = linearSlideMotorLeft.getCurrentPosition();
        positionLinearSlideMotorMinRight = linearSlideMotorRight.getCurrentPosition();
        positionLinearSlideMotorMaxLeft = linearSlideMotorLeft.getCurrentPosition() + 2100;
        positionLinearSlideMotorMaxRight = linearSlideMotorRight.getCurrentPosition() + 2100;

        linearSlideMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        linearSlideMotorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        linearSlideMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        //linearSlideMotorLeft.setTargetPosition(positionLinearSlideMotorMinLeft);
        //linearSlideMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //linearSlideMotorRight.setTargetPosition(positionLinearSlideMotorMinRight);
        //linearSlideMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    
    public void holdBottom(){
        holdPosition(positionLinearSlideMotorMinLeft, positionLinearSlideMotorMinRight);
    }
    //Move up or down as commanded by joystick.  Stop when joystick is 0 and hold position.
    public void movSlide(double speedCmd){
        //linear slide
        if (speedCmd == 0){
            holdPosition(positionLinearSlideMotorLeft, positionLinearSlideMotorRight);
        } else if ((positionLinearSlideMotorLeft < positionLinearSlideMotorMinLeft) && (speedCmd < 0)){
            holdPosition(positionLinearSlideMotorMinLeft, positionLinearSlideMotorMinRight);
        } else if (((positionLinearSlideMotorLeft > positionLinearSlideMotorMaxLeft) && (speedCmd > 0))) {
            holdPosition(positionLinearSlideMotorMaxLeft, positionLinearSlideMotorMaxRight);
        } else {
            linearSlideMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            linearSlideMotorLeft.setPower(speedCmd);
            linearSlideMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            linearSlideMotorRight.setPower(speedCmd);
            positionLinearSlideMotorLeft = linearSlideMotorLeft.getCurrentPosition();
            positionLinearSlideMotorRight = linearSlideMotorRight.getCurrentPosition();
        }
    }

    public boolean holdPosition(int left, int right){
        linearSlideMotorLeft.setTargetPosition(left);
        linearSlideMotorRight.setTargetPosition(right);
        linearSlideMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlideMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlideMotorLeft.setPower(1.0);
        linearSlideMotorRight.setPower(1.0);
        if (linearSlideMotorLeft.getCurrentPosition() == left && linearSlideMotorRight.getCurrentPosition() == right){
            return true;
        }else {
            return false;
        }
    }

    public boolean liftTransit (int position){
        linearSlideMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linearSlideMotorRight.setPower(-(linearSlideMotorLeft.getPower()));
        if (linearSlideMotorLeft.getCurrentPosition() == position) {
            linearSlideMotorLeft.setTargetPosition(position);
            linearSlideMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            return true;
        } else if (position > linearSlideMotorLeft.getCurrentPosition()){
            linearSlideMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            linearSlideMotorLeft.setPower(1);
            return false;
        } else if (position < linearSlideMotorLeft.getCurrentPosition()){
            linearSlideMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            linearSlideMotorLeft.setPower(-1);
            return false;
        }
        return false;
    }

    public int getLeftPos(){
        return linearSlideMotorLeft.getCurrentPosition();
    }

    public int getRightPos(){
        return linearSlideMotorRight.getCurrentPosition();
    }

    public double getLeftPower(){
        return linearSlideMotorLeft.getPower();
    }

    public double getRightPower(){
        return linearSlideMotorLeft.getPower();
    }

}
