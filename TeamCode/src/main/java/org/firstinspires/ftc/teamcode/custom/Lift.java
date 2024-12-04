package org.firstinspires.ftc.teamcode.custom;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {
    //LSM = Linear Slide Motor
    public DcMotor LSMLeft = null;
    public DcMotor LSMRight = null;
    private int posLSMLeft = 0;
    private int posLSMMinLeft = 0;
    private int posLSMMaxLeft = 0;
    private int posLSMRight = 0;
    private int posLSMMinRight = 0;
    private int posLSMMaxRight = 0;
    private int posLSMMaxLeftWorm = 0;
    private int posLSMMaxRightWorm = 0;
    private int posLSMAvg = 0;
    private int moveStatus;
    private int requestedPos;
    
    //Constructor
    public Lift(HardwareMap hwMap) {
        LSMLeft = hwMap.dcMotor.get("LinearSlideMotorLeft");
        LSMRight = hwMap.dcMotor.get("LinearSlideMotorRight");
        LSMRight.setDirection(DcMotorSimple.Direction.FORWARD);
        LSMLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        posLSMMinLeft = LSMLeft.getCurrentPosition();
        posLSMMinRight = LSMRight.getCurrentPosition();
        posLSMMaxLeft = LSMLeft.getCurrentPosition() + 2100;
        posLSMMaxRight = LSMRight.getCurrentPosition() + 2100;
        posLSMMaxLeftWorm = LSMLeft.getCurrentPosition() + 210000;
        posLSMMaxRightWorm = LSMRight.getCurrentPosition() + 210000;
        LSMLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LSMRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LSMLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        LSMRight.setDirection(DcMotorSimple.Direction.REVERSE);

        //LSMLeft.setTargetPosition(posLSMMinLeft);
        //LSMLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //LSMRight.setTargetPosition(posLSMMinRight);
        //LSMRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    
    public void holdBottom(){
        holdPosition(posLSMMinLeft, posLSMMinRight);
    }
    //Move up or down as commanded by joystick.  Stop when joystick is 0 and hold position.
    public void moveSlide(double speedCmd){
        //linear slide
        if (speedCmd == 0){
            holdPosition(posLSMLeft, posLSMRight);
        } else if ((posLSMLeft < posLSMMinLeft) && (speedCmd < 0)){
            holdPosition(posLSMMinLeft, posLSMMinRight);
        } else if (((posLSMLeft > posLSMMaxLeft) && (speedCmd > 0))) {
            holdPosition(posLSMMaxLeft, posLSMMaxRight);
        } else if (posLSMLeft <= 15 && speedCmd == 0) {
            LSMLeft.setPower(0);
            LSMRight.setPower(0);
        }else {
            LSMLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LSMLeft.setPower(speedCmd);
            LSMRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LSMRight.setPower(speedCmd);
            posLSMLeft = LSMLeft.getCurrentPosition();
            posLSMRight = LSMRight.getCurrentPosition();
        }
    }
    
    public void moveSlideWorm(double speedCmd){
        if (speedCmd == 0){
            LSMLeft.setPower(0);
            LSMRight.setPower(0);
        }  else if (posLSMLeft <= 15 && speedCmd == 0) {
            LSMLeft.setPower(0);
            LSMRight.setPower(0);
        } else if (((posLSMLeft > posLSMMaxLeftWorm) && (speedCmd > 0))) {
            holdPosition(posLSMMaxLeftWorm, posLSMMaxRightWorm);
        } else {
            LSMLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LSMLeft.setPower(speedCmd);
            LSMRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LSMRight.setPower(speedCmd);
            posLSMLeft = LSMLeft.getCurrentPosition();
            posLSMRight = LSMRight.getCurrentPosition();
        }
    }

    public boolean holdPosition(int left, int right){
        //this method hold the position it is being passed DO NOT USE IT TO GO TO A NEW POSITION
        //for that, use liftTransit
        LSMLeft.setTargetPosition(left);
        LSMRight.setTargetPosition(right);
        LSMLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LSMRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LSMLeft.setPower(1.0);
        LSMRight.setPower(1.0);
        if (LSMLeft.getCurrentPosition() == left && LSMRight.getCurrentPosition() == right){
            if(LSMLeft.getCurrentPosition() <= 0){
                LSMLeft.setPower(0);
                LSMRight.setPower(0);
            }
            return true;
        }else {
            return false;
        }
    }

    public boolean liftTransit (int position){
        // pass it a position the lift should be at
        //if current position is above, go up
        boolean done = false;
        if (position > requestedPos){
            moveStatus = 1;
            RUE();
            LSMLeft.setPower(1);
            LSMRight.setPower(1);
        }
        //if current position is below, go down
        else if (position < requestedPos){
            moveStatus = 2;
            RUE();
            LSMLeft.setPower(-1);
            LSMRight.setPower(-1);
        }
        //when you get there, hold position
       if (moveStatus == 1){
            //if moving up
            if (LSMLeft.getCurrentPosition() >= position){
                //means you are moving up and now you are done
                LSMLeft.setTargetPosition(position);
                LSMLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LSMRight.setTargetPosition(position);
                LSMRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                done = true;
                requestedPos = position;
                moveStatus = 0;
            }
        } else if (moveStatus == 2){
            //if moving down
            if (LSMLeft.getCurrentPosition() <= position){
                //means you are moving up and now you are done
                LSMLeft.setTargetPosition(position);
                LSMLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LSMRight.setTargetPosition(position);
                LSMRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                done = true;
                requestedPos = position;
                moveStatus = 0;
            }
        }
        if (done && (requestedPos == 0)){
            LSMLeft.setPower(0);
            LSMRight.setPower(0);
        }
        return done;
    }

    public void RTP (){
        if (LSMLeft.getMode()!=DcMotor.RunMode.RUN_TO_POSITION){
            LSMLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LSMRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
    public void RUE (){
        if (LSMLeft.getMode()!=DcMotor.RunMode.RUN_USING_ENCODER){
            LSMLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LSMRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public int getLeftPos(){
        return LSMLeft.getCurrentPosition();
    }

    public int getRightPos(){
        return LSMRight.getCurrentPosition();
    }

    public double getLeftPower(){
        return LSMLeft.getPower();
    }

    public double getRightPower(){
        return LSMLeft.getPower();
    }

}
