package org.firstinspires.ftc.teamcode.custom;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ArmMotor {

    public DcMotor armMot = null;
    private int armMotPos = 0;
    private int armMotPosMax = 0;
    private int armMotPosMin = 0;
    public ArmMotor(HardwareMap hwMap) {
        armMot = hwMap.dcMotor.get("armMotor");
        armMot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMot.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotPosMin = armMot.getCurrentPosition();
        armMotPosMax = armMot.getCurrentPosition() + 300;


    }

    public void armMotStickControl(double cmd){
        armMot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMot.setPower(cmd);
    }
    public void armMotHoldPos(int pos){
        armMot.setPower(1);
        armMot.setTargetPosition(pos);
        armMot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public boolean armGoToAngle(int targetTicks) {
        armMot.setPower(1);
        armMot.setTargetPosition(targetTicks);
        armMot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (targetTicks == armMot.getCurrentPosition()) {
            return true;
        } else {
            return false;
        }


    }
}
