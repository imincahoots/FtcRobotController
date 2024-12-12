package org.firstinspires.ftc.teamcode.custom;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class CrServo {
    CRServo crServo = null;
    double spitStart = 0;
    double spitEnd = 0;
    double suckStart = 0;
    double suckEnd = 0;

    public CrServo(HardwareMap hwMap) {
        crServo = hwMap.crservo.get("crServoRubberWheel");
    }

    public boolean suck (double howLong, double currentTime){
        if (suckEnd == 0){
            suckStart = currentTime;
            suckEnd = currentTime + howLong;
        }
        if (currentTime > suckStart + howLong){
            suckStart = 0;
            suckEnd = 0;
            crServo.setPower(0);
            return true;
        } else{
            crServo.setPower(-1);
            return false;
        }
    }

    public boolean spit (double howLong, double currentTime){
       if (spitEnd == 0){
           spitStart = currentTime;
           spitEnd = currentTime + howLong;
       }
       if (currentTime > spitStart + howLong){
           spitStart = 0;
           spitEnd = 0;
           crServo.setPower(0);
           return true;
       } else{
           crServo.setPower(1);
           return false;
       }
    }
    public void spitSimple (){
        crServo.setPower(1);
    }

    public boolean stop(){
        crServo.setPower(0);
        return true;
    }
}
