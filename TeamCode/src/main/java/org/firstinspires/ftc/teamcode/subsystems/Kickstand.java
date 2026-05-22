package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.motors.CRServo;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.util.Timing;

import java.util.concurrent.TimeUnit;

public class Kickstand {
    CRServo left, right;
    Timing.Timer timer;

    public void init(HardwareMap hwMap){
        left = hwMap.get(CRServo.class, "Kickstand Left");
        right = hwMap.get(CRServo.class, "Kickstand Right"); //lab todo one of these is reversed
        timer = new Timing.Timer(4000, TimeUnit.MILLISECONDS);
    }

    public void lift(){
        left.set(1);
        right.set(1);
    }
    public void lower(){
        left.set(-1);
        right.set(-1);
        timer.start();
    }

    public void periodic(){
        if (timer.done()) {
            left.set(0);
            right.set(0);
        }
    }

}
