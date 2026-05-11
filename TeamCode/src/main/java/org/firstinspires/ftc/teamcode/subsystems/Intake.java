package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

public class Intake {
    private MotorEx intake;


    public void init(HardwareMap hwMap) {
        intake = new MotorEx(hwMap, "Intake", Motor.GoBILDA.RPM_1150);
        intake.setCachingTolerance(0.02); //double check this is ok
    }

    public void spinIn(){
        intake.set(1);
    }
    public void spinOut(){
        intake.set(-1);
    }
    public void stop(){
        intake.set(0);
    }

}
