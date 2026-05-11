package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

public class Shooter {
    private MotorGroup flywheels;
    private ServoEx gate;
    private ServoEx hood;

    public void init(HardwareMap hwMap){
        flywheels = new MotorGroup(
                new MotorEx(hwMap, "FlywheelLeft", Motor.GoBILDA.NONE).setInverted(true),
                new MotorEx(hwMap, "FlywheelRight", Motor.GoBILDA.NONE)
        );
        flywheels.setRunMode(Motor.RunMode.VelocityControl);
        flywheels.setVeloCoefficients(0.0015, 0, 0);
        flywheels.setFeedforwardCoefficients(0, 1.45);

        gate = hwMap.get(ServoEx.class, "Gate");
        gate.setInverted(true);

        hood = hwMap.get(ServoEx.class, "Hood");
        hood.setInverted(true);
    }

}
