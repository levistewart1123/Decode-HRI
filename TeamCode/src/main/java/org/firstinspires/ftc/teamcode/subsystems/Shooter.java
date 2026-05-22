package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Shooter {

    public enum States {
        AUTOMATIC,
        FIXED,
        OFF
    }
    States state = States.AUTOMATIC;

    final double GATE_OPEN_ANGLE = Math.toRadians(120);//robot todo fix these
    final double GATE_CLOSED_ANGLE = 0;
    private MotorGroup flywheels;
    private ServoEx gate;
    private ServoEx hood;
    final double FIXED_SPEED = 0.5;
    public double targetRPM = 0;

    AbsoluteAnalogEncoder gateEncoder;

    // Example


    InterpLUT velocities = new InterpLUT();
    InterpLUT angles = new InterpLUT();

    public void init(HardwareMap hwMap){
        flywheels = new MotorGroup(
                new MotorEx(hwMap, "FlywheelLeft", Motor.GoBILDA.NONE.getCPR(), 4825).setInverted(true),
                new MotorEx(hwMap, "FlywheelRight", Motor.GoBILDA.NONE.getCPR(), 4825)
        );
        flywheels.setRunMode(Motor.RunMode.VelocityControl);

        flywheels.setVeloCoefficients(0.0015, 0, 0);
        flywheels.setFeedforwardCoefficients(0, 1.45); //robot todo use recalc

        gate = new ServoEx(hwMap, "Gate", GATE_CLOSED_ANGLE, GATE_OPEN_ANGLE);
        gate = hwMap.get(ServoEx.class, "Gate");
        gate.setInverted(true);
        gateEncoder = new AbsoluteAnalogEncoder(hwMap, "Gate Encoder", 3.3, AngleUnit.RADIANS); //robot todo try out gate encoder w/telemetry

        hood = new ServoEx(hwMap, "Hood", 0, Math.toRadians(120));
        hood.setInverted(true);

        velocities.add(1, 1); //robot todo replace these and change hood range
        velocities.createLUT();
        angles.add(1, 1);
        angles.createLUT();

        state = States.AUTOMATIC;
    }

    public void openGate(){
        gate.set(0);
    }

    public void closeGate(){
        gate.set(1);
    } //robot todo make sure this works

    public boolean gateIsOpen(){
        return gateEncoder.getCurrentPosition() == GATE_OPEN_ANGLE; //robot todo try out gate encoder w/telemetry (see above)
    }
    public boolean gateIsClosed(){
        return gateEncoder.getCurrentPosition() == GATE_CLOSED_ANGLE;
    }


    public void periodic(double distance){
        if (state == States.AUTOMATIC) {
            targetRPM = velocities.get(distance);
            flywheels.set(targetRPM / flywheels.getMaxRPM());
            hood.set(angles.get(distance));
        }
    }
    public void changeState(States state){
        this.state = state;
        switch(state){
            case OFF:
                flywheels.set(0);
                break;
            case FIXED:
                flywheels.set(FIXED_SPEED);
                hood.set(0);
                break;
        }

    }


}
