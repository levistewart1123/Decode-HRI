package org.firstinspires.ftc.teamcode.robot.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.controller.PIDFController;
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

    final double GATE_OPEN_ANGLE = 5;//robot todo fix these
    final double GATE_CLOSED_ANGLE = 0;
    PIDFController flywheelPIDF = new PIDFController(0, 0, 0, 100);
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
                new MotorEx(hwMap, "FlywheelLeft", 28, 4825).setInverted(true),
                new MotorEx(hwMap, "FlywheelRight", 28, 4825)
        );
        flywheels.setRunMode(Motor.RunMode.VelocityControl);

        flywheels.setVeloCoefficients(20, 0, 0);//maybe retune kP
        flywheels.setFeedforwardCoefficients(350, 1.26); //robot todo use recalc

        gate = new ServoEx(hwMap, "Gate");
        gate.setInverted(true);
//        gateEncoder = new AbsoluteAnalogEncoder(hwMap, "Gate Encoder", 3.3, AngleUnit.RADIANS); //robot todo try out gate encoder w/telemetry

        hood = new ServoEx(hwMap, "Hood", 0, Math.toRadians(120));
        hood.setInverted(true);

        velocities.add(1, 1);
        velocities.add(6, 7);//robot todo replace these and change hood range
        //!velocities.createLUT();
        angles.add(0, 0);
        angles.add(50, 0);
        angles.add(120, 1);
        angles.add(200, 1);
        angles.createLUT();

        state = States.AUTOMATIC;
    }
    public double getFlywheelVelocity(){
        return flywheels.getVelocity();
    }

    public void openGate(){
        gate.set(0);
    }
    public void closeGate(){
        gate.set(0.85);
    } //robot todo make sure this works

    public Command open = instant(() -> gate.set(0));
    public Command close = instant(() -> gate.set(0.85));
    public boolean gateIsOpen(){
        return gate.get() == 0; //robot todo try out gate encoder w/telemetry (see above)
    }
    public boolean gateIsClosed(){
        return gate.get() == 1;
    }


    public void periodic(double distance){
        targetRPM = velocities.get(distance);
        flywheels.set(targetRPM / flywheels.getMaxRPM());
        hood.set(angles.get(distance));

    }
    public void autoHood(double distance) {
        hood.set(angles.get(distance));
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
    public void setHood(double position){
        hood.set(position);
    }
    public void runNoPIDF(double power){
        flywheels.set(power); //!bad
    }
    public void runWithPIDF(double power){ //-1 to 1
        flywheels.set(power);
    }
    public void setFlywheelCoeffs(double kP, double kI, double kD, double kS, double kV, double kA){ //0.55, 55.90
        flywheels.setVeloCoefficients(kP, kI, kD);
        flywheels.setFeedforwardCoefficients(kS, kV, kA);
    }


    //gyrobotic droids' shooting code
    public Vector calculateShotVector(double robotHeading){
    double g = 32.174 * 12;
    double x = robotToGoalVector.getMagnitude() - ShooterConstants.PASS_THROUGH_POINT_RADIUS;
    double y = ShooterConstants.SCORE_HEIGHT;
    double a = ShooterConstants.SCORE_ANGLE;

    //calculate initial launch components
    double hoodAngle = MathFunctions.clamp(Math.atan(2 * y / x - Math.tan(a)), ShooterConstants.HOOD_MAX_ANGLE,
            ShooterConstants.HOOD_MIN_ANGLE);

    double flywheelSpeed = Math.sqrt(g * x * x / (2 * Math.pow(Math.cos(hoodAngle), 2) * (x * Math.tan(hoodAngle) - y)));

    //get robot velocity and convert it into parallel and perpendicular components
    Vector robotVelocity = hardware.poseTracker.getVelocity();

    double coordinateTheta = robotVelocity.getTheta() - robotToGoalVector.getTheta();

    double parallelComponent = -Math.cos(coordinateTheta) * robotVelocity.getMagnitude();
    double perpendicularComponent = Math.sin(coordinateTheta) * robotVelocity.getMagnitude();

    //velocity compensation variables
    double vz = flywheelSpeed * Math.sin(hoodAngle);
    double time = x / (flywheelSpeed * Math.cos(hoodAngle));
    double ivr = x / time + parallelComponent;
    double nvr = Math.sqrt(ivr * ivr + perpendicularComponent * perpendicularComponent);
    double ndr = nvr * time;

    //recalculate launch components
    hoodAngle = MathFunctions.clamp(Math.atan(vz / nvr), ShooterConstants.HOOD_MAX_ANGLE,
    ShooterConstants.HOOD_MIN_ANGLE);

    flywheelSpeed = Math.sqrt(g * ndr * ndr / (2 * Math.pow(Math.cos(hoodAngle), 2) * (ndr * Math.tan(hoodAngle) - y)));

    //update turret
    double turretVelCompOffset = Math.atan(perpendicularComponent / ivr);
    double turretAngle = Math.toDegrees(robotHeading - robotToGoalVector.getTheta() + turretVelCompOffset);


        return new Vector(flywheelSpeed, hoodAngle);
}


}
