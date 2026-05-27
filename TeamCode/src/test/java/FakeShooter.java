import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;

public class FakeShooter {


    public enum States {
            AUTOMATIC,
            FIXED,
            OFF

        }
    States state = States.AUTOMATIC;
    final double GATE_OPEN_ANGLE = Math.toRadians(120);//robot todo fix these
        final double GATE_CLOSED_ANGLE = 0;
        private FakeMotor flywheels;
        private FakeServo gate;
        private FakeServo hood;
        final double FIXED_SPEED = 0.5;
        public double targetRPM = 0;

        //AbsoluteAnalogEncoder gateEncoder;

        // Example


        InterpLUT velocities = new InterpLUT();
        InterpLUT angles = new InterpLUT();

        public void init(){
            flywheels = new FakeMotor();

            gate = new FakeServo();
            //gateEncoder = new AbsoluteAnalogEncoder(hwMap, "Gate Encoder", 3.3, AngleUnit.RADIANS); //robot todo try out gate encoder w/telemetry

            hood = new FakeServo();

            velocities.add(1, 1); //robot todo replace these and change hood range
            //velocities.createLUT();
            angles.add(1, 1);
            //angles.createLUT();

            state = States.AUTOMATIC;
        }

        public void openGate(){
            gate.set(0);
        }
        public void closeGate(){
            gate.set(1);
        } //robot todo make sure this works

        public Command open = instant(() -> gate.set(0));
        public Command close = instant(() -> gate.set(1));
        public boolean gateIsOpen(){
            return gate.get() == 0; //robot todo try out gate encoder w/telemetry (see above)
        }
        public boolean gateIsClosed(){
            return gate.get() == 1;
        }


        public void periodic(double distance){
            if (state == States.AUTOMATIC) {
                targetRPM = velocities.get(distance);
                flywheels.set(67);
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
