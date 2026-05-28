package org.firstinspires.ftc.teamcode.tests;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;

public class FakeIntake {
    private FakeMotor intake;


    public void init() {
        intake = new FakeMotor();
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

    public double getSpeed(){
        return intake.get();
    }

    public Command setIn = instant(() -> intake.set(1));
    public Command setOut = instant(() -> intake.set(-1));
    public Command turnOff = instant(() -> intake.set(0));
    public Command run(int power){
        return instant(() -> intake.set(power));
    }
}
