package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.teamcode.robot.subsystems.Prism.GoBildaPrismDriver;

import java.util.concurrent.TimeUnit;


public class BeamBreaks {
    private DigitalChannel top;
    private DigitalChannel middle;
    private DigitalChannel bottom;
    GoBildaPrismDriver prism;

    private final Timing.Timer timer = new Timing.Timer(300, TimeUnit.MILLISECONDS);
    boolean topWasPressed, middleWasPressed, bottomWasPressed = false;


    public void init(HardwareMap hwMap){
        prism = hwMap.get(GoBildaPrismDriver.class, "prism");
        top = hwMap.get(DigitalChannel.class, "Beam Break Top");//robot todo wiring
        middle = hwMap.get(DigitalChannel.class, "Beam Break Middle");
        bottom = hwMap.get(DigitalChannel.class, "Beam Break Bottom");
    }

    public int getBallCount(){
        int ballAmount = 0;
        if (top.getState() && !topWasPressed){ //robot todo check these should be true and not false
            topWasPressed = true;
        }
        if (middle.getState() && topWasPressed && !middleWasPressed){
            middleWasPressed = true;
            timer.start();
        }
        if (middleWasPressed && topWasPressed && bottom.getState() && timer.done()){
            bottomWasPressed = true;
        }
        if (topWasPressed){
            ballAmount++;
        }
        if (middleWasPressed){
            ballAmount++;
        }
        if (bottomWasPressed){
            ballAmount++;
        }
        return ballAmount;
    }

    public void reset(){
        topWasPressed = false;
        middleWasPressed = false;
        bottomWasPressed = false;
    }
    public void turnOff(){
        prism.clearAllAnimations();
    }

    public void periodic(boolean shooting, boolean autoAim){
        int balls = getBallCount();
        if (shooting){
            prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_0); //lab todo change artboards
        } else if (autoAim) {
            prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_0);
        } else if (balls == 3){
            prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_0);
        } else if (balls == 2) {
            prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_0);
        } else if (balls == 1) {
            prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_0);
        } else if (balls == 0) {
            prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_0);
        }
    }
}
