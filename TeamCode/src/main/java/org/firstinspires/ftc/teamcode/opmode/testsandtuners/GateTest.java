package org.firstinspires.ftc.teamcode.opmode.testsandtuners;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmode.CommandOpMode;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;

@Configurable
@TeleOp
public class GateTest extends CommandOpMode {
    private Shooter shooter = new Shooter();
    @Override
    public void init() {
        super.init();
        shooter.init(hardwareMap);
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        if (super.loops < 3000) {
            shooter.closeGate();
        } else {
            shooter.openGate();
        }
        super.loop();
    }

    @Override
    public void stop() {
        super.stop();
    }
}
