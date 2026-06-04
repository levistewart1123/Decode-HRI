package org.firstinspires.ftc.teamcode.opmode.testsandtuners;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmode.CommandOpMode;
import org.firstinspires.ftc.teamcode.robot.subsystems.BeamBreaks;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;

@Configurable
@TeleOp
public class BBTest extends CommandOpMode {
    private BeamBreaks beamBreaks = new BeamBreaks();
    @Override
    public void init() {
        super.init();
        beamBreaks.init(hardwareMap);
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        beamBreaks.periodic(false, false);
        super.loop();
    }

    @Override
    public void stop() {
        super.stop();
    }
}
