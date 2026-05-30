package org.firstinspires.ftc.teamcode.opmode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;

@Configurable
@TeleOp
public class FlywheelTuner extends CommandOpMode{
    private Shooter shooter = new Shooter();
    public static double kP, kI, kD, kS, kV, kA, power = 0;

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
        shooter.setFlywheelPIDFCoeffs(kP, kI, kD, kS, kV, kA);
        shooter.runWithPIDF(power);
        super.loop();

    }

    @Override
    public void stop() {
        super.stop();
    }
}
