package org.firstinspires.ftc.teamcode.opmode.testsandtuners;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmode.CommandOpMode;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;

@Configurable
@TeleOp
public class FlywheelTuner extends CommandOpMode {
    public static double velocity;
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
        shooter.setFlywheelCoeffs(kP, kI, kD, kS, kV, kA);
        shooter.runWithPIDF(power);
        velocity = shooter.getFlywheelVelocity();
        telemetry.addData("velocity: ", velocity);
        super.loop();


    }

    @Override
    public void stop() {
        super.stop();
    }
}
