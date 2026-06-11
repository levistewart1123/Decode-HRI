package org.firstinspires.ftc.teamcode.opmodes.testsAndTuners;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.CommandOpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;

@Configurable
@TeleOp(name = "Auto RPM Tuner", group = "1: tuners")
public class AutoRPMTuner extends CommandOpMode {
    public static double power = 0;
    public static double hoodPos = 0.25;
    private Robot robot = new Robot();

    @Override
    public void init() {
        super.init();
        robot.initialize(true, hardwareMap);
    }

    @Override
    public void start() {
        robot.shooter.runWithPIDF(power);
        robot.follower.setPose(new Pose(72, 72, 0));
        robot.handleIntake.schedule();
        super.start();
    }

    @Override
    public void loop() {
        robot.shooter.runWithPIDF(power);
        telemetry.addData("Power: ", power);
        telemetry.addData("Distance: ", robot.getDistToGoal());
        if (gamepad1.right_trigger > 0.1){
            robot.setIntakeState(Robot.IntakeState.IN);
        } else if (gamepad1.left_trigger > 0.1) {
            robot.setIntakeState(Robot.IntakeState.OUT);
        } else {
            robot.setIntakeState(Robot.IntakeState.OFF);
        }
//        robot.shooter.autoHood(robot.getDistToGoal());
        robot.shooter.setHood(hoodPos);
        super.loop();
        robot.follower.update();
    }

    @Override
    public void stop() {
        super.stop();
    }
}
