package org.firstinspires.ftc.teamcode.opmodes.testsAndTuners;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.CommandOpMode;
import org.firstinspires.ftc.teamcode.robot.Robot;

/**
 * for autoaim testing
 */
@TeleOp(name = "Autoaim Test", group = "2: tests")
public class OdometryAngleTester extends CommandOpMode {
    private Robot robot = new Robot();
    @Override
    public void init() {
        super.init();
        robot.initialize(false, hardwareMap);
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        robot.follower.update();
        telemetry.addData("loops: ", super.loops);
        telemetry.addData("angle error in degrees: ", robot.getOdoAngleErrorDeg());
        super.loop();
    }

    @Override
    public void stop() {
        super.stop();
    }
}
