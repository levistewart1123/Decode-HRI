package org.firstinspires.ftc.teamcode.opmode.testsandtuners;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmode.CommandOpMode;
import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp
public class AngleTester extends CommandOpMode {
    private Robot robot = new Robot();
    @Override
    public void init() {
        super.init();
        robot.init(false, hardwareMap);
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        robot.follower.update();
        telemetry.addData("loops: ", super.loops);
        telemetry.addData("angle error in degrees: ", robot.getAngleErrorDeg());
        super.loop();
        telemetry.update();
    }

    @Override
    public void stop() {
        super.stop();
    }
}
