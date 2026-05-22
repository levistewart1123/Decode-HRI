package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 *
 */
public class TeleOp extends OpMode {
    protected Robot robot = new Robot();
    protected boolean red;

    public TeleOp(boolean red) {
        this.red = red;
    }

    @Override
    public void init() {
        robot.init(red, hardwareMap);
    }

    @Override
    public void start() {
        robot.f.startTeleOpDrive();
    }

    @Override
    public void loop() {
        robot.periodic(gamepad1);
        if (gamepad1.bWasPressed()){
            robot.toggleAiming();
        }
        if (gamepad1.xWasPressed()){
            robot.startShoot();
        }
        if (gamepad1.aWasPressed()){
            robot.slowDrive = !robot.slowDrive;
        }
        //lab todo add automated drive controls from old code

    }
}
