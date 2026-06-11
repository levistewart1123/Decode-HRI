package org.firstinspires.ftc.teamcode.opmodes.testsAndTuners;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.CommandOpMode;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;


@TeleOp(name = "Hood Fixer", group = "1: tuners")
public class HoodFixer extends CommandOpMode {
    private Shooter shooter = new Shooter();
    @Override
    public void init() {
        super.init();
        shooter.initialize(hardwareMap);
        telemetry.addLine("Hi, I'm going to walk you through fixing the hood if its position is off.");
        telemetry.addLine("make sure the hood is lifted so that the gear doesn't mesh, then hit start to continue.");
        telemetry.addLine("if you're running this to test if you did it correctly, leave the hood where it is.");
        telemetry.update();
    }

    @Override
    public void start() {
        super.start();
        shooter.setHood(1);
    }

    @Override
    public void loop() {
        if (super.loops > 3000) {
            shooter.setHood(0.25);
            telemetry.addLine("Right now, the hood is set to all the way down." +
                    "\nUse this to test if it is in the right position" +
                    "\nif the gears mesh. if not, just wait.");
        } else {
            shooter.setHood(1);
            telemetry.addLine("now, the hood is set to all the way up." +
                    "\nIf you're trying to get the hood back to the" +
                    "\nright position, read this, then turn off the program," +
                    "\nand turn the gear such that the largest black mark on one tooth" +
                    "\nwill align with the 2nd empty space on the hood (marked in black)." +
                    "\nthen mesh the gears and restart the program" +
                    "\nto make sure you did it correctly.");
        }
        super.loop();
    }

    @Override
    public void stop() {
        super.stop();
    }
}
