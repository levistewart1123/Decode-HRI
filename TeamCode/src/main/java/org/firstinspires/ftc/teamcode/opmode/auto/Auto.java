package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.AutoPaths;
import org.firstinspires.ftc.teamcode.robot.Robot;

public class Auto extends OpMode {
    enum Step {
        MOVE_TO_SHOOT_PRELOADS,
        SHOOT_PRELOADS,
        INTAKE_MIDDLE,
        MOVE_TO_SHOOT_MIDDLE,
        SHOOT_MIDDLE,
        //etc
    }
    Step step = Step.MOVE_TO_SHOOT_PRELOADS;
    Robot robot = new Robot();
    protected boolean isRed;
    AutoPaths autoPaths;

    public Auto(boolean isRed) {
        this.isRed = isRed;
    }
    
    @Override
    public void init() {
        robot.init(isRed, hardwareMap);
        autoPaths = new AutoPaths(robot.follower, isRed, false);
        robot.setDriveStateManual(Robot.DriveState.OFF);
    }

    @Override
    public void loop() {
        switch (step){
            case MOVE_TO_SHOOT_PRELOADS:
                robot.setDriveStateAutomated(autoPaths.startToShoot);
                break;
                //etc
        }
    }
}
