package org.firstinspires.ftc.teamcode.opmodes.auto;

import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import org.firstinspires.ftc.teamcode.opmodes.CommandOpMode;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystems.HuskyLens;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;

import java.util.function.BooleanSupplier;


public class FarAuto extends CommandOpMode {

    Command humanPlayerZoneTo, humanPlayerZoneBack;
    Robot robot = new Robot();
    Intake intake = new Intake();
    protected boolean isRed;
    AutoPaths autoPaths = new AutoPaths(true,true);
    BooleanSupplier shooting = () -> robot.isShooting;

    public FarAuto(boolean isRed) {
        this.isRed = isRed;
    }
    
    public Command farAuto() {
        return sequential(
                //! add Flywheel spinup
                follow(robot.follower, autoPaths.startToShoot, true),
                //waitUntil(robot.shooter.getFlywheelVelocity()> robot.shooter.targetRPM), //! Flywheel to be at speed
                robot.fastShoot, //* may need to be slow Shoot
                intake.setIn,
                follow(robot.follower, autoPaths.shootToSpikeMarkBottom),
                intake.run(0.5),
                follow(robot.follower, autoPaths.spikeMarkBottomToShoot),
                robot.fastShoot, //* may need to be slow Shoot
                //waitUntil(robot.isShooting == false), //! robot.isShooting is not BooleanSupplier
                intake.setIn,
                follow(robot.follower, autoPaths.sideDetermineToFarLowHPCollect),
                intake.run(0.5),
                follow(robot.follower, autoPaths.farLowHPCollectToShoot),
                robot.fastShoot, //* may need to be slow Shoot
                follow(robot.follower, autoPaths.shootToSideDetermine),
                determineSide(),//TODO make rotate to 90 to get reading
                intake.setIn,
                humanPlayerZoneTo,
                intake.run(0.5),
                humanPlayerZoneBack,
                robot.fastShoot, //* may need to be slow Shoot
                follow(robot.follower, autoPaths.shootToSideDetermine),
                determineSide(),//TODO make rotate to 90 to get reading
                intake.setIn,
                humanPlayerZoneTo,
                intake.run(0.5),
                humanPlayerZoneBack,
                robot.fastShoot, //* may need to be slow Shoot
                follow(robot.follower, autoPaths.shootToSideDetermine),
                determineSide(),//TODO make rotate to 90 to get reading
                intake.setIn,
                humanPlayerZoneTo,
                intake.run(0.5),
                humanPlayerZoneBack,
                robot.fastShoot //* may need to be slow Shoot
                );
    }

    public Command determineSide() {
        double sNumber = HuskyLens.sideNumber();
        if (sNumber == -1){// high
            humanPlayerZoneTo = follow(robot.follower, autoPaths.sideDetermineToFarHighHPCollect);
            humanPlayerZoneBack = follow(robot.follower, autoPaths.farHighHPCollectToShoot, true);
        } else { // low
            humanPlayerZoneTo = follow(robot.follower, autoPaths.sideDetermineToFarLowHPCollect);
            humanPlayerZoneBack = follow(robot.follower, autoPaths.farLowHPCollectToShoot, true);
        }
        return null;
    }

    @Override
    public void init() {
        super.init();
        robot.initialize(true, hardwareMap);
    }

    @Override
    public void start() {
        schedule(farAuto());
        super.start();
    }

    @Override
    public void loop() {
        telemetry.addData("x", robot.follower.getPose().getX());
        telemetry.addData("y", robot.follower.getPose().getY());
        telemetry.addData("heading", robot.follower.getPose().getHeading());
        super.loop();
    }

    @Override
    public void stop() {
        super.stop();
    }
}