package org.firstinspires.ftc.teamcode.opmodes.auto;

import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystems.HuskyLens;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;

import java.util.function.BooleanSupplier;


public class Auto{

    Command humanPlayerZoneTo, humanPlayerZoneBack;
    Robot robot = new Robot();
    Intake intake = new Intake();
    protected boolean isRed;
    AutoPaths autoPaths = new AutoPaths(robot.follower,true,true);
    BooleanSupplier shooting = () -> robot.isShooting;

    public Auto(boolean isRed) {
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
}