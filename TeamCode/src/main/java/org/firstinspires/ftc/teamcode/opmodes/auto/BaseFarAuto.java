package org.firstinspires.ftc.teamcode.opmodes.auto;

import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.lazy;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.repeat;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.pedro.PedroCommands.turnTo;
import static com.pedropathing.ivy.groups.Groups.race;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.opmodes.CommandOpMode;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystems.HuskyLens;


public class BaseFarAuto extends CommandOpMode {
    protected Robot robot = new Robot();
    protected boolean isRed;
    boolean lowHPPickup = true;
    public BaseFarAuto(boolean isRed){
        this.isRed = isRed;
    }
    public Pose start, shoot, farLowHPCollect, farHighHPCollect, spikeMarkBottom;
    public PathChain startToShoot, sideDetermineToFarLowHPCollect, sideDetermineToFarHighHPCollect, farHighHPCollectToShoot, farLowHPCollectToShoot, shootToSpikeMarkBottom,spikeMarkBottomToShoot, shootToSideDetermine;
    protected Command startIntaking, prepareShoot, humanPlayerZoneTo, humanPlayerZoneBack, determineSide;

    public int sideDetermineheading;


    public void buildPaths(boolean isRed) {
        farLowHPCollect = new Pose(15, 7, Math.toRadians(183));
        farHighHPCollect = new Pose(10,25.000, Math.toRadians(180));
        spikeMarkBottom = new Pose(11.6, 40, Math.toRadians(180));

        start = new Pose(55.6, 7.2, Math.toRadians(90));
        shoot = new Pose(56.9, 20.6, Math.toRadians(119.5));
        sideDetermineheading = 180;
        //side determine heading 180

        //Pose FinalShoot = new Pose(47.4,115.0, Math.toRadians(150.4));
        if (isRed) {
            start = start.mirror();
            shoot = shoot.mirror();
            farLowHPCollect = farLowHPCollect.mirror();
            farHighHPCollect = farHighHPCollect.mirror();
            spikeMarkBottom = spikeMarkBottom.mirror();
            sideDetermineheading = 0;
        }

        startToShoot = robot.follower.pathBuilder()
                .addPath(new BezierLine(
                        start,
                        shoot
                ))
                .setLinearHeadingInterpolation(start.getHeading(), shoot.getHeading())
                .build();
        shootToSpikeMarkBottom = robot.follower.pathBuilder()
                .addPath(new BezierLine(
                        shoot,
                        spikeMarkBottom
                ))
                .setLinearHeadingInterpolation(shoot.getHeading(), spikeMarkBottom.getHeading())
                .build();
        spikeMarkBottomToShoot = robot.follower.pathBuilder()
                .addPath(new BezierLine(
                        spikeMarkBottom,
                        shoot
                ))
                .setLinearHeadingInterpolation(spikeMarkBottom.getHeading(), shoot.getHeading())
                .build();
        sideDetermineToFarLowHPCollect = robot.follower.pathBuilder()
                .addPath(new BezierLine(
                        shoot,
                        farLowHPCollect
                ))
                .setLinearHeadingInterpolation(Math.toRadians(sideDetermineheading), farLowHPCollect.getHeading())
                .build();
        sideDetermineToFarHighHPCollect = robot.follower.pathBuilder()
                .addPath(new BezierLine(
                        shoot,
                        farHighHPCollect
                ))
                .setLinearHeadingInterpolation(Math.toRadians(sideDetermineheading), farHighHPCollect.getHeading())
                .build();
        farHighHPCollectToShoot = robot.follower.pathBuilder()
                .addPath(new BezierLine(
                        farHighHPCollect,
                        shoot
                ))
                .setLinearHeadingInterpolation(farHighHPCollect.getHeading(), shoot.getHeading())
                .build();
        farLowHPCollectToShoot = robot.follower.pathBuilder()
                .addPath(new BezierLine(
                        farLowHPCollect,
                        shoot
                ))
                .setLinearHeadingInterpolation(farLowHPCollect.getHeading(), shoot.getHeading())
                .build();
        startIntaking = parallel(
                robot.shooter.close,
                robot.intake.setIn
        );
        prepareShoot = sequential(
                robot.intake.turnOff,
                waitMs(500),
                robot.shooter.open
        );

        humanPlayerZoneTo = conditional(
                () -> lowHPPickup,
                follow(robot.follower, sideDetermineToFarLowHPCollect),
                follow(robot.follower, sideDetermineToFarHighHPCollect)
        );
        humanPlayerZoneBack = conditional(
                () -> lowHPPickup,
                follow(robot.follower, farLowHPCollectToShoot, true),
                humanPlayerZoneBack = follow(robot.follower, farHighHPCollectToShoot, true)
        );
        determineSide = lazy(() -> {
                    double sNumber = HuskyLens.sideNumber();
                    return instant(() -> lowHPPickup = (sNumber != -1));
                }
        );
    }

    public Command jiggle = sequential(
            robot.intake.run(1),
            instant(() -> {
                robot.follower.startTeleOpDrive();
                robot.follower.setTeleOpDrive(-0.8, 0, 0);
            }),
            waitMs(500),
            robot.intake.run(1),
            instant(() -> robot.follower.setTeleOpDrive(0.6, 0, 0)),
            waitMs(800),
            robot.intake.run(0),
            robot.driveOff
    );

    @Override
    public void init() {
        super.init();
        robot.initialize(isRed, hardwareMap);
        robot.shooter.closeMode = false;
        telemetry.addData("follower is null?", robot.follower == null);
        telemetry.update();
        buildPaths(isRed);
        robot.follower.setPose(start);
    }

    @Override
    public void start() {
        schedule(sequential(
                        //! add Flywheel spinup
                        robot.shooter.open,
                        //waitUntil(robot.fastShooter.getFlywheelVelocity()> robot.fastShooter.targetRPM), //! Flywheel to be at speed
                        parallel(waitMs(3000), follow(robot.follower, startToShoot, true)),
                        robot.fastShoot, //* may need to be slow Shoot
                        robot.intake.setIn,
                        follow(robot.follower, shootToSpikeMarkBottom),
                        robot.intake.run(0),
                        parallel(follow(robot.follower, spikeMarkBottomToShoot), prepareShoot),
                        robot.shooter.open,
                        robot.fastShoot, //* may need to be slow Shoot
                        //waitUntil(robot.isShooting == false), //! robot.isShooting is not BooleanSupplier
                        robot.intake.setIn,
                        race(follow(robot.follower, sideDetermineToFarLowHPCollect, 0.8),waitMs(3000)),

                        jiggle,
                        parallel(follow(robot.follower, farLowHPCollectToShoot), prepareShoot),
                        robot.shooter.open,
                        robot.fastShoot, //* may need to be slow Shoot

                        turnTo(robot.follower, Math.toRadians(sideDetermineheading)),
                        determineSide,
                        robot.intake.setIn,
                        race(humanPlayerZoneTo, waitMs(2000)),
                        jiggle,
                        robot.intake.run(0),
                        parallel(humanPlayerZoneBack, prepareShoot),
                        robot.shooter.open,
                        robot.fastShoot, //* may need to be slow Shoot

                        turnTo(robot.follower, Math.toRadians(sideDetermineheading)),
                        determineSide,
                        robot.intake.setIn,
                        race(humanPlayerZoneTo, waitMs(2000)),
                        jiggle,
                        parallel(humanPlayerZoneBack, prepareShoot),
                        robot.shooter.open,
                        robot.fastShoot //* may need to be slow Shoot
                )
        );
        super.start();
    }

    @Override
    public void loop() {
        robot.update(0,0,0);

        super.loop();
    }

    @Override
    public void stop() {
        super.stop();
    }
}
