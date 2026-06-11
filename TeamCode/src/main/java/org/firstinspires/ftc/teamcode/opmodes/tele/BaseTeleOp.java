package org.firstinspires.ftc.teamcode.opmodes.tele;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.behaviors.BlockedBehavior;
import com.pedropathing.ivy.behaviors.ConflictBehavior;
import com.pedropathing.ivy.behaviors.InterruptedBehavior;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.opmodes.CommandOpMode;
import org.firstinspires.ftc.teamcode.PoseSaver;
import org.firstinspires.ftc.teamcode.robot.Robot;

import java.util.function.Supplier;

/**
 *This is our base TeleOp class.
 * Red and Blue TeleOps that extend this should be created and put on the driver station.
 */
public class BaseTeleOp extends CommandOpMode {
    protected Robot robot = new Robot();

    protected boolean isRed;

    protected Pose closeShootPose = new Pose(39, 53, Math.toRadians(270));
    protected Pose gatePose = new Pose(144 - 14.4, 58.2, Math.toRadians(180 - 144.9));
    protected Pose farOpposingShootPose = new Pose(83, 15, Math.toRadians(180 - 110.1));
    protected Pose farAlliedShootPose = new Pose(57, 15, Math.toRadians(180 - 121.1));
    protected Pose farOpposingIntakePose = new Pose(144 - 12.0, 9.25, Math.toRadians(0));
    protected Pose farAlliedIntakePose = new Pose(12.0, 9.25, Math.toRadians(180));
    private Supplier<PathChain> center;
    private Supplier<PathChain> pickupGate;
    private Supplier<PathChain> farOpposingShoot; //used
    private Supplier<PathChain> farAlliedShoot;
    private Supplier<PathChain> farOpposingIntake;
    private Supplier<PathChain> farAlliedIntake;

    public BaseTeleOp(boolean isRed) {
        this.isRed = isRed;
    }

    @Override
    public void init() {
        robot.initialize(isRed, hardwareMap);
        if (!isRed) {
            closeShootPose.mirror();
            gatePose.mirror();
            farOpposingShootPose.mirror();
            farAlliedShootPose.mirror();
            farOpposingIntakePose.mirror();
            farAlliedIntakePose.mirror();
        }
        center = () -> robot.follower.pathBuilder()
                .addPath(new Path(new BezierLine(robot.follower::getPose, new Pose(72, 72, Math.toRadians(0)))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(robot.follower::getHeading, Math.toRadians(0), 0.8))
                .build();
        pickupGate = () -> robot.follower.pathBuilder()
                .addPath(new Path(new BezierLine(robot.follower::getPose, gatePose)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(robot.follower::getHeading, gatePose.getHeading(), 0.8))
                .build();
        farOpposingShoot = () -> robot.follower.pathBuilder()
                .addPath(new Path(new BezierLine(robot.follower::getPose, farOpposingShootPose)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(robot.follower::getHeading, farOpposingShootPose.getHeading(), 0.8))
                .build();
        farAlliedShoot = () -> robot.follower.pathBuilder()
                .addPath(new Path(new BezierLine(robot.follower::getPose, farAlliedShootPose)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(robot.follower::getHeading, farAlliedShootPose.getHeading(), 0.8))
                .build();
        farOpposingIntake = () -> robot.follower.pathBuilder()
                .addPath(new Path(new BezierLine(robot.follower::getPose, farOpposingIntakePose)))
                .setConstantHeadingInterpolation(farOpposingIntakePose.getHeading())
                .build();
        farAlliedIntake = () -> robot.follower.pathBuilder()
                .addPath(new Path(new BezierLine(robot.follower::getPose, farAlliedIntakePose)))
                .setConstantHeadingInterpolation(farAlliedIntakePose.getHeading()) //this might be sketchy if not constant w/a known value
                .build();

        reset();
    }

    @Override
    public void start() {
        super.start();
        robot.update(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        schedule(robot.startManualDrive);
        schedule(robot.handleGate);
        schedule(robot.handleIntake);
    }

    @Override
    public void loop() {
        robot.update(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        //*autoaim
        if (gamepad1.bWasPressed()){
            robot.autoAiming = !robot.autoAiming;
        }
        //*shooting
        if (gamepad1.xWasPressed() && (robot.shooter.getFlywheelVelocity()>0.1)){
            schedule(robot.slowShoot);
        }
        //*slow mode
        if (gamepad1.aWasPressed()){
            robot.slowDrive = !robot.slowDrive;
        }
        //todo path following, make individual commands in Robot class to set priority, etc
        if (gamepad1.yWasPressed()){
            schedule(follow(robot.follower, center.get())
                    .setBlockedBehavior(BlockedBehavior.QUEUE)
                    .setConflictBehavior(ConflictBehavior.OVERRIDE)
                    .setInterruptedBehavior(InterruptedBehavior.END)
                    .setPriority(1)
                    .requiring(robot.follower))

            ;
        }
        //*intake
        if (gamepad1.right_trigger > 0.1){
            robot.setIntakeState(Robot.IntakeState.IN);
        } else if (gamepad1.left_trigger > 0.1) {
            robot.setIntakeState(Robot.IntakeState.OUT);
        } else {
            robot.setIntakeState(Robot.IntakeState.OFF);
        }
        //*close/far toggling
        if (gamepad1.yWasPressed()){
            robot.shooter.setClose(true);
        }
        if (gamepad1.xWasPressed()){
            robot.shooter.setClose(false);
        }
        //*telemetry
        telemetry.addData("angle error: ", (robot.getOdoAngleErrorDeg()));
        telemetry.addData("angle: ", (Math.toDegrees(robot.follower.getPose().getHeading())));
        telemetry.addLine(Scheduler.isRunning(robot.shoot) ? "shooting" : "not shooting");
        telemetry.addLine(Scheduler.isRunning(robot.handleIntake) ? "manual intake" : "not manual intake");
        telemetry.addLine(Scheduler.isRunning(robot.handleGate) ? "auto gate" : "not auto gate");
        telemetry.addLine(Scheduler.isRunning(robot.startManualDrive) ? "manual drive" : "not manual drive");
        telemetry.addData("distance to goal: ", robot.getDistToGoal());
        //lab todo add automated drive controls from old code

        super.loop(); //runs CommandOpMode's loop
    }

    public void stop(){
        PoseSaver.autoWasRun = false;
        super.stop();
    }
}
