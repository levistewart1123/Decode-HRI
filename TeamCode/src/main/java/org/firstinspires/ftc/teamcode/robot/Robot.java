package org.firstinspires.ftc.teamcode.robot;

import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.seattlesolvers.solverslib.util.MathUtils.normalizeAngle;

import static java.lang.Math.abs;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.BlockedBehavior;
import com.pedropathing.ivy.behaviors.ConflictBehavior;
import com.pedropathing.ivy.behaviors.InterruptedBehavior;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.PoseSaver;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.subsystems.BeamBreaks;
import org.firstinspires.ftc.teamcode.robot.subsystems.HuskyLens;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Kickstand;
import org.firstinspires.ftc.teamcode.robot.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;


import java.util.List;

/**
 * This holds all of our subsystem classes and puts together Commands using them.
 */
@Configurable
public class Robot {
    public enum IntakeState {
        IN,
        OUT,
        OFF
    }

    public IntakeState intakeState = IntakeState.OFF;

    public Intake intake = new Intake();
    public Shooter shooter = new Shooter();
    public HuskyLens huskyLens = new HuskyLens();
    public Follower follower;
    public BeamBreaks beamBreaks = new BeamBreaks();
    public Kickstand kickstand = new Kickstand();
    public Limelight limelight = null;
    public boolean autoAiming = false;
    public boolean limelightAim = false;
    public Pose goalPose;
    public Pose redGoal = new Pose(134, 139);
    public Pose humanPZ;
    public Pose redHPZ = new Pose(8, 11.5, 0);
    double forwardInput, rightInput, rotateInput = 0;
    public boolean isShooting = false;
    public boolean slowDrive = false;
    public static double headingKP = 0.025;
    public static double headingKI = 0;
    public static double headingKD = 0.02;
    public static double headingKF = 0.025;
    public boolean isRed;
    private double savedOdoAngleDeg;


    //*movement commands

    public Command aimAndStoreHeading() {
        return Command.build()
                .setStart(() -> {
                    autoAiming = true;
                    limelightAim = false;
                })
                .setDone(() -> abs(getOdoAngleErrorDeg()) < 0.5) //doesn't account for overshoot
                .setEnd((endCondition) -> {
                    autoAiming = false;
                    savedOdoAngleDeg = Math.toDegrees(follower.getPose().getHeading());
                })
                .requiring(follower)
                ;
    }
    public Command correctHeadingWithLimelight() {
        return Command.build()
                .setStart(() -> {
                    autoAiming = true;
                    limelightAim = true;
                })
                .setDone(() -> abs(limelight.getTx()) < 0.5) //doesn't account for overshoot
                .setEnd((endCondition) -> {
                    autoAiming = false;
                    follower.setHeading(savedOdoAngleDeg);
                })
                .requiring(follower)
                ;
    }
    public Command correctHeading = sequential(
            aimAndStoreHeading(),
            correctHeadingWithLimelight()
    )
            .requiring(follower)
            .setPriority(2)
            .setConflictBehavior(ConflictBehavior.OVERRIDE)
            ;
    public Command handleDriveInput = infinite(() -> {
        if (autoAiming) {
            if (limelightAim) {
                follower.setTeleOpDrive(forwardInput, rightInput, getAimingPIDFOutput(getOdoAngleErrorDeg()));
            } else {
                follower.setTeleOpDrive(forwardInput, rightInput, getAimingPIDFOutput(limelight.getTx()));
            }
        } else {
            follower.setTeleOpDrive(forwardInput, rightInput, rotateInput);
        }
    });
    public Command driveOff = instant(() -> follower.setTeleOpDrive(0, 0, 0));
    Command startTeleOpDrive = instant(() -> follower.startTeleOpDrive());

    public Command startManualDrive = sequential(
            startTeleOpDrive,
            handleDriveInput
    )
            .requiring(follower)
            .setPriority(0)
            .setInterruptedBehavior(InterruptedBehavior.SUSPEND)
            .setConflictBehavior(ConflictBehavior.QUEUE)
            .setBlockedBehavior(BlockedBehavior.QUEUE);

    //*shooting commands
    Command setShooting(boolean shooting) {
        return instant(() -> isShooting = shooting);
    }

    Command setAiming(boolean aiming) {
        return instant(() -> autoAiming = aiming);
    }

    Command fastShoot = sequential(
            driveOff,
            setShooting(true),
            intake.setIn,
            waitMs(700),
            intake.turnOff,
            shooter.close,
            setShooting(false),
            setAiming(false)
    )
            .requiring(intake, follower, shooter)
            .setPriority(2);
    public Command slowShoot = sequential(
            driveOff,
            setShooting(true),
            intake.turnOff,
            shooter.open,
            waitMs(300), //robot todo change to waitUntil(gateIsOpen) once it's working
            intake.setIn,
            waitMs(700),
            intake.turnOff,
            shooter.close,
            setShooting(false),
            setAiming(false)
    )
            .requiring(intake, follower, shooter)
            .setPriority(2);
    public Command shoot = conditional(
            () -> false, //!fixme
            fastShoot,
            slowShoot
    )
            .requiring(intake, follower, shooter)
            .setPriority(2);
    //*other shooter commands
    public Command handleGate = infinite(() -> {
//                if (beamBreaks.getBallCount() == 3) {
//                    shooter.openGate();
//                } else {
                shooter.closeGate();
                //}
            }
    )
            .requiring(shooter)
            .setPriority(0)
            .setInterruptedBehavior(InterruptedBehavior.SUSPEND)
            .setBlockedBehavior(BlockedBehavior.QUEUE)
            .setConflictBehavior(ConflictBehavior.QUEUE);
    public Command toggleClose;

    public Command handleIntake = infinite(
            () -> {
                switch (intakeState) {
                    case IN:
                        intake.spinIn();
                        break;
                    case OUT:
                        intake.spinOut();
                        break;
                    case OFF:
                        intake.stop();
                        break;
                }
            }
    )
            .requiring(intake)
            .setPriority(0)
            .setInterruptedBehavior(InterruptedBehavior.SUSPEND)
            .setBlockedBehavior(BlockedBehavior.QUEUE)
            .setConflictBehavior(ConflictBehavior.QUEUE);

    public void setIntakeState(IntakeState intakeState) {
        if (this.intakeState != intakeState) {
            this.intakeState = intakeState;
        }
    }


    public void initialize(boolean isRed, HardwareMap hwMap) {
        List<LynxModule> allHubs = hwMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO); //we can try setting this to manual and see how much loop times improve
        }

        follower = Constants.createFollower(hwMap);
        intake.initialize(hwMap);
        shooter.initialize(hwMap);
        huskyLens.initialize(hwMap);
        beamBreaks.initialize(hwMap);
        limelight.initialize(hwMap);

        //kickstand.init(hwMap);
        this.isRed = isRed;
        if (isRed) {
            goalPose = redGoal;
            humanPZ = redHPZ;
            limelight.setPipeline(0); //!check
        } else {
            goalPose = redGoal.mirror();
            humanPZ = redHPZ.mirror();
            limelight.setPipeline(1);
        }
        if (PoseSaver.autoWasRun) {
            follower.setStartingPose(PoseSaver.endPose);
        } else {
            follower.setStartingPose(humanPZ);
        }
        PoseSaver.autoWasRun = false;
        follower.update();
    }

    public double getDistToGoal() {
        double xDiff = follower.getPose().getX() - goalPose.getX();
        double yDiff = follower.getPose().getY() - goalPose.getY();
        return Math.sqrt(Math.pow(xDiff, 2) + Math.pow(yDiff, 2));
    }

    public void update(double f, double r, double t) {
        follower.update();

        limelight.update();

        shooter.update(getDistToGoal());

        //kickstand.update();

        forwardInput = -f;
        rightInput = -r;
        rotateInput = -t;
        if (slowDrive) {//!todo change
            forwardInput *= 0.5;
            rightInput *= 0.5;
            rotateInput *= 0.5;
        }

        beamBreaks.updatePrism(isShooting, autoAiming);
        //beamBreaks.auraFarm();
    }

    public double getOdoAngleErrorDeg() {
        double xDiff = goalPose.getX() - follower.getPose().getX();
        double yDiff = goalPose.getY() - follower.getPose().getY();
        double angleFromCoords = Math.toDegrees(Math.atan2(yDiff, xDiff));
        double targetAngle = normalizeAngle(angleFromCoords, false, AngleUnit.DEGREES);
        double currentHeading = Math.toDegrees(follower.getPose().getHeading());

        return normalizeAngle(targetAngle - currentHeading, false, AngleUnit.DEGREES);
    }

    public double getAimingPIDFOutput(double angleErrorDeg) {
        PIDController headingPID = new PIDController(headingKP, headingKI, headingKD); //robot todo tune this
        return -1 * (Range.clip((headingPID.calculate(angleErrorDeg) - headingKF * Math.signum(angleErrorDeg)), -1, 1));
    }

    double x = 0; // your initial state
    double Q = 0.1; // your model covariance
    double R = 0.4; // your sensor covariance
    double p = 1; // your initial covariance guess
    double K = 1; // your initial Kalman gain guess

    double x_previous = x;
    double p_previous = p;
    double odoChange = 0;
    double z = 0;
    public Pose filteredOdoPose(Pose rawOdometryPose) {
        odoChange = 1; // Ex: change in position from odometry.
        x = x_previous + odoChange;

        p = p_previous + Q;

        K = p/(p + R);

        z = 1; // Pose Estimate from April Tag / Distance Sensor

        x = x + K * (z - x);

        p = (1 - K) * p;

        x_previous = x;
        p_previous = p;
        return new Pose();
    }


}
