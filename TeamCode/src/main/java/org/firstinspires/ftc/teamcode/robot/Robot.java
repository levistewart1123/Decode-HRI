package org.firstinspires.ftc.teamcode.robot;

import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.BlockedBehavior;
import com.pedropathing.ivy.behaviors.ConflictBehavior;
import com.pedropathing.ivy.behaviors.InterruptedBehavior;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.teamcode.PoseSaver;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.subsystems.BeamBreaks;
import org.firstinspires.ftc.teamcode.robot.subsystems.HuskyLens;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Kickstand;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;


import java.util.List;
import java.util.concurrent.TimeUnit;

@Configurable
public class Robot {
    public enum DriveState{
        NORMAL,
        AIMING,
        AUTOMATED,
        OFF
    }
    public enum IntakeState {
        IN,
        OUT,
        OFF,
        SHOOTING
    }
    DriveState driveState = DriveState.NORMAL;
    public IntakeState intakeState = IntakeState.OFF;

    public Intake intake = new Intake();
    public Shooter shooter = new Shooter();
    public HuskyLens huskyLens = new HuskyLens();
    public Follower follower;
    public BeamBreaks beamBreaks = new BeamBreaks();
    public Kickstand kickstand = new Kickstand();
    Timing.Timer shootTimer = new Timing.Timer(700, TimeUnit.MILLISECONDS);
    boolean waitForOpen;
    public boolean autoAiming = false;
    public Pose goalPose;
    public Pose redGoal = new Pose(134,139);
    public Pose humanPZ;
    public Pose redHPZ = new Pose(8,11.5, 0);
    double forwardInput, rightInput, rotateInput = 0;
    public boolean isShooting = false;
    public boolean slowDrive = false;
    public static double headingKP = 0.025;
    public static double headingKI = 0;
    public static double headingKD = 0.02;
    public static double headingKF = 0.025;


    //*movement commands
    public Command handleDriveInput = infinite(() -> {
        if (autoAiming) {
            follower.setTeleOpDrive(forwardInput, rightInput, getAimingPIDFOutput());
        } else {
            follower.setTeleOpDrive(forwardInput, rightInput, rotateInput);
        }
    });
    public Command driveOff = instant(() -> follower.setTeleOpDrive(0,0,0));
    Command startTeleOpDrive = instant(() -> follower.startTeleOpDrive());

    public Command startManualDrive = sequential(
            startTeleOpDrive,
            handleDriveInput
    )
            .requiring(follower)
            .setPriority(0)
            .setInterruptedBehavior(InterruptedBehavior.SUSPEND)
            .setConflictBehavior(ConflictBehavior.QUEUE)
            .setBlockedBehavior(BlockedBehavior.QUEUE)
            ;
    //*shooting commands
    Command setShooting(boolean shooting){
      return instant(() -> isShooting = shooting);
    }
    Command fastShoot = sequential(
            driveOff,
            setShooting(true),
            intake.setIn,
            waitMs(700),
            intake.turnOff,
            shooter.close,
            setShooting(false)
    )
            .requiring(intake, follower, shooter)
            .setPriority(2)
            ;
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
            setShooting(false)
    )
            .requiring(intake, follower, shooter)
            .setPriority(2)
            ;
    public Command shoot = conditional(
            () -> false, //!fixme
            fastShoot,
            slowShoot
    )
            .requiring(intake, follower, shooter)
            .setPriority(2)
            ;
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
            .setConflictBehavior(ConflictBehavior.QUEUE)
            ;

    public Command handleIntake = infinite(
            () -> {
                switch (intakeState){
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
            .setConflictBehavior(ConflictBehavior.QUEUE)
            ;


    public void init(boolean isRed, HardwareMap hwMap){
        List<LynxModule> allHubs = hwMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO); //we can try setting this to manual and see how much loop times improve
        }

        follower = Constants.createFollower(hwMap);
        intake.init(hwMap);
        shooter.init(hwMap);
        huskyLens.init(hwMap);
        //beamBreaks.init(hwMap);
        //kickstand.init(hwMap);

        if (isRed){
            goalPose = redGoal;
            humanPZ = redHPZ;
        } else {
            goalPose = redGoal.mirror();
            humanPZ = redHPZ.mirror();
        }
        if (PoseSaver.autoWasRun) {
            follower.setStartingPose(PoseSaver.endPose);
        } else {
            follower.setStartingPose(humanPZ);
        }
        PoseSaver.autoWasRun = false;
        follower.update();
    }

    public double getDistToGoal(){
        double xDiff = follower.getPose().getX() - goalPose.getX();
        double yDiff = follower.getPose().getY() - goalPose.getY();
        return Math.sqrt(Math.pow(xDiff, 2) + Math.pow(yDiff, 2));
    }

    public void periodic(double f, double r, double t){
        follower.update();
        shooter.autoHood(getDistToGoal());

        //kickstand.periodic();

        forwardInput = -f;
        rightInput = -r;
        rotateInput = -t;
        if (slowDrive){
            forwardInput *= 0.2;
            rightInput *= 0.2;
            rotateInput *= 0.2;
        }
        shooter.runWithPIDF(1);
        //shooter.periodic(getDistToGoal());
        //beamBreaks.periodic(isShooting, autoAiming);
    }

    public void setIntakeState(IntakeState intakeState){
        this.intakeState = intakeState;
    }
    public void setDriveStateManual(DriveState driveState) {
        if (this.driveState != driveState && driveState != DriveState.AUTOMATED) {
            this.driveState = driveState;
            switch (driveState) {
                case NORMAL:
                case AIMING:
                    follower.startTeleOpDrive();
                    break;
                case OFF:
                    follower.setTeleOpDrive(0, 0, 0);
                    break;
            }
        }
    }
    public void setDriveStateAutomated(PathChain pathChain){
        if (this.driveState != DriveState.AUTOMATED) {
            this.driveState = DriveState.AUTOMATED;
            follower.setTeleOpDrive(0, 0, 0);
            follower.followPath(pathChain);
        }
    }

    public void toggleAiming(){
        if (driveState == DriveState.AIMING){
            setDriveStateManual(DriveState.NORMAL);
        } else if (driveState == DriveState.NORMAL) {
            setDriveStateManual(DriveState.AIMING);
        }
    }


    public void autoGate(){
        if (beamBreaks.getBallCount() < 3 && !isShooting){
            shooter.closeGate();
        } else {
            shooter.openGate();
        }
    }

    public double getAngleErrorDeg(){
        double xDiff = goalPose.getX() - follower.getPose().getX();
        double yDiff = goalPose.getY() - follower.getPose().getY();
        double badAngle = Math.toDegrees(Math.atan2(xDiff, yDiff));
        double targetAngle = Math.atan2(Math.cos(badAngle), Math.sin(badAngle));

        return Math.toDegrees(targetAngle - follower.getHeading());
    }

    public double getAimingPIDFOutput(){
        PIDController headingPID = new PIDController(headingKP, headingKI, headingKD); //robot todo tune this
        return (Range.clip((headingPID.calculate(getAngleErrorDeg()) - headingKF * Math.signum(getAngleErrorDeg())), -1, 1));
    }

    public void startShoot(){
        isShooting = true;
        setIntakeState(IntakeState.SHOOTING);
        waitForOpen = shooter.gateIsClosed();
        if (waitForOpen){
            shooter.openGate();
            intake.stop();
        } else {
            intake.spinIn();
        }
        shootTimer.start();
    }

    public void handleShoot(){
        if (waitForOpen) {
            if (shootTimer.elapsedTime() > 300){
                waitForOpen = false;
                shootTimer.start();
            }
        } else {
            intake.spinIn();
        }

        if (shootTimer.done()) {
            intake.stop();
            shooter.closeGate();
            setDriveStateManual(DriveState.NORMAL);
            setIntakeState(IntakeState.OFF);
            beamBreaks.reset();
            isShooting = false;
        }


    }


}
