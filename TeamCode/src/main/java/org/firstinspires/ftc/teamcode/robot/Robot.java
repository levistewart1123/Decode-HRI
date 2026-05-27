package org.firstinspires.ftc.teamcode.robot;

import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.BlockedBehavior;
import com.pedropathing.ivy.behaviors.ConflictBehavior;
import com.pedropathing.ivy.behaviors.InterruptedBehavior;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.teamcode.PoseSaver;
import org.firstinspires.ftc.teamcode.robot.subsystems.BeamBreaks;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Kickstand;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;


import java.util.List;
import java.util.concurrent.TimeUnit;

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
    IntakeState intakeState = IntakeState.OFF;

    public Intake intake;
    public Shooter shooter;
    public Follower follower;
    public BeamBreaks beamBreaks;
    public Kickstand kickstand;
    Timing.Timer shootTimer = new Timing.Timer(700, TimeUnit.MILLISECONDS);
    boolean waitForOpen;
    public boolean autoAiming = false;
    boolean usingAutoGate = true;
    public Pose goalPose;
    double forwardInput, rightInput, rotateInput = 0;
    public boolean isShooting = false;
    public boolean slowDrive = false;
    public Pose redGoal = new Pose(134,139);

    //*movement commands
    // Combine the logic into one infinite command
    public Command handleDriveInput = infinite(() -> {
        if (autoAiming) {
            follower.setTeleOpDrive(forwardInput, rightInput, getAimingPIDFOutput());
        } else {
            follower.setTeleOpDrive(forwardInput, rightInput, rotateInput);
        }
    });
    public Command driveOff = instant(() -> follower.setTeleOpDrive(0,0,0));
    Command startTeleOpDrive = instant(() -> follower.startTeleOpDrive());
    public Command toggleAiming = instant(() -> autoAiming = !autoAiming);

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
    );
    Command slowShoot = sequential(
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
    );
    public Command shoot = conditional(
            () -> shooter.gateIsOpen(),
            fastShoot,
            slowShoot
    )
            .requiring(intake, follower, shooter)
            .setPriority(1)
            ;
    //*other shooter commands
    public Command handleGate = conditional(
            () -> (beamBreaks.getBallCount() < 3),
            shooter.open,
            shooter.close
    )
            .requiring(shooter)
            .setPriority(0)
            .setBlockedBehavior(BlockedBehavior.CANCEL)
            ;
    //*intake commands (creates new intake commands with requirements and priorities)
    public Command startIntake = instant(
            () -> intake.spinIn()
    )
            .requiring(intake)
            .setPriority(0)
            ;
    public Command stopIntake = instant(
            () -> intake.stop()
    )
            .requiring(intake)
            .setPriority(0)
            ;
    public Command reverseIntake = instant(
            () -> intake.spinOut()
    )
            .requiring(intake)
            .setPriority(0)
            ;







    public void init(boolean isRed, HardwareMap hwMap){
        List<LynxModule> allHubs = hwMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO); //we can try setting this to manual and see how much loop times improve
        }

        intake = new Intake();
        shooter = new Shooter();
        beamBreaks = new BeamBreaks();
        kickstand = new Kickstand();
        intake.init(hwMap);
        shooter.init(hwMap);
        beamBreaks.init(hwMap);
        shooter.init(hwMap);

        if (isRed){
            goalPose = redGoal;
        } else {
            goalPose = redGoal.mirror();
        }
        if (PoseSaver.autoWasRun) {
            follower.setStartingPose(PoseSaver.endPose);
        }
    }

    public double getDistToGoal(){
        double xDiff = follower.getPose().getX() - goalPose.getX();
        double yDiff = follower.getPose().getY() - goalPose.getY();
        return Math.sqrt(Math.pow(xDiff, 2) + Math.pow(yDiff, 2)); //lab todo double check this
    }

    public void periodic(Gamepad gamepad){
        follower.update();

        kickstand.periodic();

        if (isShooting) {
            handleShoot();
        }

        forwardInput = gamepad.left_stick_y;
        rightInput = gamepad.left_stick_x;
        rotateInput = gamepad.right_stick_x;//lab todo check directions with old code
        if (slowDrive){
            forwardInput *= 0.2;
            rightInput *= 0.2;
            rotateInput *= 0.2;
        }
        switch (driveState){
            case NORMAL:
                follower.setTeleOpDrive(forwardInput, rightInput, rotateInput);
                break;
            case AIMING:
                follower.setTeleOpDrive(forwardInput, rightInput, getAimingPIDFOutput());
                break;
            case AUTOMATED:
                if (!follower.isBusy()){
                    setDriveStateManual(DriveState.NORMAL);
                }
                break;
            case OFF:
                follower.setTeleOpDrive(0,0,0);
                break;
        }

        switch (intakeState){
            case IN:
                intake.spinIn();
                break;
            case OUT:
                intake.spinOut();
                break;
            case OFF:
                intake.stop();
            case SHOOTING:
                break;
        }

        shooter.periodic(getDistToGoal());
        beamBreaks.periodic(isShooting, autoAiming);
        if (usingAutoGate){
            autoGate();
        }
    }

    public void commandPeriodic(Gamepad gamepad){
        follower.update();

        kickstand.periodic();

        if (isShooting) {
            handleShoot();
        }

        forwardInput = gamepad.left_stick_y;
        rightInput = gamepad.left_stick_x;
        rotateInput = gamepad.right_stick_x;//lab todo check directions with old code
        if (slowDrive){
            forwardInput *= 0.2;
            rightInput *= 0.2;
            rotateInput *= 0.2;
        }

        switch (intakeState){
            case IN:
                intake.spinIn();
                break;
            case OUT:
                intake.spinOut();
                break;
            case OFF:
                intake.stop();
            case SHOOTING:
                break;
        }

        shooter.periodic(getDistToGoal());
        beamBreaks.periodic(isShooting, autoAiming);
        if (usingAutoGate){
            autoGate();
        }
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

    public double getAimingPIDFOutput(){
        double xDiff = goalPose.getX() - follower.getPose().getX();
        double yDiff = goalPose.getY() - follower.getPose().getY();
        double targetAngle = Math.atan2(xDiff, yDiff);
        double error = follower.getHeading() - targetAngle;

        PIDFController headingPIDF = new PIDFController(0, 0, 0, 0); //lab todo tune this or base it off of pedro or copy it over from old code
        return headingPIDF.calculate(error); //robot todo ensure radians and degrees don't mix, this whole thing really needs testing
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
