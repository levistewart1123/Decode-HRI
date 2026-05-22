package org.firstinspires.ftc.teamcode;

import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.teamcode.subsystems.BeamBreaks;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Kickstand;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;


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
    public Follower f;
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

    Command shoot = Command.build() //might want to make this 2 separate commands if I see any issues
            .setStart(() -> {
                isShooting = true;
                intake.stop();
                waitForOpen = shooter.gateIsClosed();
                if (waitForOpen){
                    shooter.openGate();
                }
                shootTimer.start();
            })
            .setExecute(() -> {
                if (waitForOpen) {
                    if (shootTimer.elapsedTime() > 300){
                        waitForOpen = false;
                        shootTimer.start();
                    }
                } else {
                    intake.spinIn();
                }

            })
            .setDone(() -> shootTimer.done())
            .setEnd(endCondition -> {
                intake.stop();
                shooter.closeGate();
                isShooting = false;
            })
            .requiring(intake, shooter)
            //todo add conflicting behaviors here and priority
            ;

    public Command handleNormalDrive(){
        return infinite(()-> f.setTeleOpDrive(forwardInput, rightInput, rotateInput));
    }
    public Command handleAimingDrive(){
        return infinite(()-> f.setTeleOpDrive(forwardInput, rightInput, getAimingPIDFOutput()));
    }
    public Command driveOff = instant(() -> f.setTeleOpDrive(0, 0, 0));
    public Command followPath(PathChain pathChain){
        return (driveOff).then(follow(f, pathChain));
    }

    public Command handleManualDrive = conditional(
            () -> autoAiming,
            handleNormalDrive(),
            handleAimingDrive()
    );

    public void init(boolean red, HardwareMap hwMap){
        intake = new Intake();
        shooter = new Shooter();
        beamBreaks = new BeamBreaks();
        kickstand = new Kickstand();
        intake.init(hwMap);
        shooter.init(hwMap);
        beamBreaks.init(hwMap);
        shooter.init(hwMap);

        if (red){
            goalPose = Paths.redGoal;
        } else {
            goalPose = Paths.blueGoal;
        }
        f.setStartingPose(PoseSaver.endPose);
    }

    public double getDistToGoal(){
        double xDiff = f.getPose().getX() - goalPose.getX();
        double yDiff = f.getPose().getY() - goalPose.getY();
        return Math.sqrt(Math.pow(xDiff, 2) + Math.pow(yDiff, 2)); //lab todo aouble check this
    }

    public void periodic(Gamepad gamepad){
        f.update();

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
                f.setTeleOpDrive(forwardInput, rightInput, rotateInput);
                break;
            case AIMING:
                f.setTeleOpDrive(forwardInput, rightInput, getAimingPIDFOutput());
                break;
            case AUTOMATED:
                if (!f.isBusy()){
                    setDriveStateManual(DriveState.NORMAL);
                }
                break;
            case OFF:
                f.setTeleOpDrive(0,0,0);
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

    public void setIntakeState(IntakeState intakeState){
        this.intakeState = intakeState;
    }
    public void setDriveStateManual(DriveState driveState) {
        if (this.driveState != driveState && driveState != DriveState.AUTOMATED) {
            this.driveState = driveState;
            switch (driveState) {
                case NORMAL:
                case AIMING:
                    f.startTeleOpDrive();
                    break;
                case OFF:
                    f.setTeleOpDrive(0, 0, 0);
                    break;
            }
        }
    }
    public void setDriveStateAutomated(PathChain pathChain){
        if (this.driveState != DriveState.AUTOMATED) {
            this.driveState = DriveState.AUTOMATED;
            f.setTeleOpDrive(0, 0, 0);
            f.followPath(pathChain);
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
        double xDiff = f.getPose().getX() - goalPose.getX();
        double yDiff = f.getPose().getY() - goalPose.getY();
        double targetAngle = Math.atan2(xDiff, yDiff);
        double error = f.getHeading() - targetAngle; //lab todo check this

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
