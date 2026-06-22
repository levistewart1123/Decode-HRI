package org.firstinspires.ftc.teamcode.tests;

import static com.pedropathing.ivy.Scheduler.execute;
import static com.pedropathing.ivy.Scheduler.reset;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.seattlesolvers.solverslib.util.MathUtils.normalizeAngle;
import static org.junit.jupiter.api.Assertions.assertEquals;

import static java.lang.Math.abs;

import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.BlockedBehavior;
import com.pedropathing.ivy.behaviors.ConflictBehavior;
import com.pedropathing.ivy.behaviors.InterruptedBehavior;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.junit.jupiter.api.Test;

import java.util.concurrent.TimeUnit;

public class RobotTests {

    FakeIntake intake = new FakeIntake();
    FakeFollower follower = new FakeFollower();
    FakeShooter shooter = new FakeShooter();
    String currentCommand = "";
    boolean isShooting = false;
    boolean autoAiming = false;
    int ballCount = 0;
    public enum IntakeState {
        IN,
        OUT,
        OFF,
        SHOOTING
    }
    IntakeState intakeState = IntakeState.OFF;

    public Command handleDriveInput = infinite(() -> {
        if (autoAiming) {
            follower.setTeleOpDrive(1, 1, 67);
            currentCommand += "Auto Aiming ";
        } else {
            follower.setTeleOpDrive(1, 1, 1);
            currentCommand += "manual drive ";
        }
    });
    public Command driveOff = instant(() -> {
        follower.setTeleOpDrive(0,0,0);
        currentCommand += "drive turned off ";
    });
    Command startTeleOpDrive = instant(() -> {
        follower.startTeleOpDrive();
        currentCommand += "drive started ";
    });
    public Command aimingOff = instant(() -> autoAiming = false);
    Command shootDone = instant(() -> currentCommand += "shoot completed");

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
        currentCommand += "shooting changed ";
        return instant(() -> isShooting = shooting);
    }
    Command fastShoot = sequential(
            driveOff,
            aimingOff,
            setShooting(true),
            intake.setIn,
            //waitMs(700),
            intake.turnOff,
            shooter.close,
            //add resetting beam breaks here
            setShooting(false),
            shootDone
    );
    Command slowShoot = sequential(
            driveOff,
            aimingOff,
            setShooting(true),
            intake.turnOff,
            shooter.open,
            //waitMs(300), //robot todo change to waitUntil(gateIsOpen) once it's working
            intake.setIn,
            //waitMs(700),
            intake.turnOff,
            shooter.close,
            setShooting(false),
            shootDone
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
    public Command handleGate = infinite(() -> {
                if (ballCount == 3) {
                    shooter.openGate();
                    currentCommand += "gate opened automatically ";
                } else {
                    shooter.closeGate();
                    currentCommand += "gate closed automatically ";
                }

            }
    )
            .requiring(shooter)
            .setPriority(0)
            .setInterruptedBehavior(InterruptedBehavior.SUSPEND)
            .setBlockedBehavior(BlockedBehavior.CANCEL)
            ;

    //*intake commands (creates new intake commands with requirements and priorities)
    public Command handleIntake = infinite(
            () -> {
                switch (intakeState){
                    case IN:
                        intake.spinIn();
                        currentCommand += "intake in ";
                        break;
                    case OUT:
                        intake.spinOut();
                        currentCommand += "intake out ";
                        break;
                    case OFF:
                        intake.stop();
                        currentCommand += "intake off ";
                        break;
                }
            }
    )
            .requiring(intake)
            .setPriority(0)
            .setInterruptedBehavior(InterruptedBehavior.SUSPEND)
            .setBlockedBehavior(BlockedBehavior.CANCEL)
            ;
    public Command stopIntake = instant(
            () -> {
                intake.stop();
                currentCommand += "intake stopped ";
            }
    )
            .requiring(intake)
            .setPriority(0)
            ;
    public Command reverseIntake = instant(
            () -> {
                intake.spinOut();
                currentCommand += "intake reversed ";
            }
    )
            .requiring(intake)
            .setPriority(0)
            ;

    @Test
    public void testOdometryDistance(){
        double xDiff = -20;
        double yDiff = -20;
        assertEquals((20*Math.sqrt(2)), (Math.sqrt(Math.pow(xDiff, 2) + Math.pow(yDiff, 2))));
    }

    @Test
    public void testGettingAngleToGoal(){
        double xDiff = 144 - 72;
        double yDiff = 144 - 72;
        double targetAngle = Math.toDegrees(Math.atan2(xDiff, yDiff));
        double error = 180 - targetAngle;
        assertEquals(135, error);
    }

    protected int loops = 0;
    protected int secondLoops = 0;
    protected int storedLoops = 0;
    protected Timing.Stopwatch loopTimer = new Timing.Stopwatch(TimeUnit.MILLISECONDS);
    protected Timing.Timer secTimer = new Timing.Timer(1000, TimeUnit.MILLISECONDS);

    public void incrementLoops(){
        secondLoops++;
        loops++;
        if (secTimer.done()){
            storedLoops = secondLoops;
            secondLoops = 0;
            secTimer.start();
        }
    }
    @Test
    public void testLoopTimer(){
        for (int i = 0; i<500; i++){
            incrementLoops();
        }
        assertEquals(500, loops);
    }

    @Test
    public void testCommands(){
        reset();
        isShooting = false;
        ballCount = 0;
        currentCommand = "";
        shooter.init();
        intake.init();
        autoAiming = false;
        schedule(startManualDrive);
        schedule(handleIntake);
        schedule(handleGate);
        intakeState = IntakeState.OFF;
        execute();
        currentCommand = "";
        intakeState = IntakeState.IN;
        execute();
        currentCommand = "";
        ballCount = 3;
        intakeState = IntakeState.OFF;
        execute();
        currentCommand = "";
        autoAiming = true;
        execute();
        currentCommand = "";
        schedule(shoot);
        execute();
        currentCommand = "";
        execute();
        currentCommand = "";
        execute();
        currentCommand = "";
        execute();
        currentCommand = "";
        execute();
        currentCommand = "";
        execute();
        currentCommand = "";
        execute();
        currentCommand = "";
        ballCount = 0;
        execute();
        currentCommand = "";
        execute();

        assertEquals("intake off gate closed automatically manual drive ", currentCommand);
    }

    @Test
    public void getAngleErrorDeg() {
        double xDiff = (137) - (8.5);
        double yDiff = 137 - 137;
        double targetAngle =  normalizeAngle(Math.toDegrees(Math.atan2(yDiff, xDiff)), false, AngleUnit.DEGREES);

        double error = normalizeAngle(targetAngle - (0), false, AngleUnit.DEGREES);
        assertEquals(0, (targetAngle - 45));
    }

    public static double headingKP = 0.02;
    public static double headingKI = 0;
    public static double headingKD = 0.01;
    public static double headingKF = 0.03;


    @Test
    public void getAimingPIDFOutput(){
        PIDController headingPID = new PIDController(headingKP, headingKI, headingKD); //robot todo tune this
        double error = 45;
        assertEquals(-1, Range.clip((headingPID.calculate(error) - headingKF * Math.signum(error)), -1, 1)); //!added kF separately
    }

    @Test
    public void runFlywheel(){
        double speed = 0;
        shooter.init();
        shooter.flywheels.setPIDCoeffs(20, 0, 0, 0, 0.7, 0);
        for (int i = 0; i <= 100; i++){
            double PIDOutput = shooter.flywheels.PID.calculate(0.5);
            double FFOutput = shooter.flywheels.FF.calculate(0.5);
            shooter.flywheels.set(0.5);
            speed = shooter.flywheels.get();
        }
        assertEquals(0.5, speed);
    }
    @Test
    public void getClosestShootPose(){ //red side
        Pose currentPose = new Pose(120,15);
        Pose closePose;
        boolean redSide = currentPose.getX() >= 72;
        if (currentPose.getY() <= -abs(currentPose.getX() - 72) + 65) {
            closePose = new Pose(72, 65);
        } else {
            closePose = currentPose;
            boolean good = poseInShootingZone(closePose);
            while (!poseInShootingZone(closePose)) {
                closePose = new Pose(closePose.getX() + ((redSide ? -1 : 1) * 1), closePose.getY() + 1);
            }
        }
        Pose farPose;
        if (currentPose.getY() >= abs(currentPose.getX() - 72) + 31) { //this is wrong
            farPose = new Pose(72, 31);
        } else {
            farPose = currentPose;
            while (!poseInShootingZone(farPose)) {
                farPose = new Pose(farPose.getX() + ((redSide ? -1 : 1) * 0.5), (farPose.getY() <= 14 ? farPose.getY() : (farPose.getY() - 0.5)));
            }
        }
        double closeDist = getDistFromPoints(currentPose, closePose);
        double farDist = getDistFromPoints(currentPose, farPose);
        assertEquals (farPose, (getDistFromPoints(currentPose, closePose) < getDistFromPoints(currentPose, farPose) ? closePose : farPose));
    }
    boolean poseInShootingZone(Pose pose){
        return (pose.getY() >= abs(pose.getX() - 72) + 65) || (pose.getY() <= -abs(pose.getX() - 72) + 31);
    }
    double getDistFromPoints(Pose start, Pose end) {
        double xDiff = end.getX() - start.getX();
        double yDiff = end.getY() - start.getY();
        return abs(Math.sqrt(Math.pow(xDiff, 2) + Math.pow(yDiff, 2)));
    }
}
