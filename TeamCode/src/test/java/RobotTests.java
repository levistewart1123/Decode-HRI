import static com.pedropathing.ivy.Scheduler.execute;
import static com.pedropathing.ivy.Scheduler.reset;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static org.junit.jupiter.api.Assertions.assertEquals;

import static java.lang.Thread.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.BlockedBehavior;
import com.pedropathing.ivy.behaviors.ConflictBehavior;
import com.pedropathing.ivy.behaviors.InterruptedBehavior;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.junit.jupiter.api.Test;

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
    void testMath() {
        int motorPowerCalculation = 2 + 2;
        assertEquals(4, motorPowerCalculation, "Math should work perfectly!");
    }

    @Test
    void testOdometryDistance(){
        double xDiff = -20;
        double yDiff = -20;
        assertEquals((20*Math.sqrt(2)), (Math.sqrt(Math.pow(xDiff, 2) + Math.pow(yDiff, 2))));
    }

    @Test
    void testGettingAngleToGoal(){
        double xDiff = 144 - 72;
        double yDiff = 144 - 72;
        double targetAngle = Math.toDegrees(Math.atan2(xDiff, yDiff));
        double error = 0 - targetAngle;
        assertEquals(-45, error);
    }

    @Test
    void testCommands(){
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

        assertEquals("manual drive intake off gate closed automatically ", currentCommand);
    }
}
