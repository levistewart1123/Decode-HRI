package org.firstinspires.ftc.teamcode.opmode;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.util.Timing;

import java.util.concurrent.TimeUnit;

public class CommandOpMode extends OpMode {
    protected double loops = 0;
    protected double secondLoops = 0;
    protected double storedLoops = 0;
    protected ElapsedTime loopTimer = new ElapsedTime();
    protected Timing.Timer secTimer = new Timing.Timer(1000, TimeUnit.MILLISECONDS);
    public void reset() {
        Scheduler.reset();
    }

    /**
     * Schedules objects to the scheduler
     */
    public void schedule(Command... commands) {
        Scheduler.schedule(commands);
    }

    @Override
    public void init() {
        reset();
    }

    @Override
    public void start() {
        loopTimer.reset();
        secTimer.start();
    }

    @Override
    public void loop() {
        secondLoops++;
        loops++;
        if (secTimer.done()){
            storedLoops = secondLoops;
            secondLoops = 0;
            secTimer.start();
        }
        telemetry.addData("average loop time (ms): ", (loopTimer.milliseconds()/loops));
        telemetry.addData("loops per second (approximate)", storedLoops);
        Scheduler.execute();
    }

    public void stop() {
        reset();
    }
}
