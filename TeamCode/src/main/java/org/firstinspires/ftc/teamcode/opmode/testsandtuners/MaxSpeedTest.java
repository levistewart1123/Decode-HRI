package org.firstinspires.ftc.teamcode.opmode.testsandtuners;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.opmode.CommandOpMode;

@TeleOp
public class MaxSpeedTest extends CommandOpMode {
    private DcMotorEx flywheelL;
    private DcMotorEx flywheelR;

    @Override
    public void init() {
        super.init();
        flywheelL = hardwareMap.get(DcMotorEx.class, "FlywheelLeft");
        flywheelR = hardwareMap.get(DcMotorEx.class, "FlywheelRight");
    }

    @Override
    public void start() {
        super.start();
        flywheelL.setPower(1);
        flywheelR.setPower(1);
    }

    @Override
    public void loop() {
        telemetry.addData("velocity: ", flywheelL.getVelocity()/28); //rpm(?)
        super.loop();
        telemetry.update();
    }

    @Override
    public void stop() {
        super.stop();
    }
}
