package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.tele.BaseTeleOp;

@Autonomous(name = "Red Far", group = "far", preselectTeleOp = "Red TeleOp")
public class RedFarAuto extends BaseTeleOp {
    public RedFarAuto() {
        super(true);
    }
}