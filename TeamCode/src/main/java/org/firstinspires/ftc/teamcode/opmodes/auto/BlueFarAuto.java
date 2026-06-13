package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.tele.BaseTeleOp;

@Autonomous(name = "Blue Far", group = "far", preselectTeleOp = "Blue TeleOp")
public class BlueFarAuto extends BaseTeleOp {
    public BlueFarAuto() {
        super(true);
    }
}