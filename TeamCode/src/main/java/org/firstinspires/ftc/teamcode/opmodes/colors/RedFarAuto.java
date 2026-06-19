package org.firstinspires.ftc.teamcode.opmodes.colors;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.auto.BaseFarAuto;

@Autonomous(name = "Red Far", group = "far", preselectTeleOp = "Red TeleOp")
public class RedFarAuto extends BaseFarAuto {
    public RedFarAuto() {
        super(true);
    }
}