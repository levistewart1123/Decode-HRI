package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HuskeyLens {
    private static HuskyLens huskyLens;
    public static final double centerlinex = 160; //TODO change for correct half way line
    public static double leftBallAmount = 0;
    public static double rightBallAmount = 0;

    public static void init(HardwareMap hwMap){
        huskyLens = hwMap.get(HuskyLens.class,"huskylens");
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
    }

    public static double determineSide() {
        leftBallAmount = 0;
        rightBallAmount = 0;
        HuskyLens.Block[] blocks = huskyLens.blocks();
        for (int i = 0; i < blocks.length; i++) {
            double x = blocks[i].x;
            double y = blocks[i].y;
            if (x < centerlinex) {
                leftBallAmount++;
            } else if (x > centerlinex) {
                rightBallAmount++;
            }
        }

        if (leftBallAmount > rightBallAmount) {
            return -1;
        } else if (leftBallAmount < rightBallAmount) {
            return 1;
        } else {
            return 0;
        }
    }
}
