package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class HuskyLens {
    private static com.qualcomm.hardware.dfrobot.HuskyLens huskyLens;
    public static final double centerLineX = 160;
    public final double center = 2;
    public static double leftBallAmount = 0;
    public static double rightBallAmount = 0;

    public void initialize(HardwareMap hwMap) {
        huskyLens = hwMap.get(com.qualcomm.hardware.dfrobot.HuskyLens.class, "huskylens");
        huskyLens.selectAlgorithm(com.qualcomm.hardware.dfrobot.HuskyLens.Algorithm.COLOR_RECOGNITION);
    }

    public static double sideNumber() {
        leftBallAmount = 0;
        rightBallAmount = 0;
        com.qualcomm.hardware.dfrobot.HuskyLens.Block[] blocks = huskyLens.blocks();
        for (com.qualcomm.hardware.dfrobot.HuskyLens.Block block : blocks) {
            double x = block.x;
            if (x < centerLineX) {
                leftBallAmount++;
            } else if (x > centerLineX) {
                rightBallAmount++;
            }
        }

        return Double.compare(rightBallAmount, leftBallAmount);
    }
}