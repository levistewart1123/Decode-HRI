package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class HuskyLens {
    private com.qualcomm.hardware.dfrobot.HuskyLens huskyLens;
    public final double centerLineX = 160; //TODO change for correct half way line
    public double leftBallAmount = 0;
    public double rightBallAmount = 0;

    public void init(HardwareMap hwMap){
        huskyLens = hwMap.get(com.qualcomm.hardware.dfrobot.HuskyLens.class,"huskylens");
        huskyLens.selectAlgorithm(com.qualcomm.hardware.dfrobot.HuskyLens.Algorithm.COLOR_RECOGNITION);
    }

    public double determineSide() {
        leftBallAmount = 0;
        rightBallAmount = 0;
        com.qualcomm.hardware.dfrobot.HuskyLens.Block[] blocks = huskyLens.blocks();
        for (com.qualcomm.hardware.dfrobot.HuskyLens.Block block : blocks) {
            double x = block.x;
            double y = block.y;
            if (x < centerLineX) {
                leftBallAmount++;
            } else if (x > centerLineX) {
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
