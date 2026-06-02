package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class AutoPaths {

    public Pose start, shoot;
    public PathChain startToShoot;
    //etc

    public AutoPaths(Follower follower, boolean isRed, boolean far) {
        if (far) {
            start = new Pose(134, 139);
            shoot = new Pose(134, 139);
            if (isRed) {
                start.mirror();
                shoot.mirror();
            }
            startToShoot = follower.pathBuilder()
                    .addPath(new BezierLine(
                            start,
                            shoot
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180 - 216), Math.toRadians(180 - 300))
                    .build();
        } else {

        }
        //etc

    }
}





