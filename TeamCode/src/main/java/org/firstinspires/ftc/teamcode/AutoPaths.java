package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class AutoPaths {

    public Pose start, shoot, farRightCollect, farLeftCollect, gateCollect, spikeMarkI, spikeMarkII, spikeMarkIII;
    public PathChain startToShoot, shootToFarRightCollect, shootToFarLeftCollect, farLeftCollectToShoot, farRightCollectToShoot, shootToSpikeMarkI,shootToSpikeMarkII, shootToGateCollect, spikeMarkIToShoot, spikeMarkIIToShoot, gateCollectToShoot;

    public AutoPaths(Follower follower, boolean isRed, boolean far) {
        if (far) {// When far
            start = new Pose(134, 139, Math.toRadians(180));//! may not work due to reversing start heading
            shoot = new Pose(134, 139, Math.toRadians(216));
            farRightCollect = new Pose(); //TODO Add pose here
            farLeftCollect = new Pose(); //TODO Add pose here
            if (isRed) {
                start.mirror();
                shoot.mirror();
                farRightCollect.mirror();
                farLeftCollect.mirror();
            }
            startToShoot = follower.pathBuilder()
                    .addPath(new BezierLine(
                            start,
                            shoot
                    ))
                    .setLinearHeadingInterpolation(start.getHeading(), shoot.getHeading())
                    .build();
            shootToFarRightCollect = follower.pathBuilder()
                    .addPath(new BezierLine(
                            shoot,
                            farRightCollect
                    ))
                    .setLinearHeadingInterpolation(shoot.getHeading(), farRightCollect.getHeading())
                    .build();
            shootToFarLeftCollect = follower.pathBuilder()
                    .addPath(new BezierLine(
                            shoot,
                            farLeftCollect
                    ))
                    .setLinearHeadingInterpolation(shoot.getHeading(), farLeftCollect.getHeading())
                    .build();
            farLeftCollectToShoot = follower.pathBuilder()
                    .addPath(new BezierLine(
                            farLeftCollect,
                            shoot
                    ))
                    .setLinearHeadingInterpolation(farLeftCollect.getHeading(), shoot.getHeading())
                    .build();
            farRightCollectToShoot = follower.pathBuilder()
                    .addPath(new BezierLine(
                            farRightCollect,
                            shoot
                    ))
                    .setLinearHeadingInterpolation(farRightCollect.getHeading(), shoot.getHeading())
                    .build();
        } else if (!far) {// When close
            start = new Pose();//TODO Add poses here
            shoot = new Pose();//TODO Add poses here
            gateCollect = new Pose();//TODO Add poses here
            spikeMarkI = new Pose();//TODO Add poses here
            spikeMarkII = new Pose();//TODO Add poses here
            if (isRed){
                start.mirror();
                shoot.mirror();
                gateCollect.mirror();
                spikeMarkI.mirror();
                spikeMarkII.mirror();
            }
            startToShoot = follower.pathBuilder()
                    .addPath(new BezierLine(
                            start,
                            shoot
                    ))
                    .setLinearHeadingInterpolation(start.getHeading(), shoot.getHeading())
                    .build();
            shootToSpikeMarkI = follower.pathBuilder()
                    .addPath(new BezierLine(
                            shoot,
                            spikeMarkI
                    ))
                    .setLinearHeadingInterpolation(shoot.getHeading(), spikeMarkI.getHeading())
                    .build();
            shootToSpikeMarkII = follower.pathBuilder()
                    .addPath(new BezierLine(
                            shoot,
                            spikeMarkII
                    ))
                    .setLinearHeadingInterpolation(shoot.getHeading(), spikeMarkII.getHeading())
                    .build();
            shootToGateCollect = follower.pathBuilder()
                    .addPath(new BezierLine(
                            shoot,
                            gateCollect
                    ))
                    .setLinearHeadingInterpolation(shoot.getHeading(), gateCollect.getHeading())
                    .build();
            gateCollectToShoot = follower.pathBuilder()
                    .addPath(new BezierLine(
                            gateCollect,
                            shoot
                    ))
                    .setLinearHeadingInterpolation(gateCollect.getHeading(), shoot.getHeading())
                    .build();
            spikeMarkIToShoot = follower.pathBuilder()
                    .addPath(new BezierLine(
                            spikeMarkI,
                            shoot
                    ))
                    .setLinearHeadingInterpolation(spikeMarkI.getHeading(), shoot.getHeading())
                    .build();
            spikeMarkIIToShoot = follower.pathBuilder()
                    .addPath(new BezierLine(
                            spikeMarkII,
                            shoot
                    ))
                    .setLinearHeadingInterpolation(spikeMarkII.getHeading(), shoot.getHeading())
                    .build();
        }
        //etc
    }
}





