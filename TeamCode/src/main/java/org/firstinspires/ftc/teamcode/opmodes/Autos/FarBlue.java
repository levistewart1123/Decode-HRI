package org.firstinspires.ftc.teamcode.opmodes.Autos;

import static org.firstinspires.ftc.teamcode.Mechanisms.BeamBreak.updateBallAmount;
import static java.lang.Math.abs;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Mechanisms.BeamBreak;
import org.firstinspires.ftc.teamcode.Mechanisms.Hood;
import org.firstinspires.ftc.teamcode.RobotContext;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Blue Far V10", group = "12Ball")
@Configurable // Panels
public class BlueFarV10 extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private Timer pathTimer, actionTimer, opmodeTimer, flywheelTimer, shootTimeTimer; //Create timers
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private DcMotor intake = null;
    private Servo gate = null;
    private DcMotorEx flywheel1 = null;
    private DcMotorEx flywheel2 = null;
    private Limelight3A limelight = null;

    double RPMconstant = 1935.662;
    double Flywheel_Power = 0.48;
    double currentRPM = 0;
    double targetRPM = 3110;
    double AutoRPM = 0.0;
    static double maxRPM = 4700;
    double kP = 0.0015;
    double kI = 0;
    double kD = 0;
    double kF = 0;
    double kFMultiplier = 1.45;
    static double ilimit = 500;
    double errorSum = 0;
    double lastError = 0;
    boolean launching = false;
    double IntakeSpeed = 0.7;
    int LoopAmount = 0;

    int ballAmount = 3;
    int previousBallAmount = 3;

    public static double gateClosedPos = 1;
    public static double gateOpenPos = 0;

    double tx;
    double ty;

    private Hood hood;
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        flywheelTimer = new Timer();
        shootTimeTimer = new Timer();
        hood = new Hood();
        hood.init(hardwareMap);

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(start);

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        BeamBreak.init(hardwareMap);

        intake = hardwareMap.get(DcMotor.class, "Intake");
        flywheel1 = hardwareMap.get(DcMotorEx.class, "FlywheelLeft");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "FlywheelRight");
        gate = hardwareMap.get(Servo.class, "Gate");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(45); // This sets how often we ask Limelight for data ]
        limelight.start(); // This tells Limelight to start looking!

        limelight.pipelineSwitch(8); // Switch to pipeline number 8

        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel1.setDirection(DcMotor.Direction.REVERSE);
        flywheel2.setDirection(DcMotor.Direction.FORWARD);
        gate.setDirection(Servo.Direction.REVERSE);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addData("Status", "Ready to run");
        telemetry.update();
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
        flywheelTimer.resetTimer();
    }

    @Override
    public void loop() {
        hood.setHoodPosition(0.65);//! Setting hood pos
        updateBallAmount();
        ballAmount = BeamBreak.ballAmount;
        /*
        if (BeamBreak.ballAmount == previousBallAmount) {
            ballAmount = BeamBreak.ballAmount;
        }
        previousBallAmount = ballAmount;*/
        follower.update(); // Update Pedro Pathing
        if (opmodeTimer.getElapsedTimeSeconds() >= 29.5) {
            follower.holdPoint(follower.getPose());
        }
        if(opmodeTimer.getElapsedTimeSeconds() >= 29.9) {
            RobotContext.lastPose = follower.getPose();
            RobotContext.poseSet = true;
        }
        autonomousPathUpdate(); // Update autonomous state machine
        // Get Limelight result first (this frame)
        LLResult result = limelight.getLatestResult();
        telemetry.addData("Loop Amount", LoopAmount);

        telemetry.addData("ball amount", ballAmount);
        if (result != null && result.isValid()) {
            tx = result.getTx(); // How far left or right the target is (degrees)
            ty = result.getTy(); // How far up or down the target is (degrees)
            double ta = result.getTa(); // How big the target looks (0%-100% of the image)

            telemetry.addData("Target X", tx);
            telemetry.addData("Target Y", ty);
            telemetry.addData("Target Area", ta);
        } else {
            telemetry.addData("Limelight", "No Targets");
        }

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.debug("Current Flywheel RPM", currentRPM," Set RPM:", targetRPM);
        panelsTelemetry.update(telemetry);

        /* -------------------------FLYWHEEL CONTROL SECTION----------------------------------------*/
        //Ensure reasonable values
        targetRPM = Range.clip(abs(targetRPM), 0, maxRPM);
        kP = Math.max(0, kP);
        kI = Math.max(0, kI);
        kD = Math.max(0, kD);
        currentRPM = (flywheel2.getVelocity() / 28) * 60.0; //converts Ticks per second into RPM
        double dt = flywheelTimer.getElapsedTimeSeconds(); //gets change in time since last loop
        dt = Math.max(dt, 1e-6); // avoid insane derivative at very small d
        flywheelTimer.resetTimer(); //resets timer to start counting again
        double error = targetRPM - currentRPM;
        errorSum += error * dt;
        double derivative = (error - lastError) / dt;
        kF = kFMultiplier * (targetRPM / maxRPM);
        double output = (kF) + (kP * error) + (kI * errorSum) + (kD * derivative);  //PIDF controller

        Flywheel_Power = Range.clip(output, 0.0, 1.0);

        flywheel1.setPower(Flywheel_Power);
        flywheel2.setPower(Flywheel_Power); //COMMENTED OUT FOR GOAL AIMING TESTING

        lastError = error;
        /* -------------------------END FLYWHEEL CONTROL SECTION------------------------------------*/
    }
    @Override
    public void stop(){
        RobotContext.lastPose = follower.getPose();
        RobotContext.poseSet = true;
    }

    private final Pose start = new Pose(55.6, 7.2, Math.toRadians(90));
    private final Pose shootPose = new Pose(56.9, 20.6, Math.toRadians(118.5));//!not the same heading for first
    private final Pose pickupSpikeEnd = new Pose(11.6, 40, Math.toRadians(180)); // spike mark 3
    private final Pose pickupLowEnd = new Pose(10, 4, Math.toRadians(180));// human player R = 193
    private final Pose pickupHighEnd = new Pose(9,25.000, Math.toRadians(180));
    private final Pose end = new Pose(56.8, 22.6, Math.toRadians(132.4));
    private final Pose HPOut = new Pose(18,4,Math.toRadians(180));// human player move out pos

    public class Paths {
        public PathChain StartToShoot;
        public PathChain PickupSpike;
        public PathChain PickupSpikeToShoot;
        public PathChain LowPickup;
        public PathChain LowPickupToShoot;
        public PathChain HighPickup;
        public PathChain HPBounceOut;
        public PathChain HPBounceIn;
        public PathChain HighPickupToShoot;
        public PathChain ShootToEnd;
        public PathChain ShootToShoot;

        public Paths(Follower follower) {
            StartToShoot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    start,
                                    shootPose
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), (shootPose.getHeading()-Math.toRadians(1.5))) //!Different heading
                    .build();
            PickupSpike = follower.pathBuilder().addPath(
                            new BezierLine(
                                    shootPose,
                                    pickupSpikeEnd
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(170), (pickupSpikeEnd.getHeading()))
                    .build();
            PickupSpikeToShoot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    pickupSpikeEnd,
                                    shootPose
                            )
                    ).setLinearHeadingInterpolation(pickupSpikeEnd.getHeading(), shootPose.getHeading())
                    .build();

            LowPickup = follower.pathBuilder().addPath(
                            new BezierLine(
                                    shootPose,
                                    pickupLowEnd
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), pickupLowEnd.getHeading())
                    .build();

            LowPickupToShoot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    pickupLowEnd,
                                    shootPose
                            )
                    ).setLinearHeadingInterpolation(pickupLowEnd.getHeading(), shootPose.getHeading())
                    .build();


            HighPickup = follower.pathBuilder().addPath(
                            new BezierLine(
                                    pickupHighEnd,
                                    shootPose
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), pickupHighEnd.getHeading())

                    .build();

            HPBounceOut = follower.pathBuilder().addPath(
                            new BezierLine(
                                    pickupLowEnd,
                                    HPOut
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();
            HPBounceIn = follower.pathBuilder().addPath(
                            new BezierLine(
                                    HPOut,
                                    pickupLowEnd
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            HighPickupToShoot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    pickupHighEnd,
                                    shootPose
                            )
                    ).setLinearHeadingInterpolation(pickupHighEnd.getHeading(), shootPose.getHeading())

                    .build();

            ShootToEnd = follower.pathBuilder().addPath(
                            new BezierLine(
                                    shootPose,
                                    end
                            )
                    ).setLinearHeadingInterpolation(shootPose.getHeading(), end.getHeading())
                    .build();
            ShootToShoot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    shootPose,
                                    shootPose
                            )
                    ).setLinearHeadingInterpolation(shootPose.getHeading(), shootPose.getHeading())
                    .build();

        } //To create the Red Auto, you will have to edit the control points here, but should be able to keep all the paths.
    }
    //!3400 is flywheel speed from far
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: //Move from start to shoot
                gate.setPosition(gateOpenPos);
                follower.followPath(paths.StartToShoot, 0.5, true);
                targetRPM = 3125;
                setPathState(1);
                break;
            case 1:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy() && (pathTimer.getElapsedTimeSeconds() > 6 || currentRPM >= targetRPM)) {
                    /* Wait 3 seconds to Score Preload */
                    shootTime(0.5);
                    if (!launching) {
                        follower.followPath(paths.PickupSpike, 1, false);
                        gate.setPosition(gateClosedPos);
                        intake.setPower(1);
                        /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the balls */
                        setPathState(2); //Once we're done scoring, we set the path state to the next state.
                    }
                }
                break;
            case 2: //grab balls at pickup 1
                if(!follower.isBusy()) {
                    follower.followPath(paths.PickupSpikeToShoot, 0.7, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (pathTimer.getElapsedTimeSeconds() >= 0.5){
                    gate.setPosition(gateOpenPos);
                    intake.setPower(0);
                }

                if(!follower.isBusy()) {
                    shootTime(0.5);
                    if (!launching) {
                        follower.followPath(paths.LowPickup, 1, false);
                        gate.setPosition(gateClosedPos);
                        intake.setPower(1);
                        if (LoopAmount >= 1) {
                            setPathState(6);
                        } else {
                            setPathState(4);
                        }
                    }
                }
                break;
            case 4:
                if(!follower.isBusy()){
                    follower.followPath(paths.HPBounceOut,1,false);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()){
                    follower.followPath(paths.HPBounceIn,1,false);
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy() || (pathTimer.getElapsedTimeSeconds() >= 2 || ballAmount == 3)) {
                    follower.followPath(paths.LowPickupToShoot, 0.8, true);
                    intake.setPower(0);
                    LoopAmount++;
                    if(LoopAmount >= 2 && opmodeTimer.getElapsedTimeSeconds() >= 25) {
                        setPathState(9); //! moves to case 9 because 7 and 8 makes it dance.
                    } else {
                        setPathState(3);
                    }
                }
                break;
            case 7:
//                if (pathTimer.getElapsedTimeSeconds() >= 0.5){
//                    gate.setPosition(gateOpenPos);
//                }
//
//                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 2) {
//                    shootTime(0.5);
//                    if (!launching) {
//                        intake.setPower(1);
//                        gate.setPosition(gateClosedPos);
//                        follower.followPath(paths.HighPickup, 0.6, false);
//                        setPathState(8);
//                        }
//                }
                break;
            case 8:
//                if(!follower.isBusy() && (pathTimer.getElapsedTimeSeconds() >= 3 || ballAmount == 3)) {
//                    intake.setPower(0);
//                    follower.followPath(paths.HighPickupToShoot,0.7, true);
//                    setPathState(9);
//                }
                break;
            case 9:
                if (pathTimer.getElapsedTimeSeconds() >= 0.5){
                    gate.setPosition(gateOpenPos);
                }
                if(!follower.isBusy()) {
                    intake.setPower(0);
                    shootTime(0.5);
                    if (!launching){
                        follower.followPath(paths.ShootToEnd, 0.7, false);
                        setPathState(10);
                    }
                }
                break;
            case 10:
                setPathState(-1);
                break;
        }
    }
    public void shootTime(double delay) {
        if (!launching) {
            shootTimeTimer.resetTimer();
            launching = true;
        }
        if (launching) {
            double t = shootTimeTimer.getElapsedTimeSeconds() - delay;
            if (t < 0.5) {
                if (intake.getPower() != 1) {
                    intake.setPower(1);
                }
            }
            if (t > 0.5) {
                intake.setPower(0.0);
                launching = false;
            }
        }
        BeamBreak.reset();
    }
    public double getAutoRPM(){
        AutoRPM = RPMconstant + 10.82136*Math.pow(Math.E, (-0.3156084*ty));
        return AutoRPM;
    }
    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}