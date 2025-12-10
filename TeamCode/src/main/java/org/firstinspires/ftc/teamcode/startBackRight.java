package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.mechanisms.shootMotorInit;

@Autonomous
public class startBackRight extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;
    private shootMotorInit shootMotor;

    private TelemetryManager panelsTelemetry;


    /** This enum follows the logic of your second snippet but tailored to your specific pathing steps */
    public enum PathState {
        DRIVE_START_TO_SHOOT,
        SHOOT_PRELOAD,
        DRIVE_SHOOT_TO_COLLECT1_PRE,
        DRIVE_TO_COLLECT1,
        DRIVE_COLLECT1_TO_SHOOT,
        PARK
    }

    PathState pathState;

    // --- Coordinates from your first snippet ---
    private final Pose startPose = new Pose(95.41463414634146, 8.780487804878057, Math.toRadians(90));
    private final Pose shootPose = new Pose(80.19512195121952, 18.34146341463415, Math.toRadians(65));
    private final Pose collect1PrePose = new Pose(97.8475181356998, 34.991168008516965, Math.toRadians(0));
    private final Pose collect1Pose = new Pose(125.00671518477236, 35.7352555989025, Math.toRadians(0));

    private PathChain driveStartToShoot, driveShootToCollect1Pre, driveToCollect1, driveCollect1ToShoot;

    /** Builds the paths using the coordinates provided */
    public void buildPaths() {
        driveStartToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        driveShootToCollect1Pre = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, collect1PrePose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), collect1PrePose.getHeading())
                .build();

        driveToCollect1 = follower.pathBuilder()
                .addPath(new BezierLine(collect1PrePose, collect1Pose))
                .setLinearHeadingInterpolation(collect1PrePose.getHeading(), collect1Pose.getHeading())
                .build();

        driveCollect1ToShoot = follower.pathBuilder()
                .addPath(new BezierLine(collect1Pose, shootPose))
                .setLinearHeadingInterpolation(collect1Pose.getHeading(), shootPose.getHeading())
                .build();
    }

    /** Main State Machine logic */
    public void statePathUpdate() {
        switch (pathState) {
            case DRIVE_START_TO_SHOOT:
                if (!follower.isBusy()) {
                    follower.followPath(driveStartToShoot, true);
                    setPathState(PathState.SHOOT_PRELOAD);
                }
                break;

            case SHOOT_PRELOAD:
                if (!follower.isBusy()) {
                    shootMotor.setMotorSpeed(1.0);
                    // 5-second shoot timer
                    if (pathTimer.getElapsedTime() > 5000) {
                        shootMotor.setMotorSpeed(0);
                        follower.followPath(driveShootToCollect1Pre, true);
                        setPathState(PathState.DRIVE_SHOOT_TO_COLLECT1_PRE);
                    }
                }
                break;

            case DRIVE_SHOOT_TO_COLLECT1_PRE:
                if (!follower.isBusy()) {
                    follower.followPath(driveToCollect1, true);
                    setPathState(PathState.DRIVE_TO_COLLECT1);
                }
                break;

            case DRIVE_TO_COLLECT1:
                if (!follower.isBusy()) {
                    // Logic to transition to shooting from collection immediately
                    follower.followPath(driveCollect1ToShoot, true);
                    setPathState(PathState.DRIVE_COLLECT1_TO_SHOOT);
                }
                break;

            case DRIVE_COLLECT1_TO_SHOOT:
                if (!follower.isBusy()) {
                    shootMotor.setMotorSpeed(1.0);
                    if (pathTimer.getElapsedTime() > 5000) {
                        shootMotor.setMotorSpeed(0);
                        setPathState(PathState.PARK);
                    }
                }
                break;

            case PARK:
                if (!follower.isBusy()) {
                    telemetry.addLine("Autonomous Complete");
                }
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opModeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);

        shootMotor = new shootMotorInit();
        shootMotor.init(hardwareMap);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        panelsTelemetry.update(telemetry);


        buildPaths();
        pathState = PathState.DRIVE_START_TO_SHOOT;
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(PathState.DRIVE_START_TO_SHOOT);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        telemetry.addData("Path State", pathState.toString());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Path Time", pathTimer.getElapsedTime());
        telemetry.update();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }
}