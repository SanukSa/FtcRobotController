package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.mechanisms.shootMotorInit;

@Autonomous
public class rightAutoPath extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;
    private shootMotorInit shootMotor;

    private TelemetryManager panelsTelemetry;

    public enum PathState{
        DRIVE_START_TO_SHOOT_POS,
        SHOOT_PRELOAD,
        DRIVE_SHOOT_TO_COLLECT1_POSE,
        WAIT_AT_COLLECT1,              // New Wait State
        DRIVE_COLLECT1_TO_SHOOT_POSE,
        DRIVE_SHOOT_TO_COLLECT2_POSE,
        WAIT_AT_COLLECT2,              // New Wait State
        DRIVE_COLLECT2_TO_SHOOT_POSE,
        DRIVE_SHOOT_TO_COLLECT3_POSE,
        WAIT_AT_COLLECT3,              // New Wait State
        DRIVE_COLLECT3_TO_SHOOT_POSE,
        DRIVE_SHOOT_TO_END_POSE
    }

    PathState pathState;

    // Coordinates
    private final Pose startPose = new Pose(124.68292682926829, 124.29268292682926, Math.toRadians(35));
    private final Pose shootPose = new Pose(114.92682926829269, 114.73170731707317, Math.toRadians(45));

    // UPDATED: Headings set to 0 (Perpendicular)
    private final Pose collect1Pose = new Pose(124.48780487804879, 83.51219512195122, Math.toRadians(0));
    private final Pose collect2Pose = new Pose(124.87804878048782, 59.512195121951216, Math.toRadians(0));
    private final Pose collect3Pose = new Pose(125.07317073170732, 35.3170731707317, Math.toRadians(0));

    private final Pose endPose = new Pose(91.90243902439024, 120.1951219512195, Math.toRadians(44));

    private PathChain driveStartToShootPos, driveShootToCollect1Pose, driveCollect1ToShootPose, driveShootToCollect2Pose, driveCollect2ToShootPose,
            driveShootToCollect3Pose, driveCollect3ToShootPose, driveShootToEndPose;

    public void buildPaths() {
        driveStartToShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        driveShootToCollect1Pose = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootPose,
                        new Pose(91.90243902439025, 92.29268292682927, 0),
                        new Pose(76.48780487804878, 82.14634146341463, 0),
                        collect1Pose
                ))
                .setLinearHeadingInterpolation(shootPose.getHeading(), collect1Pose.getHeading())
                .build();

        driveCollect1ToShootPose = follower.pathBuilder()
                .addPath(new BezierLine(collect1Pose, shootPose))
                .setLinearHeadingInterpolation(collect1Pose.getHeading(), shootPose.getHeading())
                .build();

        driveShootToCollect2Pose = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootPose,
                        new Pose(61.073170731707314, 62.048780487804876, 0),
                        new Pose(76.29268292682927, 59.90243902439026, 0),
                        collect2Pose
                ))
                .setLinearHeadingInterpolation(shootPose.getHeading(), collect2Pose.getHeading())
                .build();

        driveCollect2ToShootPose = follower.pathBuilder()
                .addPath(new BezierLine(collect2Pose, shootPose))
                .setLinearHeadingInterpolation(collect2Pose.getHeading(), shootPose.getHeading())
                .build();

        driveShootToCollect3Pose = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootPose,
                        new Pose(54.048780487804876, 39.41463414634147, 0),
                        new Pose(52.292682926829265, 34.53658536585366, 0),
                        collect3Pose
                ))
                .setLinearHeadingInterpolation(shootPose.getHeading(), collect3Pose.getHeading())
                .build();

        driveCollect3ToShootPose = follower.pathBuilder()
                .addPath(new BezierLine(collect3Pose, shootPose))
                .setLinearHeadingInterpolation(collect3Pose.getHeading(), shootPose.getHeading())
                .build();

        driveShootToEndPose = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, endPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading())
                .build();
    }

    public void statePathUpdate() {
        switch (pathState){
            case DRIVE_START_TO_SHOOT_POS:
                follower.followPath(driveStartToShootPos,true);
                setPathState(PathState.SHOOT_PRELOAD);
                break;

            case SHOOT_PRELOAD:
                if (!follower.isBusy()) {
                    shootMotor.setMotorSpeed(1.0);
                    // Waiting 5 seconds to shoot
                    if (pathTimer.getElapsedTime() < 5000) {
                        telemetry.addLine("Shooting preload...");
                    } else {
                        shootMotor.setMotorSpeed(0);
                        follower.followPath(driveShootToCollect1Pose, true);
                        setPathState(PathState.DRIVE_SHOOT_TO_COLLECT1_POSE);
                    }
                }
                break;

            case DRIVE_SHOOT_TO_COLLECT1_POSE:
                if (!follower.isBusy()){
                    // Arrived at Collect 1. Switch to WAIT to settle.
                    telemetry.addLine("Arrived Collect 1, Settling...");
                    setPathState(PathState.WAIT_AT_COLLECT1);
                }
                break;

            case WAIT_AT_COLLECT1:
                // Wait 100ms to fix angle and stop momentum
                if(pathTimer.getElapsedTime() > 100) {
                    telemetry.addLine("Going to Shoot from Collect 1");
                    follower.followPath(driveCollect1ToShootPose, true);
                    setPathState(PathState.DRIVE_COLLECT1_TO_SHOOT_POSE);
                }
                break;

            case DRIVE_COLLECT1_TO_SHOOT_POSE:
                if (!follower.isBusy()) {
                    shootMotor.setMotorSpeed(1.0);
                    if (pathTimer.getElapsedTime() < 5000) {
                        telemetry.addLine("Shooting after Collect 1...");
                    } else {
                        shootMotor.setMotorSpeed(0);
                        follower.followPath(driveShootToCollect2Pose, true);
                        setPathState(PathState.DRIVE_SHOOT_TO_COLLECT2_POSE);
                    }
                }
                break;

            case DRIVE_SHOOT_TO_COLLECT2_POSE:
                if (!follower.isBusy()){
                    // Arrived at Collect 2. Switch to WAIT to settle.
                    telemetry.addLine("Arrived Collect 2, Settling...");
                    setPathState(PathState.WAIT_AT_COLLECT2);
                }
                break;

            case WAIT_AT_COLLECT2:
                // Wait 100ms to fix angle and stop momentum
                if(pathTimer.getElapsedTime() > 100) {
                    telemetry.addLine("Going to Shoot from Collect 2");
                    follower.followPath(driveCollect2ToShootPose, true);
                    setPathState(PathState.DRIVE_COLLECT2_TO_SHOOT_POSE);
                }
                break;

            case DRIVE_COLLECT2_TO_SHOOT_POSE:
                if (!follower.isBusy()) {
                    shootMotor.setMotorSpeed(1.0);
                    if (pathTimer.getElapsedTime() < 5000) {
                        telemetry.addLine("Shooting after Collect 2...");
                    } else {
                        shootMotor.setMotorSpeed(0);
                        follower.followPath(driveShootToCollect3Pose, true);
                        setPathState(PathState.DRIVE_SHOOT_TO_COLLECT3_POSE);
                    }
                }
                break;

            case DRIVE_SHOOT_TO_COLLECT3_POSE:
                if (!follower.isBusy()){
                    // Arrived at Collect 3. Switch to WAIT to settle.
                    telemetry.addLine("Arrived Collect 3, Settling...");
                    setPathState(PathState.WAIT_AT_COLLECT3);
                }
                break;

            case WAIT_AT_COLLECT3:
                // Wait 100ms to fix angle and stop momentum
                if(pathTimer.getElapsedTime() > 100) {
                    telemetry.addLine("Going to Shoot from Collect 3");
                    follower.followPath(driveCollect3ToShootPose, true);
                    setPathState(PathState.DRIVE_COLLECT3_TO_SHOOT_POSE);
                }
                break;

            case DRIVE_COLLECT3_TO_SHOOT_POSE:
                if (!follower.isBusy()) {
                    shootMotor.setMotorSpeed(1.0);
                    if (pathTimer.getElapsedTime() < 5000) {
                        telemetry.addLine("Shooting after Collect 3...");
                    } else {
                        shootMotor.setMotorSpeed(0);
                        follower.followPath(driveShootToEndPose, true);
                        setPathState(PathState.DRIVE_SHOOT_TO_END_POSE);
                    }
                }
                break;

            case DRIVE_SHOOT_TO_END_POSE:
                if (!follower.isBusy()){
                    shootMotor.setMotorSpeed(0);
                    telemetry.addLine("All Paths Done");
                }
                break;

            default:
                telemetry.addLine("no State Commanded");
                break;
        }
    }

    public void setPathState(PathState newState){
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        pathState = PathState.DRIVE_START_TO_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        shootMotor = new shootMotorInit();
        shootMotor.init(hardwareMap);
        buildPaths();
        follower.setPose(startPose);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    public void start(){
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        telemetry.addData("path state", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Path Time", pathTimer.getElapsedTime());
        telemetry.addData("Motor RPM", shootMotor.getMotorRevs());
        telemetry.update();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }
}