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
public class rightAutoCurveAndLine extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;
    private shootMotorInit shootMotor;

    private TelemetryManager panelsTelemetry;

    public enum PathState{
        DRIVE_START_TO_SHOOT_POS,
        SHOOT_PRELOAD,
        DRIVE_SHOOT_TO_COLLECT1_PRE,
        DRIVE_COLLECT1_PRE_TO_COLLECT1,
        WAIT_AT_COLLECT1,              // New Wait State
        DRIVE_COLLECT1_TO_SHOOT_POSE,
        DRIVE_SHOOT_TO_COLLECT2_PRE,
        DRIVE_COLLECT2_PRE_TO_COLLECT2,
        WAIT_AT_COLLECT2,              // New Wait State
        DRIVE_COLLECT2_TO_SHOOT_POSE,
        DRIVE_SHOOT_TO_COLLECT3_PRE,
        DRIVE_COLLECT3_PRE_TO_COLLECT3,
        WAIT_AT_COLLECT3,              // New Wait State
        DRIVE_COLLECT3_TO_SHOOT_POSE,
        DRIVE_SHOOT_TO_END_POSE
    }

    PathState pathState;

    // Coordinates
    private final Pose startPose = new Pose(124.68292682926829, 124.29268292682926, Math.toRadians(35));
    private final Pose shootPose = new Pose(114.92682926829269, 114.73170731707317, Math.toRadians(45));

    // UPDATED: Headings set to 0 (Perpendicular)
    private final Pose collect1Pre = new Pose(103.8048780487805, 83.51219512195122, Math.toRadians(0));
    private final Pose collect1Pose = new Pose(124.48780487804879, 83.51219512195122, Math.toRadians(0));
    private final Pose collect2Pre = new Pose(95.21951219512195, 59.512195121951216, Math.toRadians(0));
    private final Pose collect2Pose = new Pose(124.87804878048782, 59.512195121951216, Math.toRadians(0));
    private final Pose collect3Pre = new Pose(83.1219512195122, 35.51219512195122, Math.toRadians(0));
    private final Pose collect3Pose = new Pose(125.07317073170732, 35.3170731707317, Math.toRadians(0));

    private final Pose endPose = new Pose(91.90243902439024, 120.1951219512195, Math.toRadians(44));

    private PathChain driveStartToShootPos, driveShootToCollect1Pre, driveCollect1PreToCollect1, driveCollect1ToShootPose,
            driveShootToCollect2Pre, driveCollect2PreToCollect2, driveCollect2ToShootPose, driveShootToCollect3Pre, driveCollect3PreToCollect3,
            driveCollect3ToShootPose, driveShootToEndPose;

    public void buildPaths() {
        driveStartToShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        driveShootToCollect1Pre = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootPose,
                        new Pose(93.65853658536584, 97.75609756097562, 0),
                        collect1Pre
                ))
                .setLinearHeadingInterpolation(shootPose.getHeading(), collect1Pre.getHeading())
                .build();

        driveCollect1PreToCollect1 = follower.pathBuilder()
                .addPath(new BezierLine(collect1Pre, collect1Pose))
                .setLinearHeadingInterpolation(collect1Pre.getHeading(), collect1Pose.getHeading())
                .build();

        driveCollect1ToShootPose = follower.pathBuilder()
                .addPath(new BezierLine(collect1Pose, shootPose))
                .setLinearHeadingInterpolation(collect1Pose.getHeading(), shootPose.getHeading())
                .build();

        driveShootToCollect2Pre = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootPose,
                        new Pose(73.7560975609756, 79.41463414634146, 0),
                        collect2Pre
                ))
                .setLinearHeadingInterpolation(shootPose.getHeading(), collect2Pre.getHeading())
                .build();

        driveCollect2PreToCollect2 = follower.pathBuilder()
                .addPath(new BezierLine(collect2Pre, collect2Pose))
                .setLinearHeadingInterpolation(collect2Pre.getHeading(), collect2Pose.getHeading())
                .build();


        driveCollect2ToShootPose = follower.pathBuilder()
                .addPath(new BezierLine(collect2Pose, shootPose))
                .setLinearHeadingInterpolation(collect2Pose.getHeading(), shootPose.getHeading())
                .build();

        driveShootToCollect3Pre = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootPose,
                        new Pose(64.1951219512195, 69.85365853658536, 0),
                        collect3Pre
                ))
                .setLinearHeadingInterpolation(shootPose.getHeading(), collect3Pre.getHeading())
                .build();

        driveCollect3PreToCollect3 = follower.pathBuilder()
                .addPath(new BezierLine(collect3Pre, collect3Pose))
                .setLinearHeadingInterpolation(collect3Pre.getHeading(), collect3Pose.getHeading())
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
        switch (pathState) {

            /* ---------------- START → SHOOT ---------------- */
            case DRIVE_START_TO_SHOOT_POS:
                follower.followPath(driveStartToShootPos, true);
                setPathState(PathState.SHOOT_PRELOAD);
                break;

            case SHOOT_PRELOAD:
                if (!follower.isBusy()) {
                    shootMotor.setMotorSpeed(1.0);

                    if (pathTimer.getElapsedTime() >= 5000) {
                        shootMotor.setMotorSpeed(0);
                        follower.followPath(driveShootToCollect1Pre, true);
                        setPathState(PathState.DRIVE_SHOOT_TO_COLLECT1_PRE);
                    }
                }
                break;

            /* ---------------- COLLECT 1 ---------------- */
            case DRIVE_SHOOT_TO_COLLECT1_PRE:
                if (!follower.isBusy()) {
                    follower.followPath(driveCollect1PreToCollect1, true);
                    setPathState(PathState.DRIVE_COLLECT1_PRE_TO_COLLECT1);
                }
                break;

            case DRIVE_COLLECT1_PRE_TO_COLLECT1:
                if (!follower.isBusy()) {
                    setPathState(PathState.WAIT_AT_COLLECT1);
                }
                break;

            case WAIT_AT_COLLECT1:
                if (pathTimer.getElapsedTime() >= 100) {
                    follower.followPath(driveCollect1ToShootPose, true);
                    setPathState(PathState.DRIVE_COLLECT1_TO_SHOOT_POSE);
                }
                break;

            case DRIVE_COLLECT1_TO_SHOOT_POSE:
                if (!follower.isBusy()) {
                    shootMotor.setMotorSpeed(1.0);

                    if (pathTimer.getElapsedTime() >= 5000) {
                        shootMotor.setMotorSpeed(0);
                        follower.followPath(driveShootToCollect2Pre, true);
                        setPathState(PathState.DRIVE_SHOOT_TO_COLLECT2_PRE);
                    }
                }
                break;

            /* ---------------- COLLECT 2 ---------------- */
            case DRIVE_SHOOT_TO_COLLECT2_PRE:
                if (!follower.isBusy()) {
                    follower.followPath(driveCollect2PreToCollect2, true);
                    setPathState(PathState.DRIVE_COLLECT2_PRE_TO_COLLECT2);
                }
                break;

            case DRIVE_COLLECT2_PRE_TO_COLLECT2:
                if (!follower.isBusy()) {
                    setPathState(PathState.WAIT_AT_COLLECT2);
                }
                break;

            case WAIT_AT_COLLECT2:
                if (pathTimer.getElapsedTime() >= 100) {
                    follower.followPath(driveCollect2ToShootPose, true);
                    setPathState(PathState.DRIVE_COLLECT2_TO_SHOOT_POSE);
                }
                break;

            case DRIVE_COLLECT2_TO_SHOOT_POSE:
                if (!follower.isBusy()) {
                    shootMotor.setMotorSpeed(1.0);

                    if (pathTimer.getElapsedTime() >= 5000) {
                        shootMotor.setMotorSpeed(0);
                        follower.followPath(driveShootToCollect3Pre, true);
                        setPathState(PathState.DRIVE_SHOOT_TO_COLLECT3_PRE);
                    }
                }
                break;

            /* ---------------- COLLECT 3 ---------------- */
            case DRIVE_SHOOT_TO_COLLECT3_PRE:
                if (!follower.isBusy()) {
                    follower.followPath(driveCollect3PreToCollect3, true);
                    setPathState(PathState.DRIVE_COLLECT3_PRE_TO_COLLECT3);
                }
                break;

            case DRIVE_COLLECT3_PRE_TO_COLLECT3:
                if (!follower.isBusy()) {
                    setPathState(PathState.WAIT_AT_COLLECT3);
                }
                break;

            case WAIT_AT_COLLECT3:
                if (pathTimer.getElapsedTime() >= 100) {
                    follower.followPath(driveCollect3ToShootPose, true);
                    setPathState(PathState.DRIVE_COLLECT3_TO_SHOOT_POSE);
                }
                break;

            case DRIVE_COLLECT3_TO_SHOOT_POSE:
                if (!follower.isBusy()) {
                    shootMotor.setMotorSpeed(1.0);

                    if (pathTimer.getElapsedTime() >= 5000) {
                        shootMotor.setMotorSpeed(0);
                        follower.followPath(driveShootToEndPose, true);
                        setPathState(PathState.DRIVE_SHOOT_TO_END_POSE);
                    }
                }
                break;

            /* ---------------- END ---------------- */
            case DRIVE_SHOOT_TO_END_POSE:
                if (!follower.isBusy()) {
                    shootMotor.setMotorSpeed(0);
                    telemetry.addLine("Autonomous Complete");
                }
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