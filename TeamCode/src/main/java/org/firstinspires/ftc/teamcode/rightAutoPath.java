package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class rightAutoPath extends OpMode {

    double timeAfterStart = 0;
    private Follower follower;
    private Timer pathTimer, opModeTimer;


    public enum PathState{
        // Start to Shoot POS
        // Drive State
        // Shoot State

        DRIVE_START_TO_SHOOT_POS,
        SHOOT_PRELOAD,
        DRIVE_SHOOT_TO_COLLECT1_POSE,
        DRIVE_COLLECT1_TO_SHOOT_POSE,
        DRIVE_SHOOT_TO_COLLECT2_POSE,
        DRIVE_COLLECT2_TO_SHOOT_POSE,
        DRIVE_SHOOT_TO_COLLECT3_POSE,
        DRIVE_COLLECT3_TO_SHOOT_POSE,
        DRIVE_SHOOT_TO_END_POSE

    }

    PathState pathState;

    private final Pose startPose = new Pose(124.68292682926829,124.29268292682926, Math.toRadians(35));
    private final Pose shootPose = new Pose(114.92682926829269,114.73170731707317, Math.toRadians(45));
    private final Pose collect1Pose = new Pose(131.51219512195122, 83.70731707317073, Math.toRadians(0));
    private final Pose collect2Pose = new Pose(130.73170731707317, 59.707317073170735, Math.toRadians(0));
    private final Pose collect3Pose = new Pose(132.6829268292683, 35.51219512195122, Math.toRadians(0));
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
                        new Pose(63.80487804878048, 80.97560975609755,0),
                        collect1Pose
                ))
                .setLinearHeadingInterpolation(shootPose.getHeading(), collect1Pose.getHeading())
                .build();

        driveCollect1ToShootPose = follower.pathBuilder()
                .addPath(new BezierLine(collect1Pose, shootPose))
                .setLinearHeadingInterpolation(collect1Pose.getHeading(), shootPose.getHeading())
                .build();

        // Do the Shoot to Collect 2 Pose - Should be a curve
        driveShootToCollect2Pose = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootPose,
                        new Pose(53.46341463414635, 59.12195121951219, 0),
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
                        new Pose(40.58536585365853, 31.024390243902438,0),
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
                setPathState(PathState.SHOOT_PRELOAD); // resets the timer & sets state
                break;
            case SHOOT_PRELOAD:
                if (!follower.isBusy()) {          // Only start counting after the bot reaches shoot pose
                    if (pathTimer.getElapsedTime() < 5000) {
                        // Stay here, do nothing
                        telemetry.addLine("Waiting at Shoot Pos...");
                    } else {
                        // 5 seconds passed → go to next path
                        follower.followPath(driveShootToCollect1Pose, true);
                        setPathState(PathState.DRIVE_SHOOT_TO_COLLECT1_POSE);
                    }
                }
                break;

            case DRIVE_SHOOT_TO_COLLECT1_POSE:
                if (!follower.isBusy()){
                    telemetry.addLine("Going to Collect at Collect 1 Pose");
                    follower.followPath(driveCollect1ToShootPose, true);
                    setPathState(PathState.DRIVE_COLLECT1_TO_SHOOT_POSE);
                }
                break;

            case DRIVE_COLLECT1_TO_SHOOT_POSE:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTime() < 5000) {
                        telemetry.addLine("Waiting at Shoot Pos after Collect 1...");
                    } else {
                        telemetry.addLine("Going to Collect 2");
                        follower.followPath(driveShootToCollect2Pose, true);
                        setPathState(PathState.DRIVE_SHOOT_TO_COLLECT2_POSE);
                    }
                }
                break;

            case DRIVE_SHOOT_TO_COLLECT2_POSE:
                if (!follower.isBusy()){
                    telemetry.addLine("Going to Collect at Collect 2 Pose");
                    follower.followPath(driveCollect2ToShootPose, true);
                    setPathState(PathState.DRIVE_COLLECT2_TO_SHOOT_POSE);
                }

                break;

            case DRIVE_COLLECT2_TO_SHOOT_POSE:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTime() < 5000) {
                        telemetry.addLine("Waiting at Shoot Pos after Collect 2...");
                    } else {
                        telemetry.addLine("Going to Collect 3");
                        follower.followPath(driveShootToCollect3Pose, true);
                        setPathState(PathState.DRIVE_SHOOT_TO_COLLECT3_POSE);
                    }
                }
                break;

            case DRIVE_SHOOT_TO_COLLECT3_POSE:
                if (!follower.isBusy()){
                    telemetry.addLine("Going to Collect at Collect 3 Pose");
                    follower.followPath(driveCollect3ToShootPose, true);
                    setPathState(PathState.DRIVE_COLLECT3_TO_SHOOT_POSE);
                }

                break;
            case DRIVE_COLLECT3_TO_SHOOT_POSE:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTime() < 5000) {
                        telemetry.addLine("Waiting at Shoot Pos after Collect 3...");
                    } else {
                        telemetry.addLine("Going to End Pose");
                        follower.followPath(driveShootToEndPose, true);
                        setPathState(PathState.DRIVE_SHOOT_TO_END_POSE);
                    }
                }
                break;
            case DRIVE_SHOOT_TO_END_POSE:
                if (!follower.isBusy()){
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
        pathState = PathState.DRIVE_START_TO_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setPose(startPose);
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
        telemetry.update();
    }
}
