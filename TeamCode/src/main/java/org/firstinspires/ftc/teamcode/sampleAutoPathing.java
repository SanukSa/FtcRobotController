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
public class sampleAutoPathing extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState{
        // Start to Shoot POS
        // Drive State
        // Shoot State

        DRIVE_START_TO_SHOOT_POS,
        SHOOT_PRELOAD,
        COLLECT1_POSE
    }

    PathState pathState;

    private final Pose startPose = new Pose(19.31707317073171,124.29268292682926, Math.toRadians(145));
    private final Pose shootPose = new Pose(29.073170731707314,114.73170731707317, Math.toRadians(126));
    private final Pose collect1Pose = new Pose(12.487804878048781, 83.70731707317073, Math.toRadians(180));
    private final Pose collect2Pose = new Pose(13.26829268292683, 59.707317073170735, Math.toRadians(180));
    private final Pose collect3Pose = new Pose(11.317073170731707, 35.51219512195122, Math.toRadians(180));
    private final Pose endPose = new Pose(52.09756097560976, 120.1951219512195, Math.toRadians(136));

    private PathChain driveStartToShootPos, driveShootToCollect1Pose, driveCollect1ToShootPose, driveShootToCollect2Pose, driveCollect2ToShootPose,
    driveShootToCollect3Pose, driveCollect3ToShootPose, driveShootToEndPose;

    public void buildPaths() {
        driveStartToShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        driveShootToCollect1Pose = follower.pathBuilder()
                .addPath(new BezierCurve(
                collect1Pose,
                new Pose(80.19512195121952, 80.97560975609755,0),
                shootPose
        ))
                .setLinearHeadingInterpolation(
                        collect1Pose.getHeading(),
                        shootPose.getHeading()
                )
                .build();

        driveCollect1ToShootPose = follower.pathBuilder()
                .addPath(new BezierLine(collect1Pose, shootPose))
                .setLinearHeadingInterpolation(collect1Pose.getHeading(), shootPose.getHeading())
                .build();

        // Do the Shoot to Collect 2 Pose - Should be a curve

    }

    public void statePathUpdate() {
        switch (pathState){
            case DRIVE_START_TO_SHOOT_POS:
                follower.followPath(driveStartToShootPos,true);
                setPathState(PathState.SHOOT_PRELOAD); // resets the timer & sets state
                break;
            case SHOOT_PRELOAD:
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 5){
                    telemetry.addLine("Done State Mechine, Robot is at Shoot Pos,Waiting for Flywheel");
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
    }
}
