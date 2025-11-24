package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.util.Timer;

import java.nio.file.Path;

@Autonomous
public class sampleAutoPathing extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState{
        // Start to Shoot POS
        // Drive State
        // Shoot State

        DRIVE_START_TO_SHOOT_POS,
        SHOOT_PRELOAD
    }

    PathState pathState;

    private final Pose startPose = new Pose(61.65853658536585,13.658536585365848, Math.toRadians(44));
    private final Pose shootPose = new Pose(78.82926829268293,81.95121951219512, Math.toRadians(48));

    private PathChain driveStartToShootPos;

    public void buildPaths() {
        driveStartToShootPos = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

    }

    public void statePathUpadte() {
        switch (pathState){
            case DRIVE_START_TO_SHOOT_POS:
                follower.followPath(driveStartToShootPos,true);
                pathState = pathState.SHOOT_PRELOAD;
                break;
            case SHOOT_PRELOAD:
                if (!follower.isBusy()){
                    telemetry.addLine("Done State Mechine, Robot is at Shoot Pos,Waiting for Flywheel");

                }
                break;
            default:
                telemetry.addLine("no State Commanded");
                break;

        }

    }

    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }
}
