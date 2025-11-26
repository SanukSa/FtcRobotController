package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Go 48 Inches")
public class go48 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // create follower using your Constants (motor names, localizer, etc.)
        Follower follower = Constants.createFollower(hardwareMap);

        // starting pose: x=0, y=0, heading=0 radians (facing +X)
        Pose start = new Pose(0, 0, 0);

        // end pose: 48 inches forward on the +X axis, same heading
        Pose end = new Pose(48, 0, 0);

        // give the follower the starting pose
        follower.setPose(start);

        // build a PathChain by adding a single BezierLine from start -> end
        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(start, end))
                // linearly interpolate heading from start to end (they're the same here)
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .build();

        waitForStart();
        if (isStopRequested()) return;

        // follow the path
        follower.followPath(path);

        // keep updating the follower while it's busy
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            // optional: telemetry.update() if you want debug info
        }
    }
}
