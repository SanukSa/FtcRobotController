package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.mechanisms.shootMotorInit;

@Autonomous
public class rightAutoPathNoCurve extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;
    private shootMotorInit shootMotor;

    public enum PathState {
        DRIVE_START_TO_SHOOT,
        SHOOT_PRELOAD,

        DRIVE_SHOOT_TO_C1_Y,
        DRIVE_C1_Y_TO_C1_X,
        DRIVE_C1_X_TO_SHOOT,

        DRIVE_SHOOT_TO_C2_Y,
        DRIVE_C2_Y_TO_C2_X,
        DRIVE_C2_X_TO_SHOOT,

        DRIVE_SHOOT_TO_C3_Y,
        DRIVE_C3_Y_TO_C3_X,
        DRIVE_C3_X_TO_SHOOT
    }

    private PathState pathState;

    /* -------------------- POSES (MIRRORED ACROSS X=72) -------------------- */

    private final Pose startPose =
            new Pose(123.70731707317074, 122.73170731707316, Math.toRadians(144));

    // Mirrored X and H (180 - 36 = 144)
    private final Pose shootPose =
            new Pose(114.14634146341464, 112.78048780487804, Math.toRadians(36));

    // Collect 1 - Mirrored X and H (180 - 0 = 180)
    private final Pose collect1YPose =
            new Pose(93.85365853658537, 83.70731707317073, Math.toRadians(0));

    private final Pose collect1XPose =
            new Pose(131.3170731707317, 83.70731707317073, Math.toRadians(0));

    // Collect 2 - Mirrored X and H (180 - 0 = 180)
    private final Pose collect2YPose =
            new Pose(87.40878048780488, 60.29268292682927, Math.toRadians(0));

    // Mirrored X and H (180 - 36 = 144)
    private final Pose collect2XPose =
            new Pose(131.1219512195122, 59.90243902439026, Math.toRadians(36));

    // Collect 3 - Mirrored X and H (180 - 0 = 180)
    private final Pose collect3YPose =
            new Pose(81.95121951219512, 35.70731707317073, Math.toRadians(0));

    private final Pose collect3XPose =
            new Pose(129.95121951219512, 35.90243902439025, Math.toRadians(0));

    /* -------------------- PATHS (MIRRORED HEADINGS) -------------------- */

    private PathChain
            driveStartToShoot, shootToC1Y, c1YToC1X, c1XToShoot,
            shootToC2Y, c2YToC2X, c2XToShoot,
            shootToC3Y, c3YToC3X, c3XToShoot;

    public void buildPaths() {

        driveStartToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(
                        Math.toRadians(90), // 90 remains 90
                        Math.toRadians(144)) // 0 becomes 144 (based on old 36)
                .build();

        shootToC1Y = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, collect1YPose))
                .setLinearHeadingInterpolation(
                        Math.toRadians(144), // 36 becomes 144
                        Math.toRadians(180)) // 0 becomes 180
                .build();

        c1YToC1X = follower.pathBuilder()
                .addPath(new BezierLine(collect1YPose, collect1XPose))
                .setLinearHeadingInterpolation(
                        Math.toRadians(180), // 0 becomes 180
                        Math.toRadians(180)) // 0 becomes 180
                .build();

        c1XToShoot = follower.pathBuilder()
                .addPath(new BezierLine(collect1XPose, shootPose))
                .setLinearHeadingInterpolation(
                        Math.toRadians(180), // 0 becomes 180
                        Math.toRadians(144)) // 36 becomes 144
                .build();

        shootToC2Y = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, collect2YPose))
                .setLinearHeadingInterpolation(
                        Math.toRadians(144), // 36 becomes 144
                        Math.toRadians(180)) // 0 becomes 180
                .build();

        c2YToC2X = follower.pathBuilder()
                .addPath(new BezierLine(collect2YPose, collect2XPose))
                .setLinearHeadingInterpolation(
                        Math.toRadians(180), // 0 becomes 180
                        Math.toRadians(144)) // 36 becomes 144
                .build();

        c2XToShoot = follower.pathBuilder()
                .addPath(new BezierLine(collect2XPose, shootPose))
                .setLinearHeadingInterpolation(
                        Math.toRadians(144), // 36 becomes 144
                        Math.toRadians(144)) // 0 becomes 144 (based on old 36)
                .build();

        shootToC3Y = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, collect3YPose))
                .setLinearHeadingInterpolation(
                        Math.toRadians(144), // 0 becomes 144 (based on old 36)
                        Math.toRadians(180)) // 0 becomes 180
                .build();

        c3YToC3X = follower.pathBuilder()
                .addPath(new BezierLine(collect3YPose, collect3XPose))
                .setLinearHeadingInterpolation(
                        Math.toRadians(180), // 0 becomes 180
                        Math.toRadians(180)) // 0 becomes 180
                .build();

        c3XToShoot = follower.pathBuilder()
                .addPath(new BezierLine(collect3XPose, shootPose))
                .setLinearHeadingInterpolation(
                        Math.toRadians(180), // 0 becomes 180
                        Math.toRadians(144)) // 36 becomes 144
                .build();
    }

    /* -------------------- STATE MACHINE -------------------- */

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    public void statePathUpdate() {
        switch (pathState) {

            case DRIVE_START_TO_SHOOT:
                follower.followPath(driveStartToShoot, true);
                setPathState(PathState.SHOOT_PRELOAD);
                break;

            case SHOOT_PRELOAD:
                if (!follower.isBusy()) {
                    // Turn on shoot motor at full speed
                    shootMotor.setMotorSpeed(1.0);

                    if (pathTimer.getElapsedTime() > 5000) {
                        shootMotor.setMotorSpeed(0);
                        follower.followPath(shootToC1Y, true);
                        setPathState(PathState.DRIVE_SHOOT_TO_C1_Y);
                    } else {
                        telemetry.addLine("Shooting preload...");
                    }
                }
                break;

            case DRIVE_SHOOT_TO_C1_Y:
                if (!follower.isBusy()) {
                    follower.followPath(c1YToC1X, true);
                    setPathState(PathState.DRIVE_C1_Y_TO_C1_X);
                }
                break;

            case DRIVE_C1_Y_TO_C1_X:
                if (!follower.isBusy()) {
                    follower.followPath(c1XToShoot, true);
                    setPathState(PathState.DRIVE_C1_X_TO_SHOOT);
                }
                break;

            case DRIVE_C1_X_TO_SHOOT:
                if (!follower.isBusy()) {
                    // Turn on shoot motor at full speed
                    shootMotor.setMotorSpeed(1.0);

                    if (pathTimer.getElapsedTime() > 5000) {
                        shootMotor.setMotorSpeed(0);
                        follower.followPath(shootToC2Y, true);
                        setPathState(PathState.DRIVE_SHOOT_TO_C2_Y);
                    } else {
                        telemetry.addLine("Shooting after Collect 1...");
                    }
                }
                break;

            case DRIVE_SHOOT_TO_C2_Y:
                if (!follower.isBusy()) {
                    follower.followPath(c2YToC2X, true);
                    setPathState(PathState.DRIVE_C2_Y_TO_C2_X);
                }
                break;

            case DRIVE_C2_Y_TO_C2_X:
                if (!follower.isBusy()) {
                    follower.followPath(c2XToShoot, true);
                    setPathState(PathState.DRIVE_C2_X_TO_SHOOT);
                }
                break;

            case DRIVE_C2_X_TO_SHOOT:
                if (!follower.isBusy()) {
                    // Turn on shoot motor at full speed
                    shootMotor.setMotorSpeed(1.0);

                    if (pathTimer.getElapsedTime() > 5000) {
                        shootMotor.setMotorSpeed(0);
                        follower.followPath(shootToC3Y, true);
                        setPathState(PathState.DRIVE_SHOOT_TO_C3_Y);
                    } else {
                        telemetry.addLine("Shooting after Collect 2...");
                    }
                }
                break;

            case DRIVE_SHOOT_TO_C3_Y:
                if (!follower.isBusy()) {
                    follower.followPath(c3YToC3X, true);
                    setPathState(PathState.DRIVE_C3_Y_TO_C3_X);
                }
                break;

            case DRIVE_C3_Y_TO_C3_X:
                if (!follower.isBusy()) {
                    follower.followPath(c3XToShoot, true);
                    setPathState(PathState.DRIVE_C3_X_TO_SHOOT);
                }
                break;

            case DRIVE_C3_X_TO_SHOOT:
                if (!follower.isBusy()) {
                    // Turn on shoot motor at full speed
                    shootMotor.setMotorSpeed(1.0);

                    if (pathTimer.getElapsedTime() > 5000) {
                        shootMotor.setMotorSpeed(0);
                        telemetry.addLine("AUTO COMPLETE");
                    } else {
                        telemetry.addLine("Shooting after Collect 3...");
                    }
                }
                break;
        }
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        pathTimer = new Timer();
        opModeTimer = new Timer();
        shootMotor = new shootMotorInit();  // Initialize shoot motor
        shootMotor.init(hardwareMap);       // Initialize with hardware map

        buildPaths();
        follower.setPose(startPose);
        pathState = PathState.DRIVE_START_TO_SHOOT;
    }

    @Override
    public void start() {
        setPathState(pathState);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        telemetry.addData("State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Path Time", pathTimer.getElapsedTime());
        telemetry.addData("Motor RPM", shootMotor.getMotorRevs());  // Show motor speed
        telemetry.update();
    }
}