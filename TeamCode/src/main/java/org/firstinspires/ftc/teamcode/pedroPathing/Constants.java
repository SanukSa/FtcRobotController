package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.control.FilteredPIDFCoefficients;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(5.5)
            .forwardZeroPowerAcceleration(-56.42879761)
            .lateralZeroPowerAcceleration(-73.9574885591783)
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.05,
                    0,
                    0.001,
                    0.03
            ))
            .headingPIDFCoefficients(new PIDFCoefficients(
                    0.8,
                    0,
                    0.03,
                    0.028
            ))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.01,
                    0.0,
                    0.0002,
                    0.0,
                    0.03
            ))
            .centripetalScaling(0.025);
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1) //The Second Set
            .leftFrontMotorName("flMotor")
            .leftRearMotorName("blMotor")
            .rightFrontMotorName("frMotor")
            .rightRearMotorName("brMotor")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(51.66652688839821)
            .yVelocity(41.10652143555786);

    public static ThreeWheelIMUConstants localizerConstants = new ThreeWheelIMUConstants()
            .forwardTicksToInches( 0.0009161284127254752)
            .strafeTicksToInches(0.0009416652751526144)
            .turnTicksToInches(0.0019842565815800695)
            .leftPodY(101/25.4) //The Third Set
            .rightPodY(-99/25.4)
            .strafePodX(-106/25.4)
            .leftEncoder_HardwareMapName("leftDeadwheel")
            .rightEncoder_HardwareMapName("rightDeadwheel")
            .strafeEncoder_HardwareMapName("strafeDeadwheel")
            .leftEncoderDirection(Encoder.REVERSE)
            .rightEncoderDirection(Encoder.REVERSE)
            .strafeEncoderDirection(Encoder.FORWARD)
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP));


    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1.3, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .threeWheelIMULocalizer(localizerConstants)
                .build();
    }
}
