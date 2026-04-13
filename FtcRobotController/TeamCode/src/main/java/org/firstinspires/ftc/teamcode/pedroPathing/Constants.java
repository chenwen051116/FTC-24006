package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class    Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12.524)
            .forwardZeroPowerAcceleration(-31.203684649136637)
            .lateralZeroPowerAcceleration(-76.08349200453131)
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true)
            .centripetalScaling(0.00023)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.025, 0.15))
            .headingPIDFCoefficients(new PIDFCoefficients(0.3, 0.001, 0.002, 0.1))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.01, 0, 0.0009, 0.6, 0.001))
            .secondaryTranslationalPIDFCoefficients(
                    new PIDFCoefficients(0.05, 0.0001, 0, 0.02)
            )
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.7, 0.006, 0.05, 0.02))
            .secondaryDrivePIDFCoefficients(
                    new FilteredPIDFCoefficients(0.0005, 0.0001, 0.0009, 0.6, 0.0)
            );

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .leftFrontMotorName("frontLeft")
            .leftRearMotorName("backLeft")
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .useBrakeModeInTeleOp(true)
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(81.48237237404652)
            .yVelocity(63.01307209645669)
            .nominalVoltage(13.3)
            .useVoltageCompensation(true);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(64)
            .strafePodX(-157)
            .distanceUnit(DistanceUnit.MM)
            .hardwareMapName("pinpoint")
            .yawScalar(1.0)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
//            .customEncoderResolution(13.26291192)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.997,
            50,
            1,
            1

    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}
