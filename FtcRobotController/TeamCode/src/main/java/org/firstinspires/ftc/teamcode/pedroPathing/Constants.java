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
            .mass(12.4)
            .forwardZeroPowerAcceleration(-34.851204314629776)
            .lateralZeroPowerAcceleration(-60.111926352626405)
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true)
            .centripetalScaling(0.00046)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.02, 0.15))
            .headingPIDFCoefficients(new PIDFCoefficients(0.25, 0.001, 0.001, 0.1))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.02, 0, 0.0001, 0.6, 0.1))
            .secondaryTranslationalPIDFCoefficients(
                    new PIDFCoefficients(0.055, 0.000001, 0, 0.02)
            )
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.9, 0.01, 0.1, 0.01))
            .secondaryDrivePIDFCoefficients(
                    new FilteredPIDFCoefficients(0.01, 0, 0, 0.6, 0)
            );

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .leftFrontMotorName("frontLeft")
            .leftRearMotorName("backLeft")
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .useBrakeModeInTeleOp(true)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(83.86456058532234)
            .yVelocity(66.42380121186025)
            .nominalVoltage(13.3)
            .useVoltageCompensation(true);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(48)
            .strafePodX(-154.05)
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
            1.125,
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
