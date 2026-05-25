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
            .mass(12.2)
            .forwardZeroPowerAcceleration(-29.465452204903155)
            .lateralZeroPowerAcceleration(-62.6584184654734)
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true)
            .centripetalScaling(0.00048)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0.0000, 0.017, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(0.6, 0.000, 0.001, 0.05))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.008, 0, 0.0005, 0.6, 0.1))
            .secondaryTranslationalPIDFCoefficients(
                    new PIDFCoefficients(0.1, 0.000, 0.01, 0.03   )
            )
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.9, 0.01, 0.05, 0.02))
            .secondaryDrivePIDFCoefficients(
                    new FilteredPIDFCoefficients(0.02, 0.001, 0.0001, 0.6, 0)
           )
    ;

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .leftFrontMotorName("frontLeft")
            .leftRearMotorName("backLeft")
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .useBrakeModeInTeleOp(true)
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(70.28555033526084)
            .yVelocity(56.00087083230808)
            .nominalVoltage(13.2)
            .useVoltageCompensation(true);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(24)
            .strafePodX(-120)
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
            0.5,
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
