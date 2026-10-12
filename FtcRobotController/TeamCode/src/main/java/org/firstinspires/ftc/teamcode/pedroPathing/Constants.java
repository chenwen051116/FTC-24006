package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * ALL VALUES BELOW ARE PLACEHOLDERS -- this robot has not been tuned yet.
 *
 * Tuning order (AutoTune webpage: http://192.168.43.1:10158 while connected
 * to the Robot Controller, see https://pedropathing.com/docs/pathing/tuning):
 *
 *   1. Mecanum AutoTuner  -> paste generated block over drivetrainConfig below
 *   2. Pinpoint AutoTuner -> paste generated block over localizerConfig below
 *   3. Foresight AutoTuner-> paste generated block over foresightConfig below
 *      (Foresight tuning requires drivetrainConfig and localizerConfig to be done first)
 */
public class Constants {

    // TODO: replace with the output of the Mecanum AutoTuner.
    // Motor names below must match the Robot Configuration; the tuner
    // determines the directions by spinning each motor.
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("frontLeft");
        c.frontRightName.set("frontRight");
        c.backLeftName.set("backLeft");
        c.backRightName.set("backRight");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    // TODO: replace with the output of the Pinpoint AutoTuner
    // (it determines pod directions and the x/y pod offsets).
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-6.6142);
        c.yPodOffset.set(-3.7795);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    // TODO: replace with the output of the Foresight AutoTuner.
    // The values below are generic placeholders, NOT tuned for this robot.
    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
        Controller primaryTranslationalForward = Controller.proportional(0.3);
        Controller secondaryTranslationalForward = Controller.proportional(0.1);
        Controller primaryTranslationalLateral = Controller.proportional(0.3);
        Controller secondaryTranslationalLateral = Controller.proportional(0.1);

        c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
        c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

        c.coast.set(Controller.proportionalFeedforward(0.011));
        c.brake.set(Controller.proportionalFeedforward(0.009));

        c.headingFeedback.set(Controller.proportional(5.0));
        c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05, 0.006));

        c.linearBrakeCoefficients.set(Matrix.diag(0.1, 0.09));
        c.quadraticBrakeCoefficients.set(Matrix.diag(0.0015, 0.0014));

        c.maxAchievableForwardVelocity.set(70.0);
        c.maxAchievableStrafeVelocity.set(50.0);
        c.naturalForwardDeceleration.set(80.0);
        c.naturalStrafeDeceleration.set(100.0);
    });

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new Follower(
                new PinpointLocalizer(hardwareMap, localizerConfig),
                new Mecanum(hardwareMap, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}
