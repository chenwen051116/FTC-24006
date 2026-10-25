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
        c.xPodOffset.set( 1.4173);
        c.yPodOffset.set(-3.7795 );
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    // TODO: replace with the output of the Foresight AutoTuner.
    // The values below are generic placeholders, NOT tuned for this robot.

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.32065810274789963);
                Controller secondaryTranslationalForward = Controller.proportional(0.11847460082304175);
                Controller primaryTranslationalLateral = Controller.proportional(0.5925391169367593);
                Controller secondaryTranslationalLateral = Controller.proportional(0.21892737077132868);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.012770282032157047));
                c.brake.set(Controller.proportionalFeedforward(0.010854739727333489));

                c.headingFeedback.set(Controller.proportional(5.268779667719755));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.046067322695098105, 0.005554808292441821));

                c.linearBrakeCoefficients.set(Matrix.diag(0.11899024708846057, 0.05107559444359703));
                c.quadraticBrakeCoefficients.set(Matrix.diag(8.334242374255856E-4, 0.0019602491986134385));

                c.maxAchievableForwardVelocity.set(78.1162579989469);
                c.maxAchievableStrafeVelocity.set(62.46030653073429);
                c.naturalForwardDeceleration.set(31.212995247412422);
                c.naturalStrafeDeceleration.set(64.9688626312708);
            }
    );
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new Follower(
                new PinpointLocalizer(hardwareMap, localizerConfig),
                new Mecanum(hardwareMap, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}
