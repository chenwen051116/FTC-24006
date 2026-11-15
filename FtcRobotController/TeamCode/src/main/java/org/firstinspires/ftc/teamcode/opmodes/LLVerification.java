package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.reflection.ReflectionConfig;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.models.RedMlp;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;

import java.util.List;

/**
 * Live check: does the Limelight track the Pinpoint odometry, and does the
 * trained MLP beat the traditional rigid transform?
 *
 * Drives like a normal teleop. The first valid Limelight field pose becomes
 * the "bridge" (one captured odo<->LL pair); X re-bridges (park first).
 *
 * Yaw is NEVER predicted -- the encoder/gyro heading is always correct
 * (MegaTag2 even locks to the yaw we feed it); only xy can drift. So both
 * models are compared on xy only, as displacements since the bridge:
 *
 *   SE3 pred (traditional): the displacement the RAW Limelight readings
 *       imply since the bridge, rotated into the odo frame and re-anchored
 *       onto the odo pose at the bridge. A rigid transform cannot do better
 *       than this -- it is the classical calibration's live behavior.
 *   MLP pred: the same displacement, but each LL reading is first piped
 *       through the recorded RedMlp net (LL 6D -> odo xy), which can correct
 *       non-rigid / viewing-geometry bias the rigid transform cannot.
 *
 * Both diffs start at 0 at the bridge. Watch which EMA stays lower as you
 * drive: Diff SE3 >> Diff MLP means the net is genuinely adding accuracy;
 * roughly equal means a rigid transform is enough for this data.
 *
 * RED-side calibration: run TurnRed first. The net is a LOCAL model --
 * trust it near the region the calibration data covered.
 */
@TeleOp(name = "LL Verification", group = "Tuning")
public class LLVerification extends OpMode {

    private static final double IN_TO_M = 0.0254;
    private static final double M_TO_IN = 39.3700787402;

    /** Planar yaw of the recorded odo->field SE(3) solve (came out ~identity). */
    public static double tYaw = 0.0;
    /** EMA smoothing factor for the live diff lines. */
    public static double emaAlpha = 0.1;
    /** Verdict threshold: EMA below this = GOOD. */
    public static double goodInches = 3.0;

    private Drivetrain drivetrain;
    private Limelight limelight;

    // ---- bridge: one captured (odo, LL, net) triple ----
    private boolean bridged = false;
    private double odo0X, odo0Y, odo0Yaw;   // meters/radians, session odo frame
    private double ll0X, ll0Y, ll0Yaw;      // meters/radians, field frame
    private double net0X, net0Y;            // net's odo-frame xy at the bridge

    private long lastPipelineFixMs = 0;

    private double emaSe3In = Double.NaN;
    private double emaMlpIn = Double.NaN;

    private boolean xWasDown = false;

    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap, true);
        limelight = new Limelight(hardwareMap);

        // Explicitly register our statics with the dashboard (the automatic
        // DEX scan misses them in large apps).
        FtcDashboard.getInstance().withConfigRoot(root ->
                root.putVariable("LLVerification",
                        ReflectionConfig.createVariableFromClass(LLVerification.class)));

        telemetry.addLine("Run TurnRed first (RED field map).");
        telemetry.addLine("Drive normally; the first tags the Limelight sees");
        telemetry.addLine("become the reference. X = re-bridge (park first).");
    }

    @Override
    public void loop() {
        // ---- drive ----
        drivetrain.teleDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        drivetrain.period();
        limelight.updateRobotHeading(Math.toDegrees(drivetrain.pose().heading()));

        // ---- guard: re-assert the pipeline if something switched it ----
        int activePipe = limelight.getActivePipelineIndex();
        if (activePipe != Limelight.pipelineIndex) {
            long nowMs = System.currentTimeMillis();
            if (nowMs - lastPipelineFixMs > 1000) {
                lastPipelineFixMs = nowMs;
                limelight.switchPipeline(Limelight.pipelineIndex);
            }
        }

        // ---- current odo pose (meters); yaw below is always the encoder one ----
        double xm = drivetrain.pose().x() * IN_TO_M;
        double ym = drivetrain.pose().y() * IN_TO_M;
        double yaw = drivetrain.pose().heading();

        // ---- actual Limelight field pose ----
        Pose3D fieldPose = limelight.getPoseField3D();
        List<Integer> tagIds = limelight.getFiducialIds();

        // ---- bridge on first valid reading; X re-bridges ----
        if (!bridged && fieldPose != null) {
            bridge(fieldPose);
        }
        if (gamepad1.x && !xWasDown && fieldPose != null) {
            bridge(fieldPose);
        }
        xWasDown = gamepad1.x;

        telemetry.addData("Bridge", bridged
                ? String.format("SET at odo (%.1f, %.1f in) / LL (%.2f, %.2f m) -- X to redo",
                        odo0X / IN_TO_M, odo0Y / IN_TO_M, ll0X, ll0Y)
                : "waiting for tags (automatic)");
        telemetry.addData("Odo (in)", "(%.1f, %.1f, %.1f deg)",
                xm / IN_TO_M, ym / IN_TO_M, Math.toDegrees(yaw));
        telemetry.addData("LL tags", "%d ids %s", tagIds.size(), tagIds.toString());
        if (activePipe != Limelight.pipelineIndex) {
            telemetry.addData("LL pipeline", "active %d (want %d) <-- MISMATCH",
                    activePipe, Limelight.pipelineIndex);
        }

        if (fieldPose != null && bridged) {
            double llX = fieldPose.getPosition().x;
            double llY = fieldPose.getPosition().y;
            double llZ = fieldPose.getPosition().z;
            double llRoll = fieldPose.getOrientation().getRoll(AngleUnit.RADIANS);
            double llPitch = fieldPose.getOrientation().getPitch(AngleUnit.RADIANS);
            double llYaw = wrapPi(fieldPose.getOrientation().getYaw(AngleUnit.RADIANS));
            telemetry.addData("LL field (m)", "(%.3f, %.3f, %.3f) rpy (%.1f, %.1f, %.1f deg)",
                    llX, llY, llZ, Math.toDegrees(llRoll), Math.toDegrees(llPitch),
                    Math.toDegrees(llYaw));

            // ---- net's odo-frame xy for this reading ----
            double[] net = RedMlp.predict(llX, llY, llZ, llRoll, llPitch, llYaw);

            // ---- SE3 pred: displacement implied by the RAW LL readings ----
            double cS = Math.cos(odo0Yaw - ll0Yaw), sS = Math.sin(odo0Yaw - ll0Yaw);
            double dxL = llX - ll0X, dyL = llY - ll0Y;
            double se3X = odo0X + cS * dxL - sS * dyL;
            double se3Y = odo0Y + sS * dxL + cS * dyL;

            // ---- MLP pred: displacement implied by the net-corrected readings ----
            double cM = Math.cos(odo0Yaw - ll0Yaw + tYaw), sM = Math.sin(odo0Yaw - ll0Yaw + tYaw);
            double dxN = net[0] - net0X, dyN = net[1] - net0Y;
            double mlpX = odo0X + cM * dxN - sM * dyN;
            double mlpY = odo0Y + sM * dxN + cM * dyN;

            telemetry.addData("SE3 pred (in)", "(%.1f, %.1f)", se3X / IN_TO_M, se3Y / IN_TO_M);
            telemetry.addData("MLP pred (in)", "(%.1f, %.1f)", mlpX / IN_TO_M, mlpY / IN_TO_M);

            // ---- xy-only diffs vs the actual odometry ----
            double dSe3 = Math.hypot(se3X - xm, se3Y - ym) * M_TO_IN;
            double dMlp = Math.hypot(mlpX - xm, mlpY - ym) * M_TO_IN;
            emaSe3In = ema(emaSe3In, dSe3);
            emaMlpIn = ema(emaMlpIn, dMlp);
            telemetry.addData("Diff SE3", "%.2f in   EMA %.2f   %s",
                    dSe3, emaSe3In, verdict(emaSe3In));
            telemetry.addData("Diff MLP", "%.2f in   EMA %.2f   %s",
                    dMlp, emaMlpIn, verdict(emaMlpIn));
        } else if (fieldPose == null) {
            telemetry.addData("LL field", "no MT2 pose (need tags in view)");
        }

        telemetry.addLine("Yaw is always the encoder's; RED map; net valid near its training region only.");
    }

    /** Capture the current (odo, LL, net) triple as the session bridge. */
    private void bridge(Pose3D fieldPose) {
        odo0X = drivetrain.pose().x() * IN_TO_M;
        odo0Y = drivetrain.pose().y() * IN_TO_M;
        odo0Yaw = drivetrain.pose().heading();
        ll0X = fieldPose.getPosition().x;
        ll0Y = fieldPose.getPosition().y;
        ll0Yaw = wrapPi(fieldPose.getOrientation().getYaw(AngleUnit.RADIANS));
        double[] net = RedMlp.predict(ll0X, ll0Y, fieldPose.getPosition().z,
                fieldPose.getOrientation().getRoll(AngleUnit.RADIANS),
                fieldPose.getOrientation().getPitch(AngleUnit.RADIANS), ll0Yaw);
        net0X = net[0];
        net0Y = net[1];
        bridged = true;
        emaSe3In = emaMlpIn = Double.NaN;
    }

    private String verdict(double emaIn) {
        return emaIn < goodInches ? "GOOD" : "HIGH";
    }

    private double ema(double old, double v) {
        return Double.isNaN(old) ? v : (1.0 - emaAlpha) * old + emaAlpha * v;
    }

    private static double wrapPi(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }
}
