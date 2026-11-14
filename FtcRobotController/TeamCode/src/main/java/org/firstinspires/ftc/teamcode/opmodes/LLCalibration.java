package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Limelight tag-space calibration tool.
 *
 * Drives around like a normal teleop (left stick = translate, right stick x = turn).
 * The Pinpoint odometry provides (x, y, yaw); z = 0 and roll = pitch = 0 are assumed.
 *
 * Controls:
 *   X = capture a sample: averages the last ~avgWindow Limelight
 *       getRobotPoseTargetSpace() readings (dropping single-frame glitch
 *       outliers via median/MAD rejection) together with the matching
 *       odometry 6D pose. Park the robot before pressing X.
 *   Y = solve the odometry->tag transform matrix from all captured samples
 *       (least-squares style mean), and report the RMS residual.
 *   A = clear all samples.
 *
 * All math is done in meters/radians (odometry inches are converted), so the
 * resulting transform is a rigid SE(3) transform. Rotation convention is
 * R = Rz(yaw) * Ry(pitch) * Rx(roll).
 */
@TeleOp(name = "LL Calibration", group = "Tuning")
public class LLCalibration extends OpMode {

    private static final double IN_TO_M = 0.0254;
    private static final double M_TO_IN = 39.3700787402;

    private Drivetrain drivetrain;
    private Limelight limelight;

    /** One captured correspondence: odometry pose <-> Limelight tag-space pose. */
    private static class Sample {
        final int tagId;
        final double[] odoT;    // {x, y, z} meters (z = 0)
        final double[][] odoR;  // rotation (yaw only)
        final double[] llT;     // {x, y, z} meters, tag space
        final double[][] llR;   // rotation, tag space

        Sample(int tagId, double[] odoT, double[][] odoR, double[] llT, double[][] llR) {
            this.tagId = tagId;
            this.odoT = odoT;
            this.odoR = odoR;
            this.llT = llT;
            this.llR = llR;
        }
    }

    private final List<Sample> samples = new ArrayList<>();

    // ---- robust capture settings ----
    /** Rolling history length (readings with a visible tag). */
    public static int historySize = 40;
    /** Number of recent readings averaged per capture. */
    public static int avgWindow = 20;
    /** Outlier rejection: drop readings more than this many MADs from the median. */
    public static double outlierSensitivity = 3.0;
    /** Only readings fresher than this (ms) are used for a capture. */
    public static double stalenessMs = 1000.0;
    /** Minimum samples before the live RMS / solve is shown and trusted. */
    public static int minSolveSamples = 3;

    private static final double TRANS_FLOOR = 0.01;                // m, minimum rejection tolerance
    private static final double ANGLE_FLOOR = Math.toRadians(1.0); // rad, minimum rejection tolerance

    /** One live tag reading (raw material for the robust capture). */
    private static class Reading {
        final int tagId;
        final double[] ll;   // {x, y, z, roll, pitch, yaw}, meters/radians
        final double odoX, odoY, odoYaw;  // meters/radians
        final long timestampNs;

        Reading(int tagId, double[] ll, double odoX, double odoY, double odoYaw, long timestampNs) {
            this.tagId = tagId;
            this.ll = ll;
            this.odoX = odoX;
            this.odoY = odoY;
            this.odoYaw = odoYaw;
            this.timestampNs = timestampNs;
        }
    }

    private final ArrayDeque<Reading> history = new ArrayDeque<>();

    private boolean xWasDown = false;
    private boolean yWasDown = false;
    private boolean aWasDown = false;

    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap, true);
        limelight = new Limelight(hardwareMap);
        telemetry.addLine("Drive the robot around the tag.");
        telemetry.addLine("Park, then X = capture (robust avg), Y = solve, A = clear.");
    }

    @Override
    public void loop() {
        // ---- drive ----
        drivetrain.teleDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        drivetrain.period();
        limelight.updateRobotHeading(Math.toDegrees(drivetrain.pose().heading()));

        // ---- live state ----
        double odoXIn = drivetrain.pose().x();
        double odoYIn = drivetrain.pose().y();
        double odoYaw = drivetrain.pose().heading();

        LLResultTypes.FiducialResult fiducial = limelight.getPrimaryFiducial();
        Pose3D targetSpace = fiducial == null ? null : fiducial.getRobotPoseTargetSpace();

        telemetry.addData("Odo (in)", "(%.2f, %.2f, %.1f deg)", odoXIn, odoYIn, Math.toDegrees(odoYaw));
        if (targetSpace != null) {
            telemetry.addData("Tag id", fiducial.getFiducialId());
            telemetry.addData("LL target-space (m)", "(%.3f, %.3f, %.3f)",
                    targetSpace.getPosition().x, targetSpace.getPosition().y, targetSpace.getPosition().z);
            telemetry.addData("LL target-space rpy (deg)", "(%.1f, %.1f, %.1f)",
                    targetSpace.getOrientation().getRoll(AngleUnit.DEGREES),
                    targetSpace.getOrientation().getPitch(AngleUnit.DEGREES),
                    targetSpace.getOrientation().getYaw(AngleUnit.DEGREES));
        } else {
            telemetry.addData("Tag", "none visible");
        }
        telemetry.addData("Samples", samples.size());

        // ---- rolling history for the robust capture ----
        if (targetSpace != null) {
            history.addLast(new Reading(
                    fiducial.getFiducialId(),
                    new double[]{
                            targetSpace.getPosition().x, targetSpace.getPosition().y, targetSpace.getPosition().z,
                            targetSpace.getOrientation().getRoll(AngleUnit.RADIANS),
                            targetSpace.getOrientation().getPitch(AngleUnit.RADIANS),
                            targetSpace.getOrientation().getYaw(AngleUnit.RADIANS)},
                    odoXIn * IN_TO_M, odoYIn * IN_TO_M, odoYaw,
                    System.nanoTime()));
            while (history.size() > historySize) {
                history.removeFirst();
            }
        }

        // ---- X: capture (robust average of recent readings) ----
        if (gamepad1.x && !xWasDown) {
            captureRobust();
        }
        xWasDown = gamepad1.x;

        // ---- A: clear ----
        if (gamepad1.a && !aWasDown) {
            samples.clear();
            telemetry.addLine(">> samples cleared");
        }
        aWasDown = gamepad1.a;

        // ---- Y: full report ----
        if (gamepad1.y && !yWasDown) {
            reportSolve();
        }
        yWasDown = gamepad1.y;

        // ---- live RMS (auto-solves every loop once enough samples exist) ----
        if (samples.size() >= minSolveSamples) {
            SolveResult r = solve();
            if (r != null) {
                telemetry.addLine(String.format(
                        "Live tag %d | %d samples | RMS %.2f in / %.2f deg | T trans (%.2f, %.2f, %.2f) in",
                        r.tagId, r.n, r.rmsTrans * M_TO_IN, r.rmsRot,
                        r.meanT[0] * M_TO_IN, r.meanT[1] * M_TO_IN, r.meanT[2] * M_TO_IN));
            }
        }

        telemetry.update();
    }

    // ------------------------------------------------------------------
    // Robust capture: average recent readings, reject single-frame glitches
    // ------------------------------------------------------------------

    /**
     * Capture a sample from the recent tag readings instead of one frame.
     * Per component, readings further than outlierSensitivity * MAD (median
     * absolute deviation) from the median are rejected as glitches, then the
     * surviving values are averaged. Only readings of the dominant tag and
     * fresher than stalenessMs are considered. Park the robot before pressing X.
     */
    private void captureRobust() {
        long now = System.nanoTime();

        List<Reading> fresh = new ArrayList<>();
        for (Reading r : history) {
            if ((now - r.timestampNs) / 1_000_000.0 <= stalenessMs) {
                fresh.add(r);
            }
        }
        if (fresh.isEmpty()) {
            telemetry.addLine(">> no recent tag readings, sample NOT captured");
            return;
        }

        int start = Math.max(0, fresh.size() - avgWindow);
        List<Reading> window = new ArrayList<>(fresh.subList(start, fresh.size()));

        // keep only the dominant tag's readings (single-frame tag flips drop out)
        int tagId = window.get(window.size() - 1).tagId;
        int bestCount = 0;
        for (Reading r : window) {
            int c = 0;
            for (Reading t : window) {
                if (t.tagId == r.tagId) c++;
            }
            if (c > bestCount) {
                bestCount = c;
                tagId = r.tagId;
            }
        }
        List<Reading> tagged = new ArrayList<>();
        for (Reading r : window) {
            if (r.tagId == tagId) tagged.add(r);
        }
        int m = tagged.size();

        // robust-average every component (indices 3-5 of ll are angles)
        int minKept = m;
        double[] ll = new double[6];
        for (int c = 0; c < 6; c++) {
            List<Double> vals = new ArrayList<>(m);
            for (Reading r : tagged) vals.add(r.ll[c]);
            int[] kept = new int[1];
            ll[c] = robustMean(vals, c >= 3, kept);
            minKept = Math.min(minKept, kept[0]);
        }
        List<Double> oxv = new ArrayList<>(m), oyv = new ArrayList<>(m), oyawv = new ArrayList<>(m);
        for (Reading r : tagged) {
            oxv.add(r.odoX);
            oyv.add(r.odoY);
            oyawv.add(r.odoYaw);
        }
        int[] kept = new int[1];
        double ox = robustMean(oxv, false, kept); minKept = Math.min(minKept, kept[0]);
        double oy = robustMean(oyv, false, kept); minKept = Math.min(minKept, kept[0]);
        double oyaw = robustMean(oyawv, true, kept); minKept = Math.min(minKept, kept[0]);

        samples.add(new Sample(tagId,
                new double[]{ox, oy, 0.0}, rotFromRpy(0.0, 0.0, oyaw),
                new double[]{ll[0], ll[1], ll[2]}, rotFromRpy(ll[3], ll[4], ll[5])));
        telemetry.addLine(String.format(
                ">> captured sample %d (tag %d): %d/%d good readings, up to %d glitch frames dropped",
                samples.size(), tagId, minKept, m, m - minKept));
    }

    /** Median-and-MAD outlier-rejecting mean. wrap = treat values as angles. */
    private static double robustMean(List<Double> vals, boolean wrap, int[] keptOut) {
        double med = median(vals);
        List<Double> dev = new ArrayList<>(vals.size());
        for (double v : vals) {
            double d = v - med;
            if (wrap) d = wrapPi(d);
            dev.add(d);
        }
        List<Double> absDev = new ArrayList<>(vals.size());
        for (double d : dev) absDev.add(Math.abs(d));
        double mad = median(absDev);
        double tol = Math.max(outlierSensitivity * mad, wrap ? ANGLE_FLOOR : TRANS_FLOOR);
        double sum = 0;
        int n = 0;
        for (double d : dev) {
            if (Math.abs(d) <= tol) {
                sum += d;
                n++;
            }
        }
        keptOut[0] = n;
        return med + sum / n;
    }

    private static double median(List<Double> vals) {
        List<Double> sorted = new ArrayList<>(vals);
        Collections.sort(sorted);
        int n = sorted.size();
        return n % 2 == 1 ? sorted.get(n / 2) : (sorted.get(n / 2 - 1) + sorted.get(n / 2)) / 2.0;
    }

    private static double wrapPi(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }

    // ------------------------------------------------------------------
    // Solver: find T such that  LL_i ~= T * Odo_i  for all samples of one tag
    // ------------------------------------------------------------------

    /** Result of a solve (translation in meters, rotation RMS in degrees). */
    private static class SolveResult {
        final int tagId;
        final int n;
        final double rmsTrans;   // meters
        final double rmsRot;     // degrees
        final double[] meanT;    // meters
        final double[][] meanR;

        SolveResult(int tagId, int n, double rmsTrans, double rmsRot, double[] meanT, double[][] meanR) {
            this.tagId = tagId;
            this.n = n;
            this.rmsTrans = rmsTrans;
            this.rmsRot = rmsRot;
            this.meanT = meanT;
            this.meanR = meanR;
        }
    }

    /** Returns null if there are no samples. */
    private SolveResult solve() {
        if (samples.isEmpty()) {
            return null;
        }

        // group by tag: solve for the tag with the most samples
        int bestTag = -1;
        int bestCount = 0;
        for (Sample s : samples) {
            int n = 0;
            for (Sample t : samples) {
                if (t.tagId == s.tagId) n++;
            }
            if (n > bestCount) {
                bestCount = n;
                bestTag = s.tagId;
            }
        }

        List<Sample> group = new ArrayList<>();
        for (Sample s : samples) {
            if (s.tagId == bestTag) group.add(s);
        }

        // T_i = LL_i * Odo_i^-1  for each sample
        List<double[][]> tRots = new ArrayList<>();
        List<double[]> tTrans = new ArrayList<>();
        for (Sample s : group) {
            double[][] invOdoR = transpose(s.odoR);
            double[] invOdoT = mulVec(invOdoR, negate(s.odoT));
            // T_i rotation = llR * odoR^T ; translation = llR * (-odoR^T * odoT) + llT
            double[][] tR = mul(s.llR, invOdoR);
            double[] tT = add(mulVec(s.llR, negate(invOdoT)), s.llT);
            tRots.add(tR);
            tTrans.add(tT);
        }

        // mean translation
        double[] meanT = {0, 0, 0};
        for (double[] t : tTrans) {
            meanT[0] += t[0]; meanT[1] += t[1]; meanT[2] += t[2];
        }
        meanT[0] /= tTrans.size(); meanT[1] /= tTrans.size(); meanT[2] /= tTrans.size();

        // mean rotation via quaternion averaging (sign-aligned to the first)
        double[] qSum = {0, 0, 0, 0};
        double[] qRef = quatFromRot(tRots.get(0));
        for (double[][] r : tRots) {
            double[] q = quatFromRot(r);
            if (q[0] * qRef[0] + q[1] * qRef[1] + q[2] * qRef[2] + q[3] * qRef[3] < 0) {
                q = new double[]{-q[0], -q[1], -q[2], -q[3]};
            }
            qSum[0] += q[0]; qSum[1] += q[1]; qSum[2] += q[2]; qSum[3] += q[3];
        }
        double norm = Math.sqrt(qSum[0] * qSum[0] + qSum[1] * qSum[1] + qSum[2] * qSum[2] + qSum[3] * qSum[3]);
        double[] qMean = {qSum[0] / norm, qSum[1] / norm, qSum[2] / norm, qSum[3] / norm};
        double[][] meanR = rotFromQuat(qMean);

        // residuals: predicted = T * Odo_i, compare to LL_i
        double sumSqTrans = 0;
        double sumSqRot = 0;
        for (Sample s : group) {
            double[][] pR = mul(meanR, s.odoR);
            double[] pT = add(mulVec(meanR, s.odoT), meanT);
            // dT = LL * pred^-1
            double[][] invPR = transpose(pR);
            double[] invPT = mulVec(invPR, negate(pT));
            double[][] dR = mul(s.llR, invPR);
            double[] dT = add(mulVec(s.llR, negate(invPT)), s.llT);
            double eT = Math.sqrt(dT[0] * dT[0] + dT[1] * dT[1] + dT[2] * dT[2]);
            double tr = dR[0][0] + dR[1][1] + dR[2][2];
            double eR = Math.toDegrees(Math.acos(clamp((tr - 1.0) / 2.0, -1.0, 1.0)));
            sumSqTrans += eT * eT;
            sumSqRot += eR * eR;
        }
        double rmsTrans = Math.sqrt(sumSqTrans / group.size());
        double rmsRot = Math.sqrt(sumSqRot / group.size());

        return new SolveResult(bestTag, group.size(), rmsTrans, rmsRot, meanT, meanR);
    }

    /** Y: full report with the 4x4 matrix. */
    private void reportSolve() {
        SolveResult r = solve();
        if (r == null) {
            telemetry.addLine(">> no samples captured yet");
            return;
        }
        if (r.n < minSolveSamples) {
            telemetry.addLine(String.format(">> only %d sample(s) for tag %d -- capture at least %d for a trustworthy fit",
                    r.n, r.tagId, minSolveSamples));
        }
        telemetry.addLine(String.format(">> SOLVE tag %d (%d samples)", r.tagId, r.n));
        telemetry.addLine(String.format(">> RMS trans: %.2f in (%.4f m)", r.rmsTrans * M_TO_IN, r.rmsTrans));
        telemetry.addLine(String.format(">> RMS rot:   %.2f deg", r.rmsRot));
        telemetry.addLine(">> T (tag <- odo), 4x4 row-major, meters:");
        telemetry.addLine(String.format("  [%.4f %.4f %.4f %.4f]", r.meanR[0][0], r.meanR[0][1], r.meanR[0][2], r.meanT[0]));
        telemetry.addLine(String.format("  [%.4f %.4f %.4f %.4f]", r.meanR[1][0], r.meanR[1][1], r.meanR[1][2], r.meanT[1]));
        telemetry.addLine(String.format("  [%.4f %.4f %.4f %.4f]", r.meanR[2][0], r.meanR[2][1], r.meanR[2][2], r.meanT[2]));
        telemetry.addLine("  [0 0 0 1]");
        double[] rpy = rotToRpy(r.meanR);
        telemetry.addLine(String.format(">> T translation (in): (%.2f, %.2f, %.2f)",
                r.meanT[0] * M_TO_IN, r.meanT[1] * M_TO_IN, r.meanT[2] * M_TO_IN));
        telemetry.addLine(String.format(">> T rpy (deg): (%.2f, %.2f, %.2f)",
                Math.toDegrees(rpy[0]), Math.toDegrees(rpy[1]), Math.toDegrees(rpy[2])));
    }

    // ------------------------------------------------------------------
    // Tiny SE(3) / rotation math. Rotation convention: R = Rz(yaw)*Ry(pitch)*Rx(roll).
    // ------------------------------------------------------------------

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static double[] negate(double[] v) {
        return new double[]{-v[0], -v[1], -v[2]};
    }

    private static double[] add(double[] a, double[] b) {
        return new double[]{a[0] + b[0], a[1] + b[1], a[2] + b[2]};
    }

    private static double[] mulVec(double[][] m, double[] v) {
        return new double[]{
                m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
                m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
                m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2]};
    }

    private static double[][] mul(double[][] a, double[][] b) {
        double[][] r = new double[3][3];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                r[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j];
            }
        }
        return r;
    }

    private static double[][] transpose(double[][] a) {
        double[][] r = new double[3][3];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                r[i][j] = a[j][i];
            }
        }
        return r;
    }

    /** R = Rz(yaw) * Ry(pitch) * Rx(roll). */
    private static double[][] rotFromRpy(double roll, double pitch, double yaw) {
        double cr = Math.cos(roll), sr = Math.sin(roll);
        double cp = Math.cos(pitch), sp = Math.sin(pitch);
        double cy = Math.cos(yaw), sy = Math.sin(yaw);
        return new double[][]{
                {cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr},
                {sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr},
                {-sp, cp * sr, cp * cr}};
    }

    /** Extract {roll, pitch, yaw} from R = Rz(yaw)*Ry(pitch)*Rx(roll). */
    private static double[] rotToRpy(double[][] r) {
        double pitch = -Math.asin(clamp(r[2][0], -1.0, 1.0));
        double roll = Math.atan2(r[2][1], r[2][2]);
        double yaw = Math.atan2(r[1][0], r[0][0]);
        return new double[]{roll, pitch, yaw};
    }

    /** Quaternion {w, x, y, z} from rotation matrix (Shepperd's method). */
    private static double[] quatFromRot(double[][] r) {
        double tr = r[0][0] + r[1][1] + r[2][2];
        double w, x, y, z;
        if (tr > 0) {
            double s = Math.sqrt(tr + 1.0) * 2.0;
            w = 0.25 * s;
            x = (r[2][1] - r[1][2]) / s;
            y = (r[0][2] - r[2][0]) / s;
            z = (r[1][0] - r[0][1]) / s;
        } else if (r[0][0] > r[1][1] && r[0][0] > r[2][2]) {
            double s = Math.sqrt(1.0 + r[0][0] - r[1][1] - r[2][2]) * 2.0;
            w = (r[2][1] - r[1][2]) / s;
            x = 0.25 * s;
            y = (r[0][1] + r[1][0]) / s;
            z = (r[0][2] + r[2][0]) / s;
        } else if (r[1][1] > r[2][2]) {
            double s = Math.sqrt(1.0 + r[1][1] - r[0][0] - r[2][2]) * 2.0;
            w = (r[0][2] - r[2][0]) / s;
            x = (r[0][1] + r[1][0]) / s;
            y = 0.25 * s;
            z = (r[1][2] + r[2][1]) / s;
        } else {
            double s = Math.sqrt(1.0 + r[2][2] - r[0][0] - r[1][1]) * 2.0;
            w = (r[1][0] - r[0][1]) / s;
            x = (r[0][2] + r[2][0]) / s;
            y = (r[1][2] + r[2][1]) / s;
            z = 0.25 * s;
        }
        return new double[]{w, x, y, z};
    }

    private static double[][] rotFromQuat(double[] q) {
        double w = q[0], x = q[1], y = q[2], z = q[3];
        return new double[][]{
                {1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)},
                {2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)},
                {2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)}};
    }
}
