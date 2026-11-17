package org.firstinspires.ftc.teamcode.opmodes;

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
 * Limelight field-space calibration tool.
 *
 * Drives around like a normal teleop (left stick = translate, right stick x = turn).
 * The Pinpoint odometry provides (x, y, yaw); z = 0 and roll = pitch = 0 are assumed.
 *
 * Uses the Limelight FIELD-space robot pose (MegaTag2 botpose, needs the uploaded
 * field map). Because every reading is in the same fixed field frame, samples
 * captured facing ANY tags all contribute to one solve.
 *
 * Controls:
 *   X = capture a sample: averages the last ~avgWindow Limelight field-space
 *       readings (dropping single-frame glitch outliers via median/MAD
 *       rejection) together with the matching odometry 6D pose. Park the
 *       robot before pressing X.
 *   Y = solve the odometry->field transform matrix from all captured samples
 *       (least-squares style mean), report the RMS residual and print the
 *       full 4x4 matrix. The report stays on screen until the next solve.
 *   A = clear all samples.
 *
 * The "Fit" line refreshes whenever samples change, and the "Live err" line
 * compares the current Limelight reading against the solved transform every
 * loop, so you can see the calibration quality update live while driving.
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

    /** One captured correspondence: odometry pose <-> Limelight field-space pose. */
    private static class Sample {
        final double[] odoT;    // {x, y, z} meters (z = 0)
        final double[][] odoR;  // rotation (yaw only)
        final double[] llT;     // {x, y, z} meters, field frame
        final double[][] llR;   // rotation, field frame
        final double travelIn;  // odo straight-line distance from first sample (inches)
        final double dHeadDeg;  // odo heading change from first sample (degrees)
        final String tags;      // tag ids visible at capture

        Sample(double[] odoT, double[][] odoR, double[] llT, double[][] llR,
               double travelIn, double dHeadDeg, String tags) {
            this.odoT = odoT;
            this.odoR = odoR;
            this.llT = llT;
            this.llR = llR;
            this.travelIn = travelIn;
            this.dHeadDeg = dHeadDeg;
            this.tags = tags;
        }
    }

    private final List<Sample> samples = new ArrayList<>();
    private double refOdoX, refOdoY, refOdoYaw;  // first capture of the session (meters/rad)

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
    /** Alliance for the field map uploaded at init (blue: tags 38-45, red: 30-37). */
    public static boolean blueAlliance = false;

    private static final double TRANS_FLOOR = 0.01;                // m, minimum rejection tolerance
    private static final double ANGLE_FLOOR = Math.toRadians(1.0); // rad, minimum rejection tolerance

    /** One live field-space reading (raw material for the robust capture). */
    private static class Reading {
        final double[] ll;   // {x, y, z, roll, pitch, yaw}, field frame, meters/radians
        final double odoX, odoY, odoYaw;  // meters/radians
        final long timestampNs;

        Reading(double[] ll, double odoX, double odoY, double odoYaw, long timestampNs) {
            this.ll = ll;
            this.odoX = odoX;
            this.odoY = odoY;
            this.odoYaw = odoYaw;
            this.timestampNs = timestampNs;
        }
    }

    private final ArrayDeque<Reading> history = new ArrayDeque<>();
    private long lastPipelineFixMs = 0;

    /** Last one-shot status (capture/clear result), rendered every loop so it doesn't flash. */
    private String statusLine = null;
    /** Lines of the last Y-button solve report, rendered every loop so they stay on screen. */
    private final List<String> reportLines = new ArrayList<>();
    /** Most recent solve, used for the live validation error. */
    private SolveResult lastSolve = null;

    private boolean xWasDown = false;
    private boolean yWasDown = false;
    private boolean aWasDown = false;
    private boolean bWasDown = false;
    private boolean dpadUpWasDown = false;

    private boolean mapUploadedHere = false;
    private String mapChoice = "kept";

    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap, true);
        limelight = new Limelight(hardwareMap);
        // the field map is deliberately NOT uploaded here: TurnRed/TurnBlue own it
        // and it stays on the Limelight. Press B only if you need to switch it
        // while calibrating.
        telemetry.addData("Field map", "kept (set by TurnRed/TurnBlue); B = switch to %s and upload",
                blueAlliance ? "BLUE" : "RED");
        telemetry.addLine("Drive the robot around the tags.");
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

        Pose3D fieldPose = limelight.getPoseField3D();

        telemetry.addData("Odo (in)", "(%.2f, %.2f, %.1f deg)", odoXIn, odoYIn, Math.toDegrees(odoYaw));
        List<Integer> tagIds = limelight.getFiducialIds();
        telemetry.addData("LL tags", "%d ids %s", tagIds.size(), tagIds.toString());

        // guard: re-assert the pipeline if something (e.g. an open web UI) switched it
        int activePipe = limelight.getActivePipelineIndex();
        if (activePipe != Limelight.pipelineIndex) {
            long nowMs = System.currentTimeMillis();
            if (nowMs - lastPipelineFixMs > 1000) {
                lastPipelineFixMs = nowMs;
                limelight.switchPipeline(Limelight.pipelineIndex);
            }
        }
        telemetry.addData("LL pipeline", "active %d (want %d)%s",
                activePipe, Limelight.pipelineIndex,
                activePipe == Limelight.pipelineIndex ? "" : "  <-- MISMATCH");
        telemetry.addData("LL map / orient POST", "%s [%s] / %s",
                mapUploadedHere ? (limelight.isFieldMapUploadOk() ? "ok" : "FAILED") : "kept", mapChoice,
                limelight.isLastOrientationPostOk() ? "ok" : "FAILING");
        Pose3D mt1 = limelight.getBotposeMt1Raw();
        telemetry.addData("LL MT1 raw (m)", mt1 == null ? "none" : String.format("(%.3f, %.3f, %.3f)",
                mt1.getPosition().x, mt1.getPosition().y, mt1.getPosition().z));
        if (fieldPose != null) {
            telemetry.addData("LL field (m)", "(%.3f, %.3f, %.3f)",
                    fieldPose.getPosition().x, fieldPose.getPosition().y, fieldPose.getPosition().z);
            telemetry.addData("LL field rpy (deg)", "(%.1f, %.1f, %.1f)",
                    fieldPose.getOrientation().getRoll(AngleUnit.DEGREES),
                    fieldPose.getOrientation().getPitch(AngleUnit.DEGREES),
                    fieldPose.getOrientation().getYaw(AngleUnit.DEGREES));
        } else {
            telemetry.addData("LL field", "no MT2 pose (enable MegaTag2 in web UI)");
        }
        telemetry.addData("Samples", samples.size());

        // ---- rolling history for the robust capture ----
        if (fieldPose != null) {
            history.addLast(new Reading(
                    new double[]{
                            fieldPose.getPosition().x, fieldPose.getPosition().y, fieldPose.getPosition().z,
                            fieldPose.getOrientation().getRoll(AngleUnit.RADIANS),
                            fieldPose.getOrientation().getPitch(AngleUnit.RADIANS),
                            fieldPose.getOrientation().getYaw(AngleUnit.RADIANS)},
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
            lastSolve = null;
            reportLines.clear();
            statusLine = ">> samples cleared";
        }
        aWasDown = gamepad1.a;

        // ---- B: toggle red/blue field map and re-upload ----
        if (gamepad1.b && !bWasDown) {
            blueAlliance = !blueAlliance;
            boolean ok = limelight.uploadHiveFieldMap(blueAlliance);
            mapUploadedHere = true;
            mapChoice = (ok ? "ok" : "FAILED") + " (" + (blueAlliance ? "blue" : "red") + ")";
            statusLine = ">> field map switched to " + (blueAlliance ? "BLUE" : "RED")
                    + ", upload " + (ok ? "ok" : "FAILED") + " (recapture if you had samples)";
        }
        bWasDown = gamepad1.b;

        // ---- dpad_up: export samples to CSV for offline analysis ----
        if (gamepad1.dpad_up && !dpadUpWasDown) {
            int exported = exportSamplesCsv();
            statusLine = exported >= 0
                    ? ">> exported " + exported + " samples to /sdcard/FIRST/llcal_samples.csv (adb pull)"
                    : ">> CSV export failed (check /sdcard/FIRST exists)";
        }
        dpadUpWasDown = gamepad1.dpad_up;

        // ---- Y: full report ----
        if (gamepad1.y && !yWasDown) {
            reportSolve();
        }
        yWasDown = gamepad1.y;

        // ---- live fit + live validation (recomputed every loop) ----
        if (samples.size() >= minSolveSamples) {
            lastSolve = solve();
        }
        if (lastSolve != null) {
            telemetry.addLine(String.format(
                    "Fit | %d samples | RMS %.2f in / %.2f deg | T trans (%.2f, %.2f, %.2f) in",
                    lastSolve.n, lastSolve.rmsTrans * M_TO_IN, lastSolve.rmsRot,
                    lastSolve.meanT[0] * M_TO_IN, lastSolve.meanT[1] * M_TO_IN, lastSolve.meanT[2] * M_TO_IN));
            if (fieldPose != null) {
                // live check: does the solved T predict the field-space pose of the CURRENT frame?
                double[][] nowR = rotFromRpy(0.0, 0.0, odoYaw);
                double[] nowT = {odoXIn * IN_TO_M, odoYIn * IN_TO_M, 0.0};
                double[][] pR = mul(lastSolve.meanR, nowR);
                double[] pT = add(mulVec(lastSolve.meanR, nowT), lastSolve.meanT);
                double[][] llR = rotFromRpy(
                        fieldPose.getOrientation().getRoll(AngleUnit.RADIANS),
                        fieldPose.getOrientation().getPitch(AngleUnit.RADIANS),
                        fieldPose.getOrientation().getYaw(AngleUnit.RADIANS));
                double[] llT = {fieldPose.getPosition().x, fieldPose.getPosition().y, fieldPose.getPosition().z};
                double[] err = residual(llR, llT, pR, pT);
                telemetry.addLine(String.format("Live err vs fit: %.2f in / %.2f deg",
                        err[0] * M_TO_IN, err[1]));
            }
        }

        // persistent status + report so they don't flash for a single frame
        if (statusLine != null) {
            telemetry.addLine(statusLine);
        }
        for (String line : reportLines) {
            telemetry.addLine(line);
        }

        telemetry.update();
    }

    // ------------------------------------------------------------------
    // Robust capture: average recent readings, reject single-frame glitches
    // ------------------------------------------------------------------

    /**
     * Capture a sample from the recent field-space readings instead of one frame.
     * Per component, readings further than outlierSensitivity * MAD (median
     * absolute deviation) from the median are rejected as glitches, then the
     * surviving values are averaged. Only readings fresher than stalenessMs
     * are considered. Park the robot before pressing X.
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
            statusLine = ">> no recent field readings, sample NOT captured";
            return;
        }

        int start = Math.max(0, fresh.size() - avgWindow);
        List<Reading> window = new ArrayList<>(fresh.subList(start, fresh.size()));
        int m = window.size();

        // robust-average every component (indices 3-5 of ll are angles)
        int minKept = m;
        double[] ll = new double[6];
        for (int c = 0; c < 6; c++) {
            List<Double> vals = new ArrayList<>(m);
            for (Reading r : window) vals.add(r.ll[c]);
            int[] kept = new int[1];
            ll[c] = robustMean(vals, c >= 3, kept);
            minKept = Math.min(minKept, kept[0]);
        }
        List<Double> oxv = new ArrayList<>(m), oyv = new ArrayList<>(m), oyawv = new ArrayList<>(m);
        for (Reading r : window) {
            oxv.add(r.odoX);
            oyv.add(r.odoY);
            oyawv.add(r.odoYaw);
        }
        int[] kept = new int[1];
        double ox = robustMean(oxv, false, kept); minKept = Math.min(minKept, kept[0]);
        double oy = robustMean(oyv, false, kept); minKept = Math.min(minKept, kept[0]);
        double oyaw = robustMean(oyawv, true, kept); minKept = Math.min(minKept, kept[0]);

        // diagnostics: travel / heading change since the first capture of this session
        if (samples.isEmpty()) {
            refOdoX = ox; refOdoY = oy; refOdoYaw = oyaw;
        }
        double travelIn = Math.hypot(ox - refOdoX, oy - refOdoY) * M_TO_IN;
        double dHeadDeg = Math.toDegrees(wrapPi(oyaw - refOdoYaw));

        samples.add(new Sample(
                new double[]{ox, oy, 0.0}, rotFromRpy(0.0, 0.0, oyaw),
                new double[]{ll[0], ll[1], ll[2]}, rotFromRpy(ll[3], ll[4], ll[5]),
                travelIn, dHeadDeg, limelight.getFiducialIds().toString()));
        statusLine = String.format(
                ">> captured sample %d: %d/%d good readings, up to %d glitch frames dropped",
                samples.size(), minKept, m, m - minKept);
    }

    /** Writes all samples as CSV (meters/radians) for offline analysis:
     *  adb pull /sdcard/FIRST/llcal_samples.csv */
    private int exportSamplesCsv() {
        try (java.io.PrintWriter w = new java.io.PrintWriter(
                new java.io.FileWriter("/sdcard/FIRST/llcal_samples.csv"))) {
            w.println("odo_x,odo_y,odo_yaw,ll_x,ll_y,ll_z,ll_roll,ll_pitch,ll_yaw");
            for (Sample s : samples) {
                double oYaw = Math.atan2(s.odoR[1][0], s.odoR[0][0]);
                double lYaw = Math.atan2(s.llR[1][0], s.llR[0][0]);
                double lPitch = -Math.asin(Math.max(-1.0, Math.min(1.0, s.llR[2][0])));
                double lRoll = Math.atan2(s.llR[2][1], s.llR[2][2]);
                w.println(String.format(java.util.Locale.US,
                        "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
                        s.odoT[0], s.odoT[1], oYaw,
                        s.llT[0], s.llT[1], s.llT[2], lRoll, lPitch, lYaw));
            }
            return samples.size();
        } catch (Exception e) {
            return -1;
        }
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

    /** Pearson correlation; 0 if either side has no variance. */
    private static double pearson(double[] a, double[] b) {
        int n = a.length;
        double ma = 0, mb = 0;
        for (int i = 0; i < n; i++) { ma += a[i]; mb += b[i]; }
        ma /= n; mb /= n;
        double cov = 0, va = 0, vb = 0;
        for (int i = 0; i < n; i++) {
            double da = a[i] - ma, db = b[i] - mb;
            cov += da * db; va += da * da; vb += db * db;
        }
        if (va < 1e-12 || vb < 1e-12) return 0;
        return cov / Math.sqrt(va * vb);
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
        final int n;
        final double rmsTrans;   // meters
        final double rmsRot;     // degrees
        final double[] meanT;    // meters
        final double[][] meanR;
        final double[] sampleErrT;  // meters, per sample in the solved group
        final double[] sampleErrR;  // degrees, per sample in the solved group

        SolveResult(int n, double rmsTrans, double rmsRot, double[] meanT, double[][] meanR,
                    double[] sampleErrT, double[] sampleErrR) {
            this.n = n;
            this.rmsTrans = rmsTrans;
            this.rmsRot = rmsRot;
            this.meanT = meanT;
            this.meanR = meanR;
            this.sampleErrT = sampleErrT;
            this.sampleErrR = sampleErrR;
        }
    }

    /** Returns null if there are no samples. */
    private SolveResult solve() {
        if (samples.isEmpty()) {
            return null;
        }

        // all samples share the field frame -- solve them together
        List<Sample> group = new ArrayList<>(samples);

        // T_i = LL_i * Odo_i^-1  for each sample
        List<double[][]> tRots = new ArrayList<>();
        List<double[]> tTrans = new ArrayList<>();
        for (Sample s : group) {
            double[][] invOdoR = transpose(s.odoR);
            double[] invOdoT = mulVec(invOdoR, negate(s.odoT));  // = Odo^-1 translation
            // T_i rotation = llR * odoR^T ; translation = llR * (Odo^-1 translation) + llT
            double[][] tR = mul(s.llR, invOdoR);
            double[] tT = add(mulVec(s.llR, invOdoT), s.llT);
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

        // residuals: predicted = T * Odo_i, compare to LL_i (kept per sample for the report)
        double sumSqTrans = 0;
        double sumSqRot = 0;
        double[] sampleErrT = new double[group.size()];
        double[] sampleErrR = new double[group.size()];
        for (int i = 0; i < group.size(); i++) {
            Sample s = group.get(i);
            double[][] pR = mul(meanR, s.odoR);
            double[] pT = add(mulVec(meanR, s.odoT), meanT);
            double[] err = residual(s.llR, s.llT, pR, pT);
            sampleErrT[i] = err[0];
            sampleErrR[i] = err[1];
            sumSqTrans += err[0] * err[0];
            sumSqRot += err[1] * err[1];
        }
        double rmsTrans = Math.sqrt(sumSqTrans / group.size());
        double rmsRot = Math.sqrt(sumSqRot / group.size());

        return new SolveResult(group.size(), rmsTrans, rmsRot, meanT, meanR, sampleErrT, sampleErrR);
    }

    /**
     * Residual between an actual LL pose and the pose predicted by the transform:
     * d = LL * pred^-1, so identity d (0 trans, 0 rot) means a perfect match.
     * Returns {translation error (m), rotation error (deg)}.
     */
    private static double[] residual(double[][] llR, double[] llT, double[][] pR, double[] pT) {
        double[][] invPR = transpose(pR);
        double[] invPT = mulVec(invPR, negate(pT));  // = pred^-1 translation
        double[][] dR = mul(llR, invPR);
        double[] dT = add(mulVec(llR, invPT), llT);
        double eT = Math.sqrt(dT[0] * dT[0] + dT[1] * dT[1] + dT[2] * dT[2]);
        double tr = dR[0][0] + dR[1][1] + dR[2][2];
        double eR = Math.toDegrees(Math.acos(clamp((tr - 1.0) / 2.0, -1.0, 1.0)));
        return new double[]{eT, eR};
    }

    /**
     * Y: full report with the 4x4 matrix. Built into reportLines so it stays on
     * screen every loop instead of flashing for a single frame.
     */
    private void reportSolve() {
        SolveResult r = solve();
        reportLines.clear();
        if (r == null) {
            statusLine = ">> no samples captured yet";
            return;
        }
        lastSolve = r;
        if (r.n < minSolveSamples) {
            reportLines.add(String.format(">> only %d sample(s) -- capture at least %d for a trustworthy fit",
                    r.n, minSolveSamples));
        }
        reportLines.add(String.format(">> SOLVE (%d samples, field frame)", r.n));
        reportLines.add(String.format(">> RMS trans: %.2f in (%.4f m)", r.rmsTrans * M_TO_IN, r.rmsTrans));
        reportLines.add(String.format(">> RMS rot:   %.2f deg", r.rmsRot));
        reportLines.add(">> T (field <- odo), 4x4 row-major, meters:");
        reportLines.add(String.format("  [%.4f %.4f %.4f %.4f]", r.meanR[0][0], r.meanR[0][1], r.meanR[0][2], r.meanT[0]));
        reportLines.add(String.format("  [%.4f %.4f %.4f %.4f]", r.meanR[1][0], r.meanR[1][1], r.meanR[1][2], r.meanT[1]));
        reportLines.add(String.format("  [%.4f %.4f %.4f %.4f]", r.meanR[2][0], r.meanR[2][1], r.meanR[2][2], r.meanT[2]));
        reportLines.add("  [0 0 0 1]");
        double[] rpy = rotToRpy(r.meanR);
        reportLines.add(String.format(">> T translation (in): (%.2f, %.2f, %.2f)",
                r.meanT[0] * M_TO_IN, r.meanT[1] * M_TO_IN, r.meanT[2] * M_TO_IN));
        reportLines.add(String.format(">> T rpy (deg): (%.2f, %.2f, %.2f)",
                Math.toDegrees(rpy[0]), Math.toDegrees(rpy[1]), Math.toDegrees(rpy[2])));
        reportLines.add(">> per-sample residuals (a bad capture stands out here):");
        for (int i = 0; i < r.n; i++) {
            boolean suspicious = r.rmsTrans > 1e-6 && r.sampleErrT[i] > 3.0 * r.rmsTrans;
            Sample s = samples.get(i);
            reportLines.add(String.format("  s%d: %.2f in / %.2f deg  (traveled %.0f in, turned %.0f deg, tags %s)%s",
                    i + 1, r.sampleErrT[i] * M_TO_IN, r.sampleErrR[i],
                    s.travelIn, s.dHeadDeg, s.tags,
                    suspicious ? "  <-- suspicious" : ""));
        }
        if (r.n >= 3) {
            double[] err = new double[r.n], trav = new double[r.n], turn = new double[r.n];
            for (int i = 0; i < r.n; i++) {
                err[i] = r.sampleErrT[i];
                trav[i] = samples.get(i).travelIn;
                turn[i] = Math.abs(samples.get(i).dHeadDeg);
            }
            reportLines.add(String.format(">> corr(err, distance traveled) = %+.2f  [toward +1: odo drift or scale error]",
                    pearson(err, trav)));
            reportLines.add(String.format(">> corr(err, amount turned)      = %+.2f  [toward +1: heading scale or MT2 yaw skew]",
                    pearson(err, turn)));
        }
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
