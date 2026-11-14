package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.reflection.ReflectionConfig;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLFieldMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Limelight 3A vision subsystem: AprilTag-based robot localization ("Robot GPS").
 *
 * Driver: the FTC SDK's built-in Limelight driver (com.qualcomm.hardware.limelightvision.*),
 * per https://docs.limelightvision.io/docs/docs-limelight/getting-started/limelight-3a#for-ftc-1
 * No extra gradle dependency is needed on SDK 12.
 *
 * One-time setup (web UI at http://limelight.ftc:5801 or the device IP):
 *   1. Plug the Limelight into the Control Hub's USB 3.0 port; in the Driver
 *      Station config it appears as an "Ethernet Device" -- name it "limelight".
 *   2. In the AprilTag pipeline: enable "Full 3D" (Advanced tab) and select the
 *      correct field map for the current season (ships with a built-in map).
 *   3. Enter the camera mount position/rotation relative to the center of the
 *      robot footprint (Settings -> Robot Coordinate Space). This is REQUIRED
 *      for MegaTag2 accuracy.
 *   4. Put the AprilTag pipeline in a known slot (pipelineIndex below, default 0).
 *
 * Coordinate systems:
 *   - Limelight "botpose": field frame, (0,0,0) at the CENTER of the field floor,
 *     x/y in meters, yaw 0 = blue alliance on the robot's left. This subsystem
 *     converts to inches.
 *   - The Drivetrain's world frame: (0,0) wherever the robot was zeroed
 *     (dpad_up), heading in radians. To express Limelight readings in THAT
 *     frame, place the robot at a known spot, call calibrate(knownPose), and
 *     then read getPoseDrivetrainFrame().
 */
public class Limelight {

    // ------------------------------------------------------------------
    // Tunables (Dashboard: "Limelight" category)
    // ------------------------------------------------------------------
    /** Index of the AprilTag pipeline (slot number in the web UI). */
    public static int pipelineIndex = 4;

    /** true = MegaTag2 (IMU-fused, needs updateRobotHeading each loop), false = MegaTag1. */
    public static boolean useMT2 = true;

    /** Discard readings older than this (ms). */
    public static double maxStalenessMs = 250;

    /** How often Limelight sends data to the RC (Hz). */
    public static double pollRateHz = 100;

    private static final double METERS_TO_INCHES = 39.3700787402;

    private final Limelight3A limelight;
    private LLResult lastResult;

    // ---- drivetrain-frame calibration (set by calibrate()) ----
    private boolean calibrated = false;
    private Pose llAtCalibration = null; // LL-frame pose captured at calibration
    private Pose knownPoseAtCalibration = null; // drivetrain-frame pose at calibration

    public Limelight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz((int) pollRateHz);
        limelight.pipelineSwitch(pipelineIndex);
        limelight.start();

        // Explicitly register our @Config statics with the dashboard
        // (the automatic DEX scan misses secondary dex files in large apps).
        FtcDashboard.getInstance().withConfigRoot(root ->
                root.putVariable("Limelight", ReflectionConfig.createVariableFromClass(Limelight.class)));
    }

    // ------------------------------------------------------------------
    // Feed the robot heading (REQUIRED for MegaTag2)
    // ------------------------------------------------------------------

    /**
     * Tell the Limelight which way the robot is facing. Call this EVERY loop
     * (before reading the pose) with the robot's yaw in degrees, e.g. from the
     * Pinpoint: Math.toDegrees(drivetrain.pose().getHeading()).
     */
    public void updateRobotHeading(double yawDeg) {
        limelight.updateRobotOrientation(yawDeg);
    }

    // ------------------------------------------------------------------
    // Validity / health
    // ------------------------------------------------------------------

    /** True when there is a fresh, valid AprilTag-based pose available. */
    public boolean isValid() {
        LLResult result = latest();
        return result != null
                && result.isValid()
                && result.getBotposeTagCount() > 0
                && result.getStaleness() <= maxStalenessMs;
    }

    /** Number of AprilTags used in the last botpose solve. */
    public int getTagCount() {
        LLResult result = latest();
        return result == null ? 0 : result.getBotposeTagCount();
    }

    /** Age of the last result in ms. */
    public long getStalenessMs() {
        LLResult result = latest();
        return result == null ? Long.MAX_VALUE : result.getStaleness();
    }

    /** Pipeline latency of the last result in ms. */
    public double getTargetingLatencyMs() {
        LLResult result = latest();
        return result == null ? 0 : result.getTargetingLatency();
    }

    /** Crosshair horizontal offset in degrees (for "strafe until centered" style aiming). */
    public double getTx() {
        LLResult result = latest();
        return result == null ? 0 : result.getTx();
    }

    /** Crosshair vertical offset in degrees. */
    public double getTy() {
        LLResult result = latest();
        return result == null ? 0 : result.getTy();
    }

    /**
     * First visible AprilTag that has a valid target-space robot pose, or null.
     * Use getRobotPoseTargetSpace() on the result for the robot's 6D pose in
     * that tag's coordinate space (meters + degrees).
     */
    public LLResultTypes.FiducialResult getPrimaryFiducial() {
        LLResult result = latest();
        if (result == null || !result.isValid()) {
            return null;
        }
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null) {
            return null;
        }
        for (LLResultTypes.FiducialResult f : fiducials) {
            if (f != null && f.getRobotPoseTargetSpace() != null) {
                return f;
            }
        }
        return null;
    }

    // ------------------------------------------------------------------
    // Robot pose from AprilTags
    // ------------------------------------------------------------------

    /**
     * Robot pose in the LIMELIGHT FIELD frame (field-center origin, inches,
     * yaw in radians). null when no fresh valid pose.
     */
    public Pose getPoseField() {
        LLResult result = latest();
        if (result == null || !result.isValid() || result.getStaleness() > maxStalenessMs) {
            return null;
        }
        Pose3D botpose = useMT2 ? result.getBotpose_MT2() : result.getBotpose();
        if (botpose == null) {
            return null;
        }
        return new Pose(
                botpose.getPosition().x * METERS_TO_INCHES,
                botpose.getPosition().y * METERS_TO_INCHES,
                wrapAngle(Math.toRadians(botpose.getOrientation().getYaw(AngleUnit.DEGREES))));
    }

    /** Robot heading from the vision solve, radians, or 0 if invalid. */
    public double getHeadingRad() {
        Pose pose = getPoseField();
        return pose == null ? 0 : pose.heading();
    }

    /**
     * Re-define the LL->drivetrain transform: call while the robot is
     * PHYSICALLY at a known spot and the drivetrain pose has been zeroed/moved
     * to match (e.g. new Pose(0,0,0) after dpad_up). Afterwards,
     * getPoseDrivetrainFrame() returns vision readings in the same world frame
     * the Dashboard field view uses.
     */
    public void calibrate(Pose knownDrivetrainPose) {
        Pose llPose = getPoseField();
        if (llPose != null) {
            llAtCalibration = llPose;
            knownPoseAtCalibration = knownDrivetrainPose;
            calibrated = true;
        }
    }

    public boolean isCalibrated() {
        return calibrated;
    }

    /**
     * Robot pose expressed in the DRIVETRAIN world frame (the frame that was
     * active when calibrate() was last called), or null if not calibrated /
     * no fresh vision. This is the value to compare against
     * drivetrain.pose() for odometry correction (tag re-localization).
     */
    public Pose getPoseDrivetrainFrame() {
        if (!calibrated) {
            return null;
        }
        Pose ll = getPoseField();
        if (ll == null) {
            return null;
        }
        // Relative motion of the robot since calibration, in the LL-at-calibration frame
        double dH = wrapAngle(ll.heading() - llAtCalibration.heading());
        double dxL = ll.x() - llAtCalibration.x();
        double dyL = ll.y() - llAtCalibration.y();
        double rx = Math.cos(llAtCalibration.heading()) * dxL + Math.sin(llAtCalibration.heading()) * dyL;
        double ry = -Math.sin(llAtCalibration.heading()) * dxL + Math.cos(llAtCalibration.heading()) * dyL;
        // Apply the known pose at calibration to land in the drivetrain frame
        double wx = knownPoseAtCalibration.x()
                + Math.cos(knownPoseAtCalibration.heading()) * rx
                - Math.sin(knownPoseAtCalibration.heading()) * ry;
        double wy = knownPoseAtCalibration.y()
                + Math.sin(knownPoseAtCalibration.heading()) * rx
                + Math.cos(knownPoseAtCalibration.heading()) * ry;
        double wh = wrapAngle(knownPoseAtCalibration.heading() + dH);
        return new Pose(wx, wy, wh);
    }

    // ------------------------------------------------------------------
    // Field map upload (BIOBUZZ "HiveFlat", exported from the 3D Map Builder)
    // ------------------------------------------------------------------

    // All tags: 82.5 mm, apriltag3_36h11_classic, tag center height 1.075 m.
    // Two mounting orientations on the hive walls:
    //   pattern A (x = -0.32 wall): rotation [[0,0,-1],[0,-1,0],[-1,0,0]]
    //   pattern B (x = +0.32 wall): rotation [[0,0, 1],[0, 1,0],[-1,0,0]]
    private static final double TAG_SIZE_MM = 82.5;
    private static final String TAG_FAMILY = "apriltag3_36h11_classic";
    private static final double TAG_Z = 1.075;

    private static LLFieldMap.Fiducial hiveTag(int id, double tx, double ty, boolean patternA) {
        List<Double> t = new ArrayList<>(16);
        if (patternA) {
            t.addAll(Arrays.asList(0.0, 0.0, -1.0, tx,
                    0.0, -1.0, 0.0, ty,
                    -1.0, 0.0, 0.0, TAG_Z,
                    0.0, 0.0, 0.0, 1.0));
        } else {
            t.addAll(Arrays.asList(0.0, 0.0, 1.0, tx,
                    0.0, 1.0, 0.0, ty,
                    -1.0, 0.0, 0.0, TAG_Z,
                    0.0, 0.0, 0.0, 1.0));
        }
        return new LLFieldMap.Fiducial(id, TAG_SIZE_MM, TAG_FAMILY, t, true);
    }

    /**
     * Upload the BIOBUZZ HiveFlat field map to the Limelight from code
     * (no web UI needed). Call once at opmode init for your alliance:
     *   limelight.uploadHiveFieldMap(true);   // blue: tags 38-45
     *   limelight.uploadHiveFieldMap(false);  // red:  tags 30-37
     * Returns true on success.
     */
    public boolean uploadHiveFieldMap(boolean blueAlliance) {
        List<LLFieldMap.Fiducial> tags = new ArrayList<>();
        if (blueAlliance) {
            // x = -0.32 wall (pattern A)
            tags.add(hiveTag(45, -0.32, 0.1585, true));
            tags.add(hiveTag(44, -0.32, 0.2535, true));
            tags.add(hiveTag(43, -0.32, 0.3935, true));
            tags.add(hiveTag(42, -0.32, 0.4885, true));
            // x = +0.32 wall (pattern B)
            tags.add(hiveTag(38, 0.32, 0.1585, false));
            tags.add(hiveTag(39, 0.32, 0.2535, false));
            tags.add(hiveTag(40, 0.32, 0.3935, false));
            tags.add(hiveTag(41, 0.32, 0.4885, false));
        } else {
            // x = -0.32 wall (pattern A)
            tags.add(hiveTag(30, -0.32, -0.1585, true));
            tags.add(hiveTag(31, -0.32, -0.2535, true));
            tags.add(hiveTag(32, -0.32, -0.3935, true));
            tags.add(hiveTag(33, -0.32, -0.4885, true));
            // x = +0.32 wall (pattern B)
            tags.add(hiveTag(34, 0.32, -0.4885, false));
            tags.add(hiveTag(35, 0.32, -0.3935, false));
            tags.add(hiveTag(36, 0.32, -0.2535, false));
            tags.add(hiveTag(37, 0.32, -0.1585, false));
        }
        return limelight.uploadFieldmap(new LLFieldMap(tags, "ftc"), null);
    }

    // ------------------------------------------------------------------
    // Misc
    // ------------------------------------------------------------------

    /** Switch pipeline by slot index (as configured in the web UI). */
    public void switchPipeline(int index) {
        limelight.pipelineSwitch(index);
    }

    /** Push a standard telemetry block. */
    public void telemetry(Telemetry telemetry) {
        LLResult result = latest();
        boolean valid = result != null && result.isValid();
        telemetry.addData("Limelight", valid ? "TAGS OK" : "no valid pose");
        if (valid) {
            Pose pose = getPoseField();
            if (pose != null) {
                telemetry.addData("LL pose (in)", "(%.1f, %.1f, %.1f deg)",
                        pose.x(), pose.y(), Math.toDegrees(pose.heading()));
            }
            telemetry.addData("LL tag count / staleness", "%d / %d ms",
                    result.getBotposeTagCount(), result.getStaleness());
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            StringBuilder ids = new StringBuilder();
            for (LLResultTypes.FiducialResult f : fiducials) {
                if (ids.length() > 0) {
                    ids.append(", ");
                }
                ids.append(f.getFiducialId());
            }
            telemetry.addData("LL tag ids", ids.length() > 0 ? ids.toString() : "none");
        }
        telemetry.addData("LL calibrated", isCalibrated() ? "yes" : "NO (call calibrate())");
    }

    private LLResult latest() {
        lastResult = limelight.getLatestResult();
        return lastResult;
    }

    private static double wrapAngle(double a) {
        a = (a + Math.PI) % (2 * Math.PI);
        if (a < 0) {
            a += 2 * Math.PI;
        }
        return a - Math.PI;
    }
}
