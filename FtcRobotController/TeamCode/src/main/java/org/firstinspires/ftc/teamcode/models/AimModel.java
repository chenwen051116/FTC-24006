package org.firstinspires.ftc.teamcode.models;

/**
 * Aim model foundation: (x, y) -> (thetaField, rpm, hoodPos, tAirSec).
 *
 * This is the runtime contract the aim system is built against. Version 0 is
 * the analytic base: theta = atan2(target - robot), everything else constant.
 * Once aim samples are recorded with the "Aim Tune" opmode and fit offline
 * (tools/ fit script, LOO comparison), the internals get replaced by the
 * trained model -- the signature stays the same, so no caller changes:
 *
 *   thetaField = atan2(target - robot) + learned residual dTheta(x, y)
 *
 * Conventions:
 *   - x, y in INCHES (Drivetrain/Pinpoint odometry units), RED alliance frame
 *   - thetaField in RADIANS, field frame (0 = +x axis of the red-frame field)
 *   - rpm = flywheel target RPM, hoodPos = Shooter hood servo position 0..1
 *   - tAirSec = predicted ball flight time in seconds
 *
 * targetX/targetY are the goal position (red frame) -- MEASURE and set these
 * on the FTC Dashboard ("AimModel" category) or here before tuning.
 */
public final class AimModel {

    /** Goal x position, red-frame field, inches. MEASURE AND SET. */
    public static double targetX = 0.0;
    /** Goal y position, red-frame field, inches. MEASURE AND SET. */
    public static double targetY = 0.0;

    // v0 placeholders, replaced by the trained model later
    public static double defaultRPM = 2500;
    public static double defaultHood = 0.5;
    public static double defaultTAir = 1.0;

    private AimModel() {}

    /**
     * Predict the full shooting solution for a robot position.
     *
     * @param x robot x, red-frame, inches (odometry)
     * @param y robot y, red-frame, inches (odometry)
     * @return {thetaFieldRad, rpm, hoodPos, tAirSec}
     */
    public static double[] predict(double x, double y) {
        double theta = Math.atan2(targetY - y, targetX - x);
        return new double[]{theta, defaultRPM, defaultHood, defaultTAir};
    }
}
