package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.models.AimModel;

/**
 * Turns the aim model into actuator commands.
 *
 * Given the robot's odometry pose (and velocity, for shooting while moving),
 * this computes the turret angle (robot-relative, degrees -- ready for
 * Turret.settoangle), the flywheel RPM, the hood position, and the predicted
 * ball flight time.
 *
 * The math:
 *   base     = atan2(target - robot)              geometric bearing
 *   resid    = model.theta - base                 learned correction (v0: 0)
 *   lead     = atan2(target - robot - v * tAir)   moving-shot compensation:
 *              the ball inherits the robot velocity, so we aim BEHIND the
 *              target by (v * tAir); the new tAir barely changes, one pass
 *   thetaRel = wrap(lead + resid - heading)       field -> robot frame
 */
public final class AimSolver {

    private AimSolver() {}

    /** Wrap an angle to (-pi, pi]. */
    public static double wrapPi(double a) {
        while (a > Math.PI) a -= 2.0 * Math.PI;
        while (a <= -Math.PI) a += 2.0 * Math.PI;
        return a;
    }

    /**
     * Full aim solution for the current robot state. Pass velocities of 0
     * for stationary shooting.
     *
     * @param x        robot x, red-frame, inches (odometry)
     * @param y        robot y, red-frame, inches (odometry)
     * @param heading  robot heading, radians (odometry)
     * @param vx       robot x velocity, in/s (0 if stationary)
     * @param vy       robot y velocity, in/s (0 if stationary)
     * @return {turretRelDeg, rpm, hoodPos, tAirSec} -- turretRelDeg is the
     *         robot-relative aim angle, degrees, ready for Turret.settoangle
     */
    public static double[] solve(double x, double y, double heading, double vx, double vy) {
        double[] m = AimModel.predict(x, y);
        double thetaModel = m[0];
        double tAir = m[3];

        // geometric bearing to the goal
        double tx = AimModel.targetX - x;
        double ty = AimModel.targetY - y;
        double base = Math.atan2(ty, tx);

        // learned residual on top of the bearing (v0: identically 0)
        double resid = wrapPi(thetaModel - base);

        // moving-shot lead: aim at target - v * tAir
        double lead = Math.atan2(ty - vy * tAir, tx - vx * tAir);

        double turretRelDeg = Math.toDegrees(wrapPi(lead + resid - heading));
        return new double[]{turretRelDeg, m[1], m[2], m[3]};
    }
}
