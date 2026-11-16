package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.config.reflection.ReflectionConfig;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

/**
 * Turret subsystem, ported from last season's robot and adapted for Albert.
 *
 * Two servos (turretServoL / turretServoR) rotate the shooter turret. The
 * turret aims by arc angle: settoangle(deg) maps the angle to a servo
 * position via arctoDegree; with auto aim off, periodic() centers it.
 *
 * Aiming flow (same as last year):
 *   - auto aim:  turret.aimangle = <angle to target>; turret.updateAutoShoot(true)
 *   - manual:    turret.updateAutoShoot(false) -> turret recenters
 *   - trim:      turret.changeOffset(deg) -> persistent aim offset
 */
@Config
public class Turret extends SubsystemBase {

    private final ServoImplEx turretServoLeft;
    private final ServoImplEx turretServoRight;

    /** Auto-aim angle in degrees (set by the aiming code, consumed in periodic). */
    public double aimangle = 0;
    /** Persistent trim added on top of aimangle (adjust with changeOffset). */
    public double offset = 0;
    /** Last commanded servo offset from center (telemetry/debugging). */
    public double aimposition = 0;

    /** Auto aim enabled (set by updateAutoShoot). */
    public boolean shooterAuto = false;
    /** Force aiming even when shooterAuto is off. */
    public boolean autoForce = false;
    /** Set true to freeze the turret and ignore aim/center commands. */
    public boolean automode = false;

    /** Servo position change per degree of turret arc. */
    public static double arctoDegree = 0.155019;

    /** Commands within +/- this many degrees of the servo wrap point (straight
     *  backward, +/-180 from front) snap to the nearest end of the range. */
    public static double deadZoneDeg = 5.0;
    /** Left servo center trim (compensates mounting difference between the two servos). */
    public static double ServoLoff = 0;

    public Turret(HardwareMap hardwareMap, boolean isTele) {
        turretServoLeft = hardwareMap.get(ServoImplEx.class, "hood");
        turretServoRight = hardwareMap.get(ServoImplEx.class, "shootLimit");
        turretServoLeft.setPwmRange(new PwmControl.PwmRange(500, 2500));
        turretServoRight.setPwmRange(new PwmControl.PwmRange(500, 2500));

        // Explicitly register our @Config statics with the dashboard
        // (the automatic DEX scan misses secondary dex files in large apps).
        FtcDashboard.getInstance().withConfigRoot(root ->
                root.putVariable("Turret", ReflectionConfig.createVariableFromClass(Turret.class)));
    }

    /** Park the turret at center (call at the start of an auto). */
    public void autoreset() {
        turretServoLeft.setPosition(0.5 + ServoLoff);
        turretServoRight.setPosition(0.5);
    }

    /** Aim the turret to an arc angle in degrees (0 = straight ahead). */
    public void settoangle(double arcangle) {
        // normalize to (-180, 180] so any input convention works (0..360 or signed)
        arcangle = wrapDeg(arcangle);
        // dead zone at the turning point (servo 0 <-> 1, aiming straight
        // backward): commands within deadZoneDeg of +/-180 snap to the near
        // end so the turret pins at the wrap instead of swinging a full turn
        if (Math.abs(arcangle) > 180.0 - deadZoneDeg) {
            arcangle = Math.signum(arcangle) * 180.0;
        }
        aimposition = -arcangle * arctoDegree;
        double left = 0.5 + ServoLoff + aimposition;
        if (left > 0.995) {
            turretServoLeft.setPosition(0.995);
            turretServoRight.setPosition(0.995);
        } else if (left < 0.005) {
            turretServoLeft.setPosition(0.005);
            turretServoRight.setPosition(0.005);
        } else {
            turretServoLeft.setPosition(left);
            turretServoRight.setPosition(0.5 + aimposition);
        }
    }

    /** Wrap an angle in degrees into (-180, 180]. */
    private static double wrapDeg(double a) {
        double w = (a + 180.0) % 360.0;
        if (w < 0) w += 360.0;
        return w - 180.0;
    }

    /** Return the turret to center. */
    public void centering() {
        turretServoLeft.setPosition(0.5 + ServoLoff);
        turretServoRight.setPosition(0.5);
    }

    /** Enable/disable auto aiming. */
    public void updateAutoShoot(boolean auto) {
        shooterAuto = auto;
    }

    /** Adjust the persistent aim trim. */
    public void changeOffset(double change) {
        offset += change;
    }

    @Override
    public void periodic() {
        if (!automode) {
            if (shooterAuto || autoForce) {
                settoangle(aimangle + offset);
            } else {
                centering();
            }
        }
    }

    /**
     * Clear auto flags so TeleOp starts clean.
     */
    public void resetTeleop() {
        automode = false;
        shooterAuto = false;
        autoForce = false;
    }
}
