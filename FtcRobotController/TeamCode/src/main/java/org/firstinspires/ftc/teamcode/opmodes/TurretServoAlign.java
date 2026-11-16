package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.reflection.ReflectionConfig;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Turret;

/**
 * Turret bench tool: command an exact angle AND fine-tune the servo pair
 * alignment.
 *
 * Angle command (this is the "turn turret to a specific angle" path):
 *   angleDeg (public static, dashboard-tunable under "TurretServoAlign")
 *                      : the commanded arc angle in degrees, 0 = robot front.
 *                        Typed on the FTC Dashboard for exact values, or
 *                        nudged with the gamepad (below). Routed through
 *                        Turret.settoangle(), so arctoDegree scaling,
 *                        ServoLoff trim, the +/-180 seam dead zone and the
 *                        endpoint clamps all apply.
 *   dpad right / left  : angle +/- 0.5 deg per loop while held
 *   B                  : reset angle to 0 (front)
 *
 * Servo pair alignment (startup-offset trim for the left servo):
 *   dpad up / down     : fine adjust (+/- 0.001 per loop while held)
 *   right/left bumper  : coarse adjust (+/- 0.01 per loop while held)
 *   A                  : reset trim to 0
 *
 * Note: commanded angles are only as accurate as Turret.arctoDegree -- if
 * that scale is still uncalibrated for this turret, use this teleop to
 * calibrate it too (command a known angle, measure the real rotation,
 * arctoDegree = servoDelta / degrees).
 */
@TeleOp(name = "Turret Servo Align", group = "Tuning")
public class TurretServoAlign extends OpMode {

    /** Commanded turret angle in degrees (0 = robot front). Dashboard-tunable. */
    public static double angleDeg = 0.0;

    private Turret turret;

    private boolean aWasDown = false;
    private boolean bWasDown = false;

    @Override
    public void init() {
        turret = new Turret(hardwareMap, true);
        turret.autoreset();

        FtcDashboard.getInstance().withConfigRoot(root ->
                root.putVariable("TurretServoAlign", ReflectionConfig.createVariableFromClass(TurretServoAlign.class)));

        telemetry.addLine("dpad L/R = angle +/-0.5 deg, B = angle 0 (front)");
        telemetry.addLine("dpad up/down = trim fine (+/-0.001), bumpers = coarse (+/-0.01), A = trim 0");
        telemetry.update();
    }

    @Override
    public void loop() {
        // ---- angle command ----
        if (gamepad1.dpad_right) angleDeg += 0.5;
        if (gamepad1.dpad_left) angleDeg -= 0.5;

        if (gamepad1.b && !bWasDown) angleDeg = 0;
        bWasDown = gamepad1.b;

        // keep the commanded angle in the turret's range
        angleDeg = Math.max(-180.0, Math.min(180.0, angleDeg));

        // ---- servo pair trim ----
        if (gamepad1.dpad_up) Turret.ServoLoff += 0.001;
        if (gamepad1.dpad_down) Turret.ServoLoff -= 0.001;
        if (gamepad1.right_bumper) Turret.ServoLoff += 0.01;
        if (gamepad1.left_bumper) Turret.ServoLoff -= 0.01;

        if (gamepad1.a && !aWasDown) Turret.ServoLoff = 0;
        aWasDown = gamepad1.a;

        // keep the trim sane: center +/- 0.4 of full servo range
        Turret.ServoLoff = Math.max(-0.4, Math.min(0.4, Turret.ServoLoff));

        // command the angle every loop (0 deg = center + trim, same as before)
        turret.settoangle(angleDeg);

        double aimPos = -angleDeg * Turret.arctoDegree;
        telemetry.addData("angle cmd (deg)", String.format("%.2f", angleDeg));
        telemetry.addData("ServoLoff", String.format("%.6f", Turret.ServoLoff));
        telemetry.addData("left cmd", String.format("%.4f", clamp01(0.5 + Turret.ServoLoff + aimPos)));
        telemetry.addData("right cmd", String.format("%.4f", clamp01(0.5 + aimPos)));
        telemetry.addData("arctoDegree", String.format("%.6f", Turret.arctoDegree));
        telemetry.addLine("angle accuracy depends on arctoDegree (calibrate here)");
        telemetry.addLine("copy ServoLoff into Turret.java when done");
        telemetry.update();
    }

    private static double clamp01(double v) {
        return Math.max(0.0, Math.min(1.0, v));
    }
}
