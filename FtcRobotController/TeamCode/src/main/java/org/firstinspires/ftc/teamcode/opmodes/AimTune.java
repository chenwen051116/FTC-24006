package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.reflection.ReflectionConfig;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.models.AimModel;
import org.firstinspires.ftc.teamcode.subsystems.AimSolver;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import java.io.File;
import java.io.FileWriter;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Aim tuning teleop: drive + turn the WHOLE ROBOT to aim (turret stays
 * centered at robot front), so the recorded heading IS the field aim angle.
 *
 * Everything is set live from the FTC Dashboard:
 *   AimModel.targetX / targetY : goal position, red frame, inches (measure first!)
 *   Shooter.aimRPM             : flywheel target RPM (0 = stopped)
 *   Shooter.hoodPos            : hood servo position (applied every loop)
 *   AimTune.flyTime            : measured ball flight time (stopwatch), recorded per sample
 *
 * gamepad1:
 *   left stick / right stick x : drive
 *   A (hold)                   : feed balls (shoot bar open)
 *   X                          : save this sample (parked + aimed + scoring)
 *   Y                          : export CSV to /sdcard/FIRST/aim_samples.csv
 *
 * Each saved sample stores: x, y, heading (= the aim angle, turret at 0),
 * Shooter.aimRPM, Shooter.hoodPos, AimTune.flyTime.
 *
 * The "dTheta" telemetry line is the live residual the model will learn:
 * your aimed heading minus the geometric bearing to the goal.
 */
@TeleOp(name = "Aim Tune", group = "Tuning")
public class AimTune extends OpMode {

    /** Measured ball flight time for the current distance; set per sample. */
    public static double flyTime = 1.0;    // sec

    private Drivetrain drivetrain;
    private Turret turret;
    private Shooter shooter;

    private final List<double[]> samples = new ArrayList<>();

    private boolean xWasDown = false;
    private boolean yWasDown = false;
    private String status = null;

    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap, true);
        turret = new Turret(hardwareMap, true);
        shooter = new Shooter(hardwareMap);

        // explicit dashboard registration (the automatic @Config scan misses
        // secondary dex files in large apps); Shooter registers its own statics
        FtcDashboard.getInstance().withConfigRoot(root -> {
            root.putVariable("AimModel", ReflectionConfig.createVariableFromClass(AimModel.class));
            root.putVariable("AimTune", ReflectionConfig.createVariableFromClass(AimTune.class));
        });

        telemetry.addLine("Set AimModel.targetX/Y, Shooter.aimRPM/hoodPos, AimTune.flyTime on the dashboard.");
        telemetry.addLine("Turn the robot to aim, X = save sample, Y = export CSV.");
    }

    @Override
    public void loop() {
        // ---- drive ----
        drivetrain.teleDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        drivetrain.period();

        double x = drivetrain.pose().x();
        double y = drivetrain.pose().y();
        double heading = drivetrain.pose().heading();

        // ---- turret locked to robot front ----
        turret.centering();

        // ---- flywheel at the dashboard-set aimRPM, hood at hoodPos ----
        shooter.setTargetRPM(Shooter.aimRPM);
        shooter.updateFlywheelPID();
        shooter.setHood(Shooter.hoodPos);

        // ---- feed balls while A is held ----
        if (gamepad1.a) shooter.shootbarOff(); else shooter.shootbarOn();

        // ---- live residual: what the model will learn ----
        double base = Math.atan2(AimModel.targetY - y, AimModel.targetX - x);
        double dthetaDeg = Math.toDegrees(AimSolver.wrapPi(heading - base));
        double dist = Math.hypot(AimModel.targetX - x, AimModel.targetY - y);

        // ---- X: save sample ----
        if (gamepad1.x && !xWasDown) {
            samples.add(new double[]{x, y, heading,
                    Shooter.aimRPM, Shooter.hoodPos, flyTime});
            status = String.format(Locale.US,
                    "saved #%d: (%.1f, %.1f) in, dist %.1f, aim %.2f deg, dTheta %.2f deg",
                    samples.size(), x, y, dist, Math.toDegrees(heading), dthetaDeg);
        }
        xWasDown = gamepad1.x;

        // ---- Y: export CSV ----
        if (gamepad1.y && !yWasDown) {
            int exported = exportCsv();
            status = exported >= 0
                    ? ">> exported " + exported + " samples: adb pull /sdcard/FIRST/aim_samples.csv"
                    : ">> CSV export FAILED";
        }
        yWasDown = gamepad1.y;

        // ---- telemetry ----
        telemetry.addData("Odo", "(%.1f, %.1f) in, aim %.2f deg", x, y, Math.toDegrees(heading));
        telemetry.addData("Dist to goal", "%.1f in", dist);
        telemetry.addData("dTheta residual", "%.2f deg", dthetaDeg);
        telemetry.addData("RPM", "target %.0f, actual %.0f",
                shooter.getTargetRPM(), shooter.getFlyWheelRPM());
        telemetry.addData("Hood", "%.3f", Shooter.hoodPos);
        telemetry.addData("flyTime", "%.2f s", flyTime);
        telemetry.addData("Samples", "%d", samples.size());
        if (status != null) telemetry.addLine(status);
    }

    /**
     * Write all saved samples to /sdcard/FIRST/aim_samples.csv.
     * Pull it afterwards:  adb pull /sdcard/FIRST/aim_samples.csv
     */
    private int exportCsv() {
        File dir = new File("/sdcard/FIRST");
        if (!dir.exists()) dir.mkdirs();
        try (PrintWriter w = new PrintWriter(new FileWriter(new File(dir, "aim_samples.csv")))) {
            w.println("x_in,y_in,heading_rad,rpm,hood_pos,flytime_s");
            for (double[] s : samples) {
                w.println(String.format(Locale.US, "%.3f,%.3f,%.6f,%.0f,%.4f,%.3f",
                        s[0], s[1], s[2], s[3], s[4], s[5]));
            }
            return samples.size();
        } catch (Exception e) {
            return -1;
        }
    }
}
