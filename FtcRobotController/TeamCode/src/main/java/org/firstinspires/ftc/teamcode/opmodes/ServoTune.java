package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.reflection.ReflectionConfig;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

/**
 * General servo tuner: pick a servo with X, nudge it to any position, then
 * copy the value from telemetry into the subsystem source. Every target is
 * a public static that the real match code uses, so a tuned value can also
 * just be typed into the dashboard and it takes effect live.
 *
 * gamepad1:
 *   X            : cycle target
 *                  hood -> shootLimit open -> shootLimit closed ->
 *                  flower INTAKING -> INTAKING_FLOWER -> STOP -> OUTTAKING ->
 *                  flower R offset
 *   dpad up/down : fine   (+/- 0.005 per loop while held)
 *   dpad L/R     : coarse (+/- 0.05  per loop while held)
 *   A            : reset current target (0.5, flower offset resets to 0)
 *
 * Where tuned values go:
 *   Shooter.hoodPos / shootLimitOpen / shootLimitClosed   (used by match code)
 *   Intake.flowerPosIntaking / IntakingFlower / Stop / Outtaking
 *                                                        (runtime flower poses)
 *   Intake.flowerOffset  (right servo trim, like Turret.ServoLoff)
 */
@TeleOp(name = "Servo Tune", group = "Tuning")
public class ServoTune extends OpMode {

    private static final String[] TARGETS = {
            "hood  -> Shooter.hoodPos",
            "shootLimit open  -> Shooter.shootLimitOpen",
            "shootLimit closed  -> Shooter.shootLimitClosed",
            "flower INTAKING  -> Intake.flowerPosIntaking",
            "flower INTAKING_FLOWER  -> Intake.flowerPosIntakingFlower",
            "flower STOP  -> Intake.flowerPosStop",
            "flower OUTTAKING  -> Intake.flowerPosOuttaking",
            "flower R offset  -> Intake.flowerOffset",
    };

    private Shooter shooter;
    private Intake intake;

    /** Exact value box (dashboard, "ServoTune" category): the current target
     *  always follows this -- type a position here, or nudge with the dpad. */
    public static double tuneValue = 0.5;

    private int target = 0;
    private boolean xWasDown = false;
    private boolean aWasDown = false;

    @Override
    public void init() {
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        FtcDashboard.getInstance().withConfigRoot(root ->
                root.putVariable("ServoTune", ReflectionConfig.createVariableFromClass(ServoTune.class)));
        tuneValue = getTarget();

        telemetry.addLine("X = cycle servo, dpad up/down = fine, dpad L/R = coarse, A = reset");
        telemetry.addLine("Type an exact position in ServoTune/tuneValue on the dashboard.");
    }

    @Override
    public void loop() {
        if (gamepad1.x && !xWasDown) {
            target = (target + 1) % TARGETS.length;
            tuneValue = getTarget();   // load the new target's current value
        }
        xWasDown = gamepad1.x;

        double delta = 0;
        if (gamepad1.dpad_up) delta += 0.005;
        if (gamepad1.dpad_down) delta -= 0.005;
        if (gamepad1.dpad_right) delta += 0.05;
        if (gamepad1.dpad_left) delta -= 0.05;
        if (delta != 0) tuneValue += delta;

        if (gamepad1.a && !aWasDown) tuneValue = (target == 7 ? 0.0 : 0.5);
        aWasDown = gamepad1.a;

        // push into the target static (clamped) and read back, so both dpad
        // nudges and exact values typed on the dashboard drive the servo
        setTarget(tuneValue);
        tuneValue = getTarget();

        // apply continuously so the servo always shows the value being tuned
        shooter.setHood(Shooter.hoodPos);
        if (target == 1) shooter.shootbarOff();   // applies shootLimitOpen
        if (target == 2) shooter.shootbarOn();    // applies shootLimitClosed
        if (target >= 3 && target <= 6) intake.setFlowers(getTarget());
        if (target == 7) intake.setFlowers(Intake.flowerPosIntakingFlower); // show pair with trim

        telemetry.addData("Target", "[%d/%d] %s", target + 1, TARGETS.length, TARGETS[target]);
        telemetry.addData("Value", "%.4f", getTarget());
        telemetry.addData("Shooter", "hood %.4f | shootLimit %.4f / %.4f",
                Shooter.hoodPos, Shooter.shootLimitOpen, Shooter.shootLimitClosed);
        telemetry.addData("Flowers", "int %.4f | flw %.4f | stop %.4f | out %.4f | Roff %+.4f",
                Intake.flowerPosIntaking, Intake.flowerPosIntakingFlower,
                Intake.flowerPosStop, Intake.flowerPosOuttaking, Intake.flowerOffset);
        telemetry.addLine("copy the tuned value into the matching static when done");
    }

    private double getTarget() {
        switch (target) {
            case 0: return Shooter.hoodPos;
            case 1: return Shooter.shootLimitOpen;
            case 2: return Shooter.shootLimitClosed;
            case 3: return Intake.flowerPosIntaking;
            case 4: return Intake.flowerPosIntakingFlower;
            case 5: return Intake.flowerPosStop;
            case 6: return Intake.flowerPosOuttaking;
            default: return Intake.flowerOffset;
        }
    }

    private void setTarget(double v) {
        // flower offset is an add-on trim, allow negative; others are 0..1
        if (target == 7) v = Math.max(-0.5, Math.min(0.5, v));
        else v = Math.max(0.0, Math.min(1.0, v));
        switch (target) {
            case 0: Shooter.hoodPos = v; break;
            case 1: Shooter.shootLimitOpen = v; break;
            case 2: Shooter.shootLimitClosed = v; break;
            case 3: Intake.flowerPosIntaking = v; break;
            case 4: Intake.flowerPosIntakingFlower = v; break;
            case 5: Intake.flowerPosStop = v; break;
            case 6: Intake.flowerPosOuttaking = v; break;
            default: Intake.flowerOffset = v; break;
        }
    }
}
