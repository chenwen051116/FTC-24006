package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Turret;

/**
 * Fine-tune the turret servos' startup alignment.
 *
 * The two turret servos can rest at slightly different angles at power-on
 * (hobby servo tolerance). Turret.ServoLoff is the left servo's center trim
 * that compensates the difference so both hold the turret straight.
 *
 *   dpad up / down      : fine adjust (+/- 0.001 per loop while held)
 *   right/left bumper   : coarse adjust (+/- 0.01 per loop while held)
 *   A                   : reset trim to 0
 *
 * Physically hold/align the turret straight ahead, nudge until both servos
 * keep it there without fighting, then note the value from telemetry and
 * copy it into Turret.ServoLoff in Turret.java as the permanent default.
 * The trim is written straight into Turret.ServoLoff, so the Turret (and
 * the FTC Dashboard "Turret" category) uses it immediately in this session.
 */
@TeleOp(name = "Turret Servo Align", group = "Tuning")
public class TurretServoAlign extends OpMode {

    private Turret turret;

    private boolean aWasDown = false;

    @Override
    public void init() {
        turret = new Turret(hardwareMap, true);
        turret.autoreset();

        telemetry.addLine("Align the turret straight, then trim until it holds.");
        telemetry.addLine("dpad up/down = fine (+/-0.001), bumpers = coarse (+/-0.01), A = reset");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up) Turret.ServoLoff += 0.001;
        if (gamepad1.dpad_down) Turret.ServoLoff -= 0.001;
        if (gamepad1.right_bumper) Turret.ServoLoff += 0.01;
        if (gamepad1.left_bumper) Turret.ServoLoff -= 0.01;

        if (gamepad1.a && !aWasDown) Turret.ServoLoff = 0;
        aWasDown = gamepad1.a;

        // keep the trim sane: center +/- 0.4 of full servo range
        Turret.ServoLoff = Math.max(-0.4, Math.min(0.4, Turret.ServoLoff));

        // re-center with the current trim every loop
        turret.centering();

        telemetry.addData("ServoLoff", String.format("%.6f", Turret.ServoLoff));
        telemetry.addData("left cmd", String.format("%.4f", 0.5 + Turret.ServoLoff));
        telemetry.addData("right cmd", String.format("%.4f", 0.5));
        telemetry.addLine("copy ServoLoff into Turret.java when done");
        telemetry.update();
    }
}
