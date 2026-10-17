package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;

/**
 * Find the two shoot bar servo positions.
 *
 *   dpad up / down      : fine adjust (+/- 0.005 per loop while held)
 *   right/left bumper   : coarse adjust (+/- 0.05 per loop while held)
 *   A                   : save the current position as CLOSED
 *   B                   : save the current position as OPEN
 *   X / Y               : jump back to the saved CLOSED / OPEN position
 *
 * Saved values are written straight into Shooter.shootLimitClosed /
 * Shooter.shootLimitOpen (they also show up on the FTC Dashboard), so the
 * Shooter uses them immediately in the same app session. Note the numbers
 * from telemetry and copy them into Shooter.java as the permanent defaults.
 */
@TeleOp(name = "ShootLimit Servo Test", group = "Tuning")
public class ShootLimitTest extends OpMode {

    private Servo shootLimit;
    private static double pos = 0.5;

    private boolean aWasDown = false;
    private boolean bWasDown = false;

    @Override
    public void init() {
        shootLimit = hardwareMap.get(Servo.class, "drop");
        pos = Shooter.shootLimitClosed;
        shootLimit.setPosition(pos);

        telemetry.addLine("dpad up/down = fine, bumpers = coarse");
        telemetry.addLine("A = save CLOSED, B = save OPEN, X/Y = jump to saved");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up) pos += 0.005;
        if (gamepad1.dpad_down) pos -= 0.005;
        if (gamepad1.right_bumper) pos += 0.05;
        if (gamepad1.left_bumper) pos -= 0.05;

        if (gamepad1.x) pos = Shooter.shootLimitClosed;
        if (gamepad1.y) pos = Shooter.shootLimitOpen;

        pos = Math.max(0.0, Math.min(1.0, pos));

        if (gamepad1.a && !aWasDown) Shooter.shootLimitClosed = pos;
        aWasDown = gamepad1.a;

        if (gamepad1.b && !bWasDown) Shooter.shootLimitOpen = pos;
        bWasDown = gamepad1.b;

        shootLimit.setPosition(pos);

        telemetry.addData("servo pos", String.format("%.3f", pos));
        telemetry.addData("CLOSED (saved)", String.format("%.3f", Shooter.shootLimitClosed));
        telemetry.addData("OPEN (saved)", String.format("%.3f", Shooter.shootLimitOpen));
        telemetry.addLine("copy the saved values into Shooter.java");
        telemetry.update();
    }
}
