package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * Solo odometry pod test -- NO Pinpoint needed.
 *
 * The goBILDA 4-bar pod is a plain quadrature encoder, so it plugs directly
 * into any REV hub encoder port (the 4-pin JST PH connector next to a motor
 * port, labeled 0-3). Configure a DcMotor named "podEncoder" on that motor
 * channel -- no actual motor needs to be attached.
 *
 * Checks:
 *   - spin the wheel by hand -> counts change smoothly, no jumps/dropouts
 *   - mark the wheel, spin exactly 10 full turns -> "wheel turns" reads ~10.0
 *     (goBILDA 4-bar pod = 2000 countable events per wheel revolution)
 *   - roll the pod forward -> counts INCREASE
 *   - rotate the pod housing without spinning the wheel -> counts must NOT move
 *
 * Press A to zero the baseline before a measured run.
 */
@TeleOp(name = "Pod Solo Test (Hub)", group = "Tuning")
public class PodSoloTest extends OpMode {

    private DcMotorEx pod;

    private int base = 0;
    private boolean aWasDown = false;

    @Override
    public void init() {
        pod = hardwareMap.get(DcMotorEx.class, "pod");
        pod.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pod.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        base = 0;

        telemetry.addLine("Spin the pod wheel by hand.");
        telemetry.addLine("A = zero the baseline.");
        telemetry.addLine("Spin a marked wheel exactly 10 turns ->");
        telemetry.addLine("'wheel turns' should read ~10.0 (2000 counts/rev).");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.a && !aWasDown) {
            base = pod.getCurrentPosition();
        }
        aWasDown = gamepad1.a;

        int pos = pod.getCurrentPosition();
        int delta = pos - base;

        telemetry.addData("raw counts", pos);
        telemetry.addData("since baseline", delta);
        telemetry.addData("wheel turns", String.format("%.2f", delta / 2000.0));
        telemetry.addData("velocity (counts/s)", (int) pod.getVelocity());
        telemetry.addLine();
        telemetry.addData("baseline", "A to re-zero (current: " + base + ")");
        telemetry.update();
    }
}
