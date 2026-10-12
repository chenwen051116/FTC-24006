package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Solo odometry pod test -- reads raw encoder counts straight off the Pinpoint.
 *
 * Works with one or two pods plugged in. Raw counts do not depend on the pod
 * offsets, resolution, or configured directions, so this isolates the pod itself:
 *   - spin the wheel by hand -> counts should change smoothly
 *   - mark the wheel, spin exactly 10 full turns -> "wheel turns" should read ~10.0
 *     (goBILDA 4-bar pod = 2000 countable events per wheel revolution)
 *   - roll the pod forward (top of wheel moving forward) -> counts should INCREASE
 *   - rotate the pod housing without spinning the wheel -> counts should NOT move
 *
 * Press A to zero the baseline before a measured run.
 */
@TeleOp(name = "Pod Test", group = "Tuning")
public class PodTest extends OpMode {

    private GoBildaPinpointDriver pinpoint;

    private int baseX = 0, baseY = 0;
    private boolean aWasDown = false;

    @Override
    public void init() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        // known directions so the sign is predictable (raw counts are unaffected)
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        telemetry.addLine("Spin the pod wheel by hand.");
        telemetry.addLine("A = zero the baseline.");
        telemetry.addLine("Spin a marked wheel exactly 10 turns ->");
        telemetry.addLine("'wheel turns' should read ~10.0 (2000 counts/rev).");
        telemetry.update();
    }

    @Override
    public void loop() {
        pinpoint.update();

        if (gamepad1.a && !aWasDown) {
            baseX = pinpoint.getEncoderX();
            baseY = pinpoint.getEncoderY();
        }
        aWasDown = gamepad1.a;

        int x = pinpoint.getEncoderX();
        int y = pinpoint.getEncoderY();
        int dx = x - baseX;
        int dy = y - baseY;

        telemetry.addData("Status", pinpoint.getDeviceStatus());
        telemetry.addData("Pinpoint loop", pinpoint.getLoopTime() + " us / " + (int) pinpoint.getFrequency() + " Hz");
        telemetry.addLine();
        telemetry.addData("X raw", x);
        telemetry.addData("Y raw", y);
        telemetry.addLine();
        telemetry.addData("X since baseline", dx + "   (" + String.format("%.2f", dx / 2000.0) + " wheel turns)");
        telemetry.addData("Y since baseline", dy + "   (" + String.format("%.2f", dy / 2000.0) + " wheel turns)");
        telemetry.update();
    }
}
