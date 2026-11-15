package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;

import java.util.List;

/**
 * Pre-match prep: configures the Limelight for the BLUE alliance.
 * Uploads the blue Hive field map (tags 38-45) and verifies tags/pose.
 */
@TeleOp
public class TurnBlue extends OpMode {
    private Limelight limelight;
    private boolean uploaded = false;

    @Override
    public void init() {
        limelight = new Limelight(hardwareMap);
        uploaded = limelight.uploadHiveFieldMap(true);
        telemetry.addData("Field map", uploaded ? "BLUE uploaded ok" : "BLUE upload FAILED");
        telemetry.addLine("Tags below should be 38-45. If you see 30-37, run TurnRed instead.");
    }

    @Override
    public void loop() {
        List<Integer> ids = limelight.getFiducialIds();
        telemetry.addData("Field map", uploaded ? "BLUE uploaded ok" : "BLUE upload FAILED");
        telemetry.addData("LL tags visible", "%d ids %s", ids.size(), ids.toString());
        Pose3D mt1 = limelight.getBotposeMt1Raw();
        telemetry.addData("LL MT1 (m)", mt1 == null ? "none"
                : String.format("%.3f, %.3f, %.3f",
                        mt1.getPosition().x, mt1.getPosition().y, mt1.getPosition().z));
        telemetry.addLine("Robot is ready for BLUE. Stop this and start your match opmode.");
    }
}
