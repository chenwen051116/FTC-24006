package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;

import java.util.List;

/**
 * Pre-match prep: configures the Limelight for the RED alliance.
 * Uploads the red Hive field map (tags 30-37) and verifies tags/pose.
 */
@TeleOp
public class TurnRed extends OpMode {
    private Limelight limelight;
    private boolean uploaded = false;

    @Override
    public void init() {
        limelight = new Limelight(hardwareMap);
        uploaded = limelight.uploadHiveFieldMap(false);
        telemetry.addData("Field map", uploaded ? "RED uploaded ok" : "RED upload FAILED");
        telemetry.addLine("Tags below should be 30-37. If you see 38-45, run TurnBlue instead.");
    }

    @Override
    public void loop() {
        List<Integer> ids = limelight.getFiducialIds();
        telemetry.addData("Field map", uploaded ? "RED uploaded ok" : "RED upload FAILED");
        telemetry.addData("LL tags visible", "%d ids %s", ids.size(), ids.toString());
        Pose3D mt1 = limelight.getBotposeMt1Raw();
        telemetry.addData("LL MT1 (m)", mt1 == null ? "none"
                : String.format("%.3f, %.3f, %.3f",
                        mt1.getPosition().x, mt1.getPosition().y, mt1.getPosition().z));
        telemetry.addLine("Robot is ready for RED. Stop this and start your match opmode.");
    }
}
