package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLFieldMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MyLimelight extends SubsystemBase {
    // Hardware (motor servo...)
    private final Limelight3A limelight;
    private LLResult aprilTagLatestResult;
    private final ElapsedTime timer = new ElapsedTime();

    public MyLimelight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // fast updates
    }

    @Override
    public void periodic() {
        // Fetch most recent vision result each scheduler loop
        aprilTagLatestResult = limelight.getLatestResult();
    }

    public boolean hasTarget() { //Has to be a METHOD instead a VARIABLE since limelight is constantly updating
        return aprilTagLatestResult != null && aprilTagLatestResult.isValid(); //This will return true only if the data is not empty and valid
    }

    public LLResult getAprilTagResult() {
        return aprilTagLatestResult;
    }

    public void switchPipeline(int index) {
        limelight.pipelineSwitch(index);
    }
}
