package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLFieldMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

public class MyLimelight extends SubsystemBase {
    // Hardware (motor servo...)
    private final Limelight3A limelight;
    private LLResult aprilTagLatestResult;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean llenable = false;

    public MyLimelight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // fast updates

    }

    public void initRvision(){
        limelight.pipelineSwitch(8);
        limelight.start();
    }
    public void initBvision(){
        limelight.pipelineSwitch(7);
        limelight.start();
    }
    public void initPatvision(){
        limelight.pipelineSwitch(9);
        limelight.start();
    }

    public double returnDis() {
        // Fetch most recent vision result each scheduler loop
        if (llenable){
            aprilTagLatestResult = limelight.getLatestResult();
            List<LLResultTypes.FiducialResult> fiducialResults = aprilTagLatestResult.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    return fr.getRobotPoseTargetSpace().getPosition().y;
            }
        }
        return 0;
    }

    public void startDetect(){
        llenable = true;
    }
    public void stopDetect(){
        llenable = false;
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
    public int getAprilTagID() {
        return hasTarget() && !aprilTagLatestResult.getFiducialResults().isEmpty() ?
                aprilTagLatestResult.getFiducialResults().get(0).getFiducialId() : -1;
    }
}
