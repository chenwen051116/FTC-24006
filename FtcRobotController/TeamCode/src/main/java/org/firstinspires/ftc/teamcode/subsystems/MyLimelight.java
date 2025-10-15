package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Math.sqrt;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

public class MyLimelight extends SubsystemBase {
    // Hardware (motor servo...)
    private final Limelight3A limelight;
    private LLResult aprilTagLatestResult;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean llenable = false;

    private Robot.Side side =  Robot.Side.Red;

    public MyLimelight(HardwareMap hardwareMap, Robot.Side side) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // fast updates
        this.side = side;

    }
    public void initPipeline(){
        if(side == Robot.Side.Blue){
            limelight.pipelineSwitch(7);
            limelight.start();
        }
        else {
            limelight.pipelineSwitch(8);
            limelight.start();
        }
    }
    public void initRedPipeline(){
        limelight.pipelineSwitch(8);
        limelight.start();
    }
    public void initPatternPipeline(){
        limelight.pipelineSwitch(9);
        limelight.start();
    }
    public double getPitch() {
        aprilTagLatestResult = limelight.getLatestResult();
        if (llenable && hasTarget()) {
            return aprilTagLatestResult.getFiducialResults().get(0)
                    .getTargetPoseCameraSpace().getOrientation().getPitch(AngleUnit.DEGREES);
        }
        return 0;
    }
    public double getX() {
        aprilTagLatestResult = limelight.getLatestResult();
        if (llenable && hasTarget()) {
            return aprilTagLatestResult.getFiducialResults().get(0)
                    .getTargetPoseCameraSpace().getPosition().x;
        }
        return 0;
    }
    public double getY() {
        aprilTagLatestResult = limelight.getLatestResult();
        if (llenable && hasTarget()) {
            return aprilTagLatestResult.getFiducialResults().get(0)
                    .getTargetPoseCameraSpace().getPosition().y;
        }
        return 0;
    }
    public double getZ() {
        aprilTagLatestResult = limelight.getLatestResult();
        if (llenable && hasTarget()) {
            return aprilTagLatestResult.getFiducialResults().get(0)
                    .getTargetPoseCameraSpace().getPosition().z;
        }
        return 0;
    }
    public double getTx() {
        aprilTagLatestResult = limelight.getLatestResult();
        if (llenable && hasTarget()) {
            return aprilTagLatestResult.getTx();
        }
        return 0;
    }
    public double getDis() {
        aprilTagLatestResult = limelight.getLatestResult();
        // Fetch most recent vision result each scheduler loop
        if (llenable && hasTarget()){
            List<LLResultTypes.FiducialResult> fiducialResults = aprilTagLatestResult.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    return sqrt(fr.getTargetPoseCameraSpace().getPosition().y * fr.getTargetPoseCameraSpace().getPosition().y
                            + (fr.getTargetPoseCameraSpace().getPosition().x) * (fr.getTargetPoseCameraSpace().getPosition().x)
                            + (fr.getTargetPoseCameraSpace().getPosition().z) * (fr.getTargetPoseCameraSpace().getPosition().z)
                    );
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
        aprilTagLatestResult = limelight.getLatestResult();
        return aprilTagLatestResult != null && aprilTagLatestResult.isValid(); //This will return true only if the data is not empty and valid
    }

    public LLResult getAprilTagResult() {
        aprilTagLatestResult = limelight.getLatestResult();
        return aprilTagLatestResult;
    }
    public int getAprilTagID() {
        aprilTagLatestResult = limelight.getLatestResult();
        return llenable && hasTarget() && !aprilTagLatestResult.getFiducialResults().isEmpty() ?
                aprilTagLatestResult.getFiducialResults().get(0).getFiducialId() : -1;
    }

}


