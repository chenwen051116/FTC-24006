package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MyLimelight;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
public class Robot {
    public Drivetrain drivetrain;
    public Intake intake;
    public Shooter shooter;
    public MyLimelight limelight;

    public enum Side {
        Red, Blue
    }

    public void init(){
        limelight.initPipeline();
        limelight.startDetect();
    }

    public Robot(HardwareMap hardwareMap, Side side){
        drivetrain = new Drivetrain(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        limelight = new MyLimelight(hardwareMap, side);
    }

    public void update(){
        shooter.periodic();
        if(shooter.shooterStatus == Shooter.ShooterStatus.Shooting){
            shooter.updateDis(limelight.getDis());
        }
        if(drivetrain.drivestatus == Drivetrain.DriveStatus.Autofocusing){
            drivetrain.updatellTx(limelight.getTx());
        }
    }

    public void setIntakeState(Intake.IntakeTransferState state){
        intake.setIntakeState(state);
    }

    public void setShooterState(Shooter.ShooterStatus state){
        shooter.setShooterStatus(state);
    }

    public void setDrivetrainStatus(Drivetrain.DriveStatus state) {
        drivetrain.setDriveStatus(state);
    }

    public void teleDrive(double x, double y, double z){
        drivetrain.teleDrive(x, y, z);
    }







}
