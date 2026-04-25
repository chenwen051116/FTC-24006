package org.firstinspires.ftc.teamcode.commands;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MyLimelight;

@Config
public class EndgameLockin extends CommandBase {
    Drivetrain drivetrain;
    MyLimelight limelight;
    Gamepad gamepad1;
    public static double Kp = -0.45;
    private double aimpos[] = {0.785,2.355,-0.785,-2.355};
    private double aimerror[] = {0,0,0,0};

    private double findLow(){
        double low = 10;
        double abslow = 10;
        for(int i = 0;i<4;i++){
            if(abs(aimerror[i])<abslow) {
                low = aimerror[i];
                abslow = abs(aimerror[i]);
            }
        }
        return low;
    }

    public EndgameLockin(Drivetrain drivetrain, MyLimelight limelight, Gamepad gamepad1) { //()里传参
        this.drivetrain = drivetrain; //this. = instance variable(上面的), 右面的 = ()里的
        this.limelight = limelight;
        this.gamepad1 = gamepad1;
        addRequirements(drivetrain, limelight);
    }

    @Override // Annotation, 重写super class的函数
    public void initialize() {
        gamepad1.rumble(200);
        //limelight.startDetect();
    }

    @Override
    public void execute() { // scheduler periodically calls the function
        for(int i = 0;i<4;i++){
            aimerror[i]=aimpos[i]-drivetrain.follower.getPose().getHeading();
        }
        drivetrain.teleDrive(-0.9 * gamepad1.left_stick_y, 0.9 * gamepad1.left_stick_x,
                Kp * findLow());

    }

    @Override
    public void end(boolean interrupted) {
        gamepad1.rumble(200);
       // limelight.stopDetect();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}