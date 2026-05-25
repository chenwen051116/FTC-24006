package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MyLimelight;

@Config
public class LimelightLockInCommand extends CommandBase {
    Drivetrain drivetrain;
    MyLimelight limelight;
    Gamepad gamepad1;
    public static double Kp = -0.022;
    public static double Ki = -0.0002;
    public static double Kd = -0.0022;
    public static double tole = 0.5;
    public double off = 0;
    private final PIDController pidController;

    public LimelightLockInCommand(Drivetrain drivetrain, MyLimelight limelight, Gamepad gamepad1) { //()里传参
        this.drivetrain = drivetrain; //this. = instance variable(上面的), 右面的 = ()里的
        this.limelight = limelight;
        this.gamepad1 = gamepad1;
        this.pidController = new PIDController(0.03,0,0);
        addRequirements(drivetrain, limelight);
    }

    @Override // Annotation, 重写super class的函数
    public void initialize() {
        gamepad1.rumble(200);
        limelight.startDetect();
    }
    public void addoff(double value){
        off+=value;
    }

    @Override
    public void execute() { // scheduler periodically calls the function
        pidController.setPID(Kp, Ki, Kd);
        pidController.setSetPoint(0);
        pidController.setTolerance(0.3);
        double power = pidController.calculate(limelight.getTx()-off);
        power = Math.max(-1.0, Math.min(1.0, power));
        drivetrain.teleDrive(-0.9 * gamepad1.left_stick_y, 0.9 * gamepad1.left_stick_x,
                power);

    }

    @Override
    public void end(boolean interrupted) {
        gamepad1.rumble(200);
        limelight.stopDetect();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}