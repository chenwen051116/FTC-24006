package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MyLimelight;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Config
public class ShooterCommand extends CommandBase {
    Shooter shooter;
    MyLimelight limelight;
    Gamepad gamepad1;

    public ShooterCommand(Shooter shooter, MyLimelight limelight, Gamepad gamepad1) { //()里传参
        this.shooter = shooter;
        this.limelight = limelight;
        this.gamepad1 = gamepad1;
        addRequirements(shooter, limelight);
    }

    public void initialize() {
        gamepad1.rumble(200);
        limelight.startDetect();
    }
}