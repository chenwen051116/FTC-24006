package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

public class DriveInTeleOpCommand extends CommandBase {
    Drivetrain drivetrain;
    Gamepad gamepad1;

    public DriveInTeleOpCommand (Gamepad gamepad1, Drivetrain drivetrain) { //()里传参
        this.drivetrain = drivetrain; //this. = instance variable(上面的), 右面的 = ()里的
        this.gamepad1 = gamepad1;
        addRequirements(drivetrain);
    }

    @Override //Annotation, 重写super class的函数
    public void initialize() {}

    @Override
    public void execute() {
        //scheduler periodically calls the function
        if(!gamepad1.b) {
            drivetrain.teleDrive(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));

        }
        else{
            drivetrain.teleDrive(new PoseVelocity2d(
                    new Vector2d(
                            -Drivetrain.testspeedy,
                            -Drivetrain.testspeedx
                    ),
                    -Drivetrain.testspeedrx
            ));
        }
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {

        return false;
    }
}
