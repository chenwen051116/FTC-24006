package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class IntakeCommand extends CommandBase {
    Intake intake;
    Gamepad gamepad1;

    public IntakeCommand (Gamepad gamepad1, Intake intake) { //()里传参
        this.intake = intake; //this. = instance variable(上面的), 右面的 = ()里的
        this.gamepad1 = gamepad1;
        addRequirements(intake);
    }

    public void execute() {
        if (gamepad1.right_trigger > 0.3 ){
        intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
            intake.autoForce = false;

    }else if (gamepad1.left_trigger > 0.3){
            intake.setIntakeState(Intake.IntakeTransferState.Split_Out);
            intake.autoForce = false;
    }else if (gamepad1.right_bumper) {
            intake.setIntakeState(Intake.IntakeTransferState.Send_It_Up);
            intake.autoForce = true;
    }else {
            intake.setIntakeState(Intake.IntakeTransferState.Intake_Steady);
            intake.autoForce = false;
    }
    }

    @Override
    public void end(boolean interrupted) {
        intake.setIntakePower(0);
    }
}
