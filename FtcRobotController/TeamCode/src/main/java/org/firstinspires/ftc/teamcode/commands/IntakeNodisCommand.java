package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Intake_NoDis;

public class IntakeNodisCommand extends CommandBase {
    Intake_NoDis intake;
    Gamepad gamepad1;

    public IntakeNodisCommand (Gamepad gamepad1, Intake_NoDis intake) { //()里传参
        this.intake = intake; //this. = instance variable(上面的), 右面的 = ()里的
        this.gamepad1 = gamepad1;
        addRequirements(intake);
    }

    public void execute() {
        if (gamepad1.right_trigger > 0.3 ){
            intake.setIntakeState(Intake_NoDis.IntakeTransferState.Suck_In);
        }else if (gamepad1.left_trigger > 0.3){
            intake.setIntakeState(Intake_NoDis.IntakeTransferState.Split_Out);
        }else if (gamepad1.right_bumper) {
            intake.setIntakeState(Intake_NoDis.IntakeTransferState.Send_It_Up);
        }else {
            intake.setIntakeState(Intake_NoDis.IntakeTransferState.Intake_Steady);
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.setIntakePower(0);
    }
}
