package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake extends SubsystemBase {
    private final DcMotor intakeMotor;
    private final Servo intakePivot;
    private boolean intakeHasReset = false;

    private Intake.IntakeState currentState = IntakeState.INTAKE_RESET;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        intakePivot = hardwareMap.get(Servo.class, "intakePivot");

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void periodic() {
        if (!intakeHasReset) {
            setIntakeState(IntakeState.INTAKE_RESET);
            intakeHasReset = true;
        }
    }

    public enum IntakeState {
        INTAKING(1, 1),
        OUTTAKING(0.8, -0.4),
        PREINTAKE(0.8, 0),
        INTAKE_RESET(0.57, 0);

        private final double intakePivotPosition;
        private final double intakeMotorPower;

        IntakeState(double intakePivotPosition, double intakeMotorPower) {
            this.intakePivotPosition = intakePivotPosition;
            this.intakeMotorPower = intakeMotorPower;
        }

    }

    public void setIntakeState(IntakeState intakeState) {
        currentState = intakeState;
        intakeMotor.setPower(intakeState.intakeMotorPower);
        intakePivot.setPosition(intakeState.intakePivotPosition);
    }


    public IntakeState getCurrentState() {
        return currentState;
    }


}
