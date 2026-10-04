package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import static java.lang.Math.abs;
import static java.lang.Math.floor;

import com.acmerobotics.dashboard.config.Config;
// Pedro 3 no longer provides this unused timer.
// import com.pedropathing.util.Timer;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class Intake extends SubsystemBase {
    private final DcMotorEx intakeL;
    private final DcMotorEx intakeR;
    public Intake(HardwareMap hardwareMap) {
       intakeL = hardwareMap.get(DcMotorEx.class, "intakeL");
       intakeR = hardwareMap.get(DcMotorEx.class, "intakeR");
    }
    public enum IntakeState {
        INTAKING( 1),
        INTAKE_STOP(0),
        OUTTAKING( -0.4);
        private final double intakeMotorPower;

        IntakeState( double intakeMotorPower) {
            this.intakeMotorPower = intakeMotorPower;
        }
    }
    public void setIntakeState(IntakeState intakeState) {
        intakeL.setPower(intakeState.intakeMotorPower);
        intakeR.setPower(intakeState.intakeMotorPower);
    }
    /**
     * Get current flywheel velocity in rad/s
     * Uses shooterLeft (the motor with encoder) for velocity feedback
     */

    @Override
    public void periodic(){

    }

}
