package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@TeleOp
public class AlbertTele extends CommandOpMode {
    private Shooter shooter;
    private Intake intake;
    private boolean xjustpressed = false;
    private boolean xholding = false;
    private boolean yjustpressed = false;
    private boolean yholding = false;
    private boolean y2justpressed = false;
    private boolean y2holding = false;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        CommandScheduler.getInstance().reset();
        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
        GamepadEx gamepadEx2 = new GamepadEx(gamepad2);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(()-> shooter.hoodNectar());
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(()-> shooter.hoodPollen());
        gamepadEx1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(()-> intake.setIntakeState(Intake.IntakeState.OUTTAKING));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(()-> intake.setIntakeState(Intake.IntakeState.INTAKING));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenReleased(
                ()-> intake.setIntakeState(Intake.IntakeState.INTAKE_STOP));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenReleased(
                ()-> intake.setIntakeState(Intake.IntakeState.INTAKE_STOP));
        shooter.resetTeleop();

    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

        if (gamepad2.left_trigger > 0.5) {
            shooter.idleSpeed = 2600;
        }
        if (gamepad2.right_trigger > 0.5) {
            shooter.idleSpeed = 3200;
        }

        if (gamepad1.x) {
            if (!xholding) {
                xjustpressed = true;
                xholding = true;
            }
        } else {
            xholding = false;
            xjustpressed = false;
        }

        if (gamepad1.y) {
            if (!yholding) {
                yjustpressed = true;
                yholding = true;
            }
        } else {
            yholding = false;
            yjustpressed = false;
        }

        if (gamepad2.y) {
            if (!y2holding) {
                y2justpressed = true;
                y2holding = true;
            }
        } else {
            y2holding = false;
            y2justpressed = false;
        }

        if (yjustpressed && shooter.shooterStatus != Shooter.ShooterStatus.Shooting) {
            if (shooter.shooterStatus == Shooter.ShooterStatus.Idling) {
                shooter.setShooterStatus(Shooter.ShooterStatus.Stop);
            } else {
                shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
            }
            yjustpressed = false;
        }

        if (xjustpressed) {
            gamepad1.rumble(200);
            if (shooter.shooterStatus == Shooter.ShooterStatus.Shooting) {
                shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
            } else {
                shooter.setShooterStatus(Shooter.ShooterStatus.Shooting);
            }
            xjustpressed = false;
        }

        if (y2justpressed && shooter.shooterStatus != Shooter.ShooterStatus.Shooting) {
            if (shooter.shooterStatus == Shooter.ShooterStatus.Idling) {
                shooter.setShooterStatus(Shooter.ShooterStatus.Stop);
            } else {
                shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
            }
            y2justpressed = false;
        }

        if (shooter.shooterStatus == Shooter.ShooterStatus.Shooting) {
            shooter.setTargetRPM(Shooter.aimRPM);
        }

        telemetry.addData("Shooter Target RPM", shooter.getTargetRPM());
        telemetry.addData("Shooter Current RPM", shooter.getFlyWheelRPM());
        telemetry.addData("PID Output", shooter.getCurrentPIDOutput());
        telemetry.update();
    }
}
