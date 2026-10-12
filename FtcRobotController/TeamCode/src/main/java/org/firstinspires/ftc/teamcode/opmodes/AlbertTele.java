package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.commands.DriveInTeleOpCommand;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@TeleOp
public class AlbertTele extends CommandOpMode {
    private FtcDashboard dashboard;
    private Drivetrain drivetrain;
    private Shooter shooter;
    private Intake intake;
    private boolean xjustpressed = false;
    private boolean xholding = false;
    private boolean yjustpressed = false;
    private boolean yholding = false;
    private boolean y2justpressed = false;
    private boolean y2holding = false;

    private boolean aimAssistOn = true;
    private boolean dDownJustPressed = false;
    private boolean dDownHolding = false;

    @Override
    public void initialize() {
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        CommandScheduler.getInstance().reset();
        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
        GamepadEx gamepadEx2 = new GamepadEx(gamepad2);
        drivetrain = new Drivetrain(hardwareMap, true);
        drivetrain.setDefaultCommand(new DriveInTeleOpCommand(gamepad1, drivetrain));
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        gamepadEx1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(()-> intake.setIntakeState(Intake.IntakeState.INTAKING));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(()-> intake.setIntakeState(Intake.IntakeState.OUTTAKING));
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

        // shooter <-> intake auto-feed correlation:
        // forceShooting defaults on while Shooting, so once the flywheel
        // reaches target RPM once (rpmreached latch), isAtTargetRPM() stays
        // true and the intake feeds at full speed until shooting stops
        if (shooter.shooterStatus == Shooter.ShooterStatus.Shooting) {
            shooter.forceShooting = true;
            intake.updateAutoshoot(true);
            intake.updateautotranse(shooter.isAtTargetRPM());
        } else {
            intake.updateAutoshoot(false);
        }

        // aim assist: dpad_down toggles the master switch;
        // the drivetrain auto-aims while the shooter is Shooting
        if (gamepad1.dpad_down) {
            if (!dDownHolding) {
                dDownJustPressed = true;
                dDownHolding = true;
            }
        } else {
            dDownHolding = false;
            dDownJustPressed = false;
        }
        if (dDownJustPressed) {
            aimAssistOn = !aimAssistOn;
            gamepad1.rumble(150);
            dDownJustPressed = false;
        }
        drivetrain.aimingMode = aimAssistOn && shooter.shooterStatus == Shooter.ShooterStatus.Shooting;

        if (gamepad1.dpad_up) {
            drivetrain.zeroPose();
        }

        telemetry.addData("Shooter Target RPM", shooter.getTargetRPM());
        telemetry.addData("Shooter Current RPM", shooter.getFlyWheelRPM());
        telemetry.addData("PID Output", shooter.getCurrentPIDOutput());
        telemetry.addData("x", drivetrain.pose().x());
        telemetry.addData("y", drivetrain.pose().y());
        telemetry.addData("h", drivetrain.pose().heading());
        telemetry.addData("aim dis (in)", String.format("%.1f", drivetrain.getdis3D()));
        telemetry.addData("AutoFeed", intake.autoFeeding() ? "FEEDING" : (intake.shooterAuto ? "waiting RPM" : "off"));
        telemetry.addData("Aim assist", aimAssistOn ? (drivetrain.aimingMode ? "AIMING" : "armed") : "OFF");
        telemetry.addData("Aim pos", String.format("(%.2f, %.2f)", drivetrain.aimX(), drivetrain.aimY()));
        telemetry.addData("Aim statics", String.format("(%.2f, %.2f)", Drivetrain.aimPointX, Drivetrain.aimPointY));
        telemetry.addData("Aim err (deg)", String.format("%.1f", Math.toDegrees(drivetrain.aimError())));
        telemetry.update();

        // live pose on the FTC Dashboard field view (http://192.168.43.1:8080)
        // Pedro field coords: origin at corner, x 0..144 along the wall, y 0..-144 into the field.
        // Dashboard field view: origin at center, +/-72 -> shift x by -72 and flip+shift y.
        TelemetryPacket packet = new TelemetryPacket();
        Canvas field = packet.fieldOverlay();
        double px = drivetrain.pose().x();
        double py = drivetrain.pose().y();
        double ph = drivetrain.pose().heading();
        double dx = px ;
        double dy = py ;
        field.setStroke("red").setFill("red");
        field.strokeCircle(dx, dy, 9);
        field.fillCircle(dx, dy, 3);
        field.setStroke("black");
        field.strokeLine(dx, dy, dx + 9 * Math.cos(ph), dy + 9 * Math.sin(ph));
        dashboard.sendTelemetryPacket(packet);

        drivetrain.period();
    }
}
