package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commands.DriveInTeleOpCommand;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Horiz;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Vertical;

@TeleOp
public class BohanTele extends CommandOpMode {
//    private Drivetrain drivetrain;
    private Shooter shooter;
    private int telemetryCounter = 0;


    @Override
    public void initialize() { //Init button on DriverHUB
        //Settings Stuff....Make sure to create a "xxx = new...." before using it to avoid nullPointerObject error
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
        GamepadEx gamepadEx2 = new GamepadEx(gamepad2);
        //Subsystems
//        drivetrain = new Drivetrain(hardwareMap);
        shooter = new Shooter(hardwareMap);

        //Commands
        shooter.setHoodServoPos(0.95);
//        drivetrain.setDefaultCommand(new DriveInTeleOpCommand(gamepad1, drivetrain)); //Change velocity multiplier in Commands
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(() -> shooter.increaseHoodDeg(0.05));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(() -> shooter.decreaseHoodDeg(0.05));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(() -> shooter.setHoodServoPos(0.95));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(() -> shooter.setHoodServoPos(0.56));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(() -> {
            shooter.shooterOn = !shooter.shooterOn; // toggle
            if (shooter.shooterOn) {
                shooter.startRPMTimer(); // Start timer when turning on
            } else {
                shooter.resetRPMTimer(); // Reset timer when turning off
            }
            shooter.updateShooter(); // actually set motor powers
        });

        //DRIVER TWO
    }


    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        telemetry.addData("HoodPosition", shooter.getHoodServoPos());
        telemetry.addData("ShooterCurrents", shooter.getMotorCurrents());
        telemetry.addData("Flywheel(RPM)", shooter.getFlyWheelRPM());
        telemetry.addData("Flywheel(rad/s)", shooter.getFlyWheelVelocity());

        telemetry.addData("Time to 6450 RPM", shooter.getTimeToTargetRPM());

        telemetry.addData("Gamepad1 Right Stick X", gamepad1.right_stick_x);
        telemetry.addData("Gamepad2 Left Stick Y", gamepad2.left_stick_y);
        telemetry.addData("Gamepad2 Right Stick Y", gamepad2.right_stick_y);
//        telemetry.addData("FL Power", drivetrain.getFrontLeftPower());
//        telemetry.addData("FR Power", drivetrain.getFrontRightPower());
//        telemetry.addData("BL Power", drivetrain.getBackLeftPower());
//        telemetry.addData("BR Power", drivetrain.getBackRightPower());
        telemetry.update();
    }
}

