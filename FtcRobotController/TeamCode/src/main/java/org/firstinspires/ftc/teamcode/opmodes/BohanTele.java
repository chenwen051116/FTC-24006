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
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;


@TeleOp
public class BohanTele extends CommandOpMode {
    private Drivetrain drivetrain;
    private Intake intake;
    private Shooter shooter;
    @Override
    public void initialize() { //Init button on DriverHUB
        //Settings Stuff....Make sure to create a "xxx = new...." before using it to avoid nullPointerObject error
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
        GamepadEx gamepadEx2 = new GamepadEx(gamepad2);
        //Subsystems
        drivetrain = new Drivetrain(hardwareMap);
        drivetrain.setDefaultCommand(new DriveInTeleOpCommand(gamepad1, drivetrain));
        intake = new Intake(hardwareMap);
        intake.setDefaultCommand(new IntakeCommand(gamepad1, intake));
        shooter = new Shooter(hardwareMap);
        //Driver One - Button A toggles RPM (0→3000→4000→5000→0)
        gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(shooter::toggleRPM);
        //DRIVER TWO
    }


    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        
        // Update shooter PID
        shooter.updateFlywheelPID();
        
        telemetry.addData("Distance", intake.getDist());
        telemetry.addData("Shooter Target RPM", shooter.getTargetRPM());
        telemetry.addData("Shooter Current RPM", shooter.getFlyWheelRPM());
        telemetry.addData("Shooter At Target", shooter.isAtTargetRPM() ? "YES" : "NO");
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

