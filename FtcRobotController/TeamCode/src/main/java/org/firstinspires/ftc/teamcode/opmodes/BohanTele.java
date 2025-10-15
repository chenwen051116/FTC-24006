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
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commands.DriveInTeleOpCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.LimelightLockInCommand;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MyLimelight;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

import java.util.List;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class BohanTele extends LinearOpMode {
    private Robot robot;
    private final Robot.Side side = Robot.Side.Red;


        @Override
        public void runOpMode() {
            Robot robot = new Robot(hardwareMap,side);
            robot.init();
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
            GamepadEx gamepadEx2 = new GamepadEx(gamepad2);
            ToggleButtonReader autoReader = new ToggleButtonReader(
                   gamepadEx1, GamepadKeys.Button.X
            );
            waitForStart();
            while (opModeIsActive()) {
                robot.update();
                robot.teleDrive(-0.9*gamepad1.left_stick_y, 0.9*gamepad1.left_stick_x, 0.7*gamepad1.right_stick_x);
                if (gamepad1.a) {
                    robot.shooter.toggleRPM();
                }
                if(autoReader.getState()){
                    robot.setDrivetrainStatus(Drivetrain.DriveStatus.Autofocusing);
                }
                else{
                    robot.setDrivetrainStatus(Drivetrain.DriveStatus.Tele);
                }

                if (gamepad1.right_trigger > 0.3 ){
                    robot.setIntakeState(Intake.IntakeTransferState.Suck_In);
                }else if (gamepad1.left_trigger > 0.3){
                    robot.setIntakeState(Intake.IntakeTransferState.Split_Out);
                }else if (gamepad1.right_bumper) {
                    robot.setIntakeState(Intake.IntakeTransferState.Send_It_Up);
                }else {
                    robot.setIntakeState(Intake.IntakeTransferState.Intake_Steady);
                }

                telemetry.addData("Shooter Target RPM", robot.shooter.getTargetRPM());
                telemetry.addData("Shooter Current RPM", robot.shooter.getFlyWheelRPM());
                telemetry.addData("Shooter At Target", robot.shooter.isAtTargetRPM() ? "YES" : "NO");
                telemetry.addData("Gamepad1 Right Stick X", gamepad1.right_stick_x);
                telemetry.addData("Gamepad2 Left Stick Y", gamepad2.left_stick_y);
                telemetry.addData("Gamepad2 Right Stick Y", gamepad2.right_stick_y);
                telemetry.addData("Apriltag dist", robot.limelight.getDis());
                telemetry.addData("Apriltag X", robot.limelight.getX());
                telemetry.addData("Apriltag(PoI) Tx", robot.limelight.getTx());
                telemetry.addData("Apriltag ID", robot.limelight.getAprilTagID());
                telemetry.addData("Pitch", robot.limelight.getPitch());
                telemetry.addData("State", robot.drivetrain.llTx);
                telemetry.addData("State2", robot.drivetrain.drivestatus);
                telemetry.update();
        }
    }

    }


