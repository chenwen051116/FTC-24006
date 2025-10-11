package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.google.firebase.perf.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.DriveInTeleOpCommand;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;


@Autonomous
public class PedroLimelightTest extends CommandOpMode {
    Limelight3A limelight;
    //Pedro stuff------------------------------------------------------------------------------------
    private Follower follower;
    private Timer pathTimer;
    private int pathState;
    //Poses -----------------------------------------------------------------------------------------
    private final Pose sampleStartPose = new Pose(8, 104, Math.toRadians(270));

    //Subsystems ------------------------------------------------------------------------------------
    private Drivetrain drivetrain;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
        GamepadEx gamepadEx2 = new GamepadEx(gamepad2);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.pipelineSwitch(8);//Can also be switched after init
        limelight.start(); // This tells Limelight to start looking!

        drivetrain = new Drivetrain(hardwareMap);

        drivetrain.setDefaultCommand(new DriveInTeleOpCommand(gamepad1, drivetrain));
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

        telemetry.update();
    }
}
