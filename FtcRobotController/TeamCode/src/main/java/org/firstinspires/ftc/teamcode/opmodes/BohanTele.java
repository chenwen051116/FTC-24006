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
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commands.DriveInTeleOpCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.LimelightLockInCommand;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MyLimelight;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import java.util.List;


@TeleOp
public class BohanTele extends CommandOpMode {
    private Drivetrain drivetrain;
    private Intake intake;
    private Shooter shooter;
    private MyLimelight limelight;
    private Turret turret;

    private boolean xjustpressed = false;
    private boolean xholding = false;
    private boolean yjustpressed = false;
    private boolean yholding = false;



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
        limelight = new MyLimelight(hardwareMap);
        turret = new Turret(hardwareMap);

        // Clear any leftover autonomous state that might still be latched on hardware
        shooter.automode = false;
        shooter.forceShooting = false;
        shooter.autoLonger = true;
        shooter.setShooterStatus(Shooter.ShooterStatus.Stop);

        intake.updateAutoshoot(false);
        intake.updateautotranse(false);
        intake.autoForce = false;
        intake.setIntakeState(Intake.IntakeTransferState.Intake_Steady);

        turret.automode = false;
        turret.updateAutoShoot(false);
        turret.autoForce = false;

        if(Drivetrain.TredFblue){
            limelight.initRedPipeline();
            drivetrain.redinit();
        }
        else{
            limelight.initBluePipeline();
            drivetrain.blueinit();
        }

        //Commands
        //LimelightLockInCommand limelightLock = new LimelightLockInCommand(drivetrain, limelight, gamepad1);
        //Driver One - Button A toggles RPM (0→3000→4000→5000→0)

        //gamepadEx1.getGamepadButton(GamepadKeys.Button.X).toggleWhenPressed(limelightLock);

        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(() -> limelight.initBluePipeline());
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(() -> limelight.initRedPipeline());
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(() -> drivetrain.blueinit());
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(() -> drivetrain.redinit());
        //DRIVER TWO
    }


    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        shooter.periodic();
        turret.periodic();
        limelight.periodic();
//        intake.periodic();
        if(gamepad1.dpad_left){
            drivetrain.TredFblue = false;
        }
        if(gamepad1.dpad_right){
            drivetrain.TredFblue = true;
        }
        if(gamepad1.dpad_up){
            drivetrain.originInit();
        }
        if(gamepad1.a&&gamepad1.left_bumper){
            turret.isManeulCentering = true;
            turret.centeringDir = false;
        }
        if(gamepad1.a&&gamepad1.right_bumper){
            turret.isManeulCentering = true;
            turret.centeringDir = true;
        }
        if(shooter.shooterStatus == Shooter.ShooterStatus.Shooting){
            intake.updateAutoshoot(true);
//            if(shooter.reverIntake){
//                intake.updateAutoshoot(false);
//                intake.setIntakeState(Intake.IntakeTransferState.Split_Out);
//            }
            intake.updateautotranse(shooter.isAtTargetRPM());
            shooter.updateDis(limelight.getDis());
            shooter.updateFocused(limelight.isFocused());
            //shooter.updateFocused(true);



        }
        else{
            intake.updateAutoshoot(false);

        }

        if(shooter.shooterStatus != Shooter.ShooterStatus.Stop){
            shooter.ododis = drivetrain.getdis();
            turret.updateAutoShoot(true);
            turret.tx = limelight.getTx();
            turret.aimangle = drivetrain.getturretangle();
        }
        else{
            turret.updateAutoShoot(false);
        }
        shooter.forceShooting = (gamepad1.right_trigger > 0.3 && shooter.shooterStatus == Shooter.ShooterStatus.Shooting);
        if(gamepad1.x){
            if(!xholding){
                xjustpressed = true;
                xholding = true;
          }
        }
        else{
            xholding = false;
            xjustpressed = false;
        }

        if(gamepad1.y){
            if(!yholding){
                yjustpressed = true;
                yholding = true;
            }
        }
        else{
            yholding = false;
            yjustpressed = false;
        }
        if(yjustpressed&&shooter.shooterStatus != Shooter.ShooterStatus.Shooting){
            if(shooter.shooterStatus == Shooter.ShooterStatus.Idling) {
                shooter.setShooterStatus(Shooter.ShooterStatus.Stop);
            }
            else{
                shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
            }
            yjustpressed = false;
        }
        if(xjustpressed){
            if(shooter.shooterStatus == Shooter.ShooterStatus.Shooting){
                shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
            }
            else{
                shooter.setShooterStatus(Shooter.ShooterStatus.Shooting);
            }
            xjustpressed = false;

        }


        telemetry.addData("Shooter Target RPM", shooter.getTargetRPM());
        telemetry.addData("Shooter Current RPM", shooter.getFlyWheelRPM());
        telemetry.addData("PIDoutput", turret.turretpidOut);
        telemetry.addData("Shooter At Target", shooter.isAtTargetRPM() ? "YES" : "NO");
        telemetry.addData("Gamepad1 Right Stick X", gamepad1.right_stick_x);
        telemetry.addData("Gamepad2 Left Stick Y", gamepad2.left_stick_y);
        telemetry.addData("Gamepad2 Right Stick Y", gamepad2.right_stick_y);
        telemetry.addData("Apriltag dist", limelight.getDis());
        telemetry.addData("Apriltag X", limelight.getX());
        telemetry.addData("Apriltag(PoI) Tx", limelight.getTx());
        telemetry.addData("Apriltag ID", limelight.getAprilTagID());
        telemetry.addData("Pitch", limelight.getPitch());
        telemetry.addData("Shooterdis", shooter.distance);
        telemetry.addData("tuaimanglex", turret.aimangle);
        telemetry.addData("x", drivetrain.follower.getPose().getX());
        telemetry.addData("y", drivetrain.follower.getPose().getY());
        telemetry.addData("h", drivetrain.follower.getPose().getHeading());
        telemetry.addData("dis", drivetrain.getdis());
        telemetry.addData("1_Right_Trig", gamepad1.right_trigger);
        telemetry.addData("ShotterForce?", shooter.forceShooting);
        telemetry.addData("Rpm_Range", shooter.rpmreached);


//        telemetry.addData("FL Power", drivetrain.getFrontLeftPower());
//        telemetry.addData("FR Power", drivetrain.getFrontRightPower());
//        telemetry.addData("BL Power", drivetrain.getBackLeftPower());
//        telemetry.addData("BR Power", drivetrain.getBackRightPower());
        telemetry.update();
    }
}

