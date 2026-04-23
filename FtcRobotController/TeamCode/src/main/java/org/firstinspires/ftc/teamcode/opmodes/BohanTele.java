package org.firstinspires.ftc.teamcode.opmodes;

import android.annotation.SuppressLint;

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
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.commands.DriveInTeleOpCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.LimelightLockInCommand;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Light;
import org.firstinspires.ftc.teamcode.subsystems.MyLimelight;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import java.util.List;


@TeleOp
public class BohanTele extends CommandOpMode {
    private Drivetrain drivetrain;
    private Intake intake;
    private List<LynxModule> allHubs;
    private Shooter shooter;
    private MyLimelight limelight;
    private Turret turret;

    private Light light;
    private boolean xjustpressed = false;
    private boolean xholding = false;
    private boolean yjustpressed = false;
    private boolean yholding = false;

    private boolean y2justpressed = false;
    private boolean y2holding = false;

    private boolean MovingshootingMode = false;





    @Override
    public void initialize() { //Init button on DriverHUB
        //Settings Stuff....Make sure to create a "xxx = new...." before using it to avoid nullPointerObject error
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        CommandScheduler.getInstance().reset(); // drop any stale commands from previous opmode
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
        GamepadEx gamepadEx2 = new GamepadEx(gamepad2);
        //Subsystems
        drivetrain = new Drivetrain(hardwareMap,true);
        drivetrain.setDefaultCommand(new DriveInTeleOpCommand(gamepad1, drivetrain));
        intake = new Intake(hardwareMap);
        intake.setDefaultCommand(new IntakeCommand(gamepad1, intake));
        shooter = new Shooter(hardwareMap);
        limelight = new MyLimelight(hardwareMap);
        turret = new Turret(hardwareMap,true);
        light = new Light(hardwareMap);

        // Clear any leftover autonomous state that might still be latched on hardware
        shooter.resetTeleop();
        intake.resetTeleop();
        turret.resetTeleop();

        if(Drivetrain.TredFblue){
            limelight.initPatternPipeline();
            drivetrain.redinit();
        }
        else{
            limelight.initPatternPipeline();
            drivetrain.blueinit();
        }

        //Commands
        LimelightLockInCommand limelightLock = new LimelightLockInCommand(drivetrain, limelight, gamepad1);
        //Driver One - Button A toggles RPM (0→3000→4000→5000→0)

        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).toggleWhenPressed(limelightLock);
//        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(() -> limelight.initBluePipeline());
//        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(() -> limelight.initRedPipeline());
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(() -> drivetrain.blueinit());
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(() -> drivetrain.redinit());
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenReleased(()->light.setLight(Light.Color.Off, Light.Color.Off));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whileHeld(()->light.setLight(Light.Color.Green, Light.Color.Green));

        //DRIVER TWO
        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(()->drivetrain.xposChange(-0.5));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(()->drivetrain.xposChange(0.5));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whileHeld(()->light.setLight(Light.Color.Off, Light.Color.Orange));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whileHeld(()->light.setLight(Light.Color.Orange, Light.Color.Off));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenReleased(()->light.setLight(Light.Color.Off, Light.Color.Off));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenReleased(()->light.setLight(Light.Color.Off, Light.Color.Off));
//        gamepadEx2.getGamepadButton(GamepadKeys.Button.A).whenPressed(() -> limelight.initLocalizePipeline());

        gamepadEx2.getGamepadButton(GamepadKeys.Button.X).whenPressed(()->updateMovingshooting(true));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.B).whenPressed(()->updateMovingshooting(false));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.B).whenPressed(()->light.setLight(Light.Color.Off, Light.Color.Off));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(()->shooter.changeoffset(10));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(()->shooter.changeoffset(-10));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_UP).whileHeld(()->light.setLight(Light.Color.Red, Light.Color.Red));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whileHeld(()->light.setLight(Light.Color.Blue, Light.Color.Blue));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenReleased(()->light.setLight(Light.Color.Off, Light.Color.Off));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenReleased(()->light.setLight(Light.Color.Off, Light.Color.Off));

        gamepadEx2.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).whenPressed(()->togglesafeMode());
    }
    public void updateMovingshooting(boolean flag){
        MovingshootingMode = flag;
    }

public void togglesafeMode(){
    if(gamepad2.left_stick_button){
        drivetrain.safeMode = !drivetrain.safeMode;
        if(drivetrain.safeMode){
            light.setLight(Light.Color.Red, Light.Color.Red);
        }
        else{
            light.setLight(Light.Color.Off, Light.Color.Off);
        }
    }
}



    @SuppressLint("DefaultLocale")
    @Override
    public void run() {
        CommandScheduler.getInstance().run();

        limelight.llheading  = drivetrain.follower.getHeading()/3.14*180;

        if(gamepad2.left_trigger>0.5){
            shooter.idleSpeed = 2500;
        }
        if(gamepad2.right_trigger>0.5){
            shooter.idleSpeed = 3000;
        }

        if(MovingshootingMode){
            light.setLight(Light.Color.Violet,Light.Color.Violet);
        }
        if(gamepad1.dpad_left){
            Drivetrain.TredFblue = false;
        }
        if(gamepad1.dpad_right){
            Drivetrain.TredFblue = true;
        }
        if(gamepad1.dpad_up){
            drivetrain.originInit();
        }

        if(shooter.shooterStatus == Shooter.ShooterStatus.Shooting){
            intake.FarTeleTransFactor = shooter.transFactor;
            shooter.isfocused = turret.isfocuedTu();
            intake.updateAutoshoot(true);
           // intake.isFarTeleMode = shooter.isAtFar();
//            if(shooter.reverIntake){
//                intake.updateAutoshoot(false);
//                intake.setIntakeState(Intake.IntakeTransferState.Split_Out);
//            }
            intake.updateautotranse(shooter.isAtTargetRPM());
//            shooter.updateDis(limelight.getDis());
//            shooter.updateFocused(limelight.isFocused());
            //shooter.updateFocused(true);



        }
        else{
            intake.updateAutoshoot(false);

        }

        if(shooter.shooterStatus != Shooter.ShooterStatus.Stop){
            if(MovingshootingMode) {
                drivetrain.ifMovingShooting = true;
                shooter.ododis = drivetrain.getdis_TWO();
                turret.aimangle = drivetrain.getturretangle_TWO();
            }
            else{
                drivetrain.ifMovingShooting = false;
                shooter.ododis = drivetrain.getdis();
                turret.aimangle = drivetrain.getturretangle();
            }
            turret.updateAutoShoot(true);
            //turret.tx = limelight.getTx();

        }
        else{
            turret.updateAutoShoot(false);
        }
        shooter.forceShooting = (gamepad1.right_trigger > 0.3 && shooter.shooterStatus == Shooter.ShooterStatus.Shooting);
        if(gamepad1.x||gamepad1.left_bumper){
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

        if(gamepad2.y){
            if(!y2holding){
                y2justpressed = true;
                y2holding = true;
            }
        }
        else{
            y2holding = false;
            y2justpressed = false;
        }
        if(yjustpressed&&shooter.shooterStatus != Shooter.ShooterStatus.Shooting){
           // shooter.idleSpeed = 2600;
            if(shooter.shooterStatus == Shooter.ShooterStatus.Idling) {
                shooter.setShooterStatus(Shooter.ShooterStatus.Stop);
            }
            else{
                shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
            }
            yjustpressed = false;
        }
        if(xjustpressed){
          //  shooter.idleSpeed = 2600;
            gamepad1.rumble(200);
            if(shooter.shooterStatus == Shooter.ShooterStatus.Shooting){
                shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
            }
            else{
                shooter.setShooterStatus(Shooter.ShooterStatus.Shooting);
            }
            xjustpressed = false;

        }

        if(y2justpressed&&shooter.shooterStatus != Shooter.ShooterStatus.Shooting){
            if(shooter.shooterStatus == Shooter.ShooterStatus.Idling) {
                shooter.setShooterStatus(Shooter.ShooterStatus.Stop);
            }
            else{
                shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
            }
            y2justpressed = false;
        }

        if(gamepad1.left_stick_button&& gamepad1.right_stick_button){
            drivetrain.tilt();
        }
        telemetry.addData("Shooter Target RPM", shooter.getTargetRPM());
        telemetry.addData("Shooter Current RPM", shooter.getFlyWheelRPM());
          telemetry.addData("Pidoutput", shooter.getCurrentPIDOutput());
        telemetry.addData("dis", drivetrain.getdis());
//        telemetry.addData("maglim", turret.magLim.getState());
////        telemetry.addData("Omega", drivetrain.angularVel());
////        telemetry.addData("speed over all",drivetrain.getallspeed());
////        telemetry.addData("speed towards",drivetrain.forwardvel());
////        telemetry.addData("x", drivetrain.follower.getPose().getX());
////        telemetry.addData("y", drivetrain.follower.getPose().getY());
////        telemetry.addData("h", drivetrain.follower.getPose().getHeading());
        telemetry.addData("PIDoutput", turret.turretpidOut);
////        telemetry.addData("Shooter At Target", shooter.isAtTargetRPM() ? "YES" : "NO");
////        telemetry.addData("Gamepad1 Right Stick X", gamepad1.right_stick_x);
////        telemetry.addData("Gamepad2 Left Stick Y", gamepad2.left_stick_y);
//////        telemetry.addData("Gamepad2 Right Stick Y", gamepad2.right_stick_y);
//////        telemetry.addData("Apriltag dist", limelight.getDis());
//        telemetry.addData("Apriltag X", limelight.getpatterTx());
//////        telemetry.addData("Apriltag(PoI) Tx", limelight.getTx());
//////        telemetry.addData("Apriltag ID", limelight.getAprilTagID());
//////        telemetry.addData("Pitch", limelight.getPitch());
//        telemetry.addData("Shooterdis", shooter.ododis);
////        telemetry.addData("DRIVETRAIN GIVE ANGLE", drivetrain.getturretangle());
        telemetry.addData("turretaimpos", turret.aimposition);
//////        telemetry.addData("1_Right_Trig", gamepad1.right_trigger);
//////        telemetry.addData("ShotterForce?", shooter.forceShooting);
//       telemetry.addData("accel", drivetrain.angularVelnum);
        telemetry.addData("pidoutput", turret.output);
       telemetry.addData("trans1ball", intake.hasballCheck(1));
        telemetry.addData("trans2ball", intake.hasballCheck(2));
        telemetry.addData("trans3ball", intake.hasballCheck(3));

        telemetry.addData("trans1ballraw", intake.hasball1sum);
        telemetry.addData("trans2ballraw", intake.hasball2sum);
        telemetry.addData("trans3ballrawr", intake.hasball3sum);
        telemetry.addData("Looptime", drivetrain.looptime);
        telemetry.addData("shootervel", drivetrain.getdis_TWO());
        telemetry.addData("turret",drivetrain.getturretangle_TWO());
        telemetry.update();
        drivetrain.period();

        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }
}

