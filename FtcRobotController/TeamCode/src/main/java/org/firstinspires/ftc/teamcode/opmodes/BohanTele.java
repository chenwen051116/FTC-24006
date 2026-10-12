//package org.firstinspires.ftc.teamcode.opmodes;
//
//import android.annotation.SuppressLint;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.canvas.Canvas;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.arcrobotics.ftclib.command.CommandOpMode;
//import com.arcrobotics.ftclib.command.CommandScheduler;
//import com.arcrobotics.ftclib.gamepad.GamepadEx;
//import com.arcrobotics.ftclib.gamepad.GamepadKeys;
//import com.qualcomm.hardware.lynx.LynxModule;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.commands.DriveInTeleOpCommand;
//import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
//import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
//import org.firstinspires.ftc.teamcode.subsystems.Intake;
//import org.firstinspires.ftc.teamcode.subsystems.Shooter;
//
//import java.util.List;
//
//
//@TeleOp
//public class BohanTele extends CommandOpMode {
//    private FtcDashboard dashboard;
//    private Drivetrain drivetrain;
//    private Intake intake;
//    private List<LynxModule> allHubs;
//    private Shooter shooter;
//
//    private boolean xjustpressed = false;
//    private boolean xholding = false;
//    private boolean yjustpressed = false;
//    private boolean yholding = false;
//
//    private boolean y2justpressed = false;
//    private boolean y2holding = false;
//
//    private boolean MovingshootingMode = false;
//
//    private boolean aimAssistOn = true;
//    private boolean dDownJustPressed = false;
//    private boolean dDownHolding = false;
//
//
//    @Override
//    public void initialize() { //Init button on DriverHUB
//        //Settings Stuff....Make sure to create a "xxx = new...." before using it to avoid nullPointerObject error
//        dashboard = FtcDashboard.getInstance();
//        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
//        CommandScheduler.getInstance().reset(); // drop any stale commands from previous opmode
//        allHubs = hardwareMap.getAll(LynxModule.class);
//        for (LynxModule hub : allHubs) {
//            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        }
//        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
//        GamepadEx gamepadEx2 = new GamepadEx(gamepad2);
//        //Subsystems
//        drivetrain = new Drivetrain(hardwareMap, true);
//        drivetrain.setDefaultCommand(new DriveInTeleOpCommand(gamepad1, drivetrain));
//        intake = new Intake(hardwareMap);
//        intake.setDefaultCommand(new IntakeCommand(gamepad1, intake));
//        shooter = new Shooter(hardwareMap);
//
//        // Clear any leftover autonomous state that might still be latched on hardware
//        shooter.resetTeleop();
//        intake.resetTeleop();
//
//        if (Drivetrain.TredFblue) {
//            drivetrain.redinit();
//        } else {
//            drivetrain.blueinit();
//        }
//
//        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(() -> drivetrain.blueinit());
//        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(() -> drivetrain.redinit());
//
//        //DRIVER TWO
//
//        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(() -> trimvalue(true));
//        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(() -> trimvalue(false));
//
//        gamepadEx2.getGamepadButton(GamepadKeys.Button.X).whenPressed(() -> updateMovingshooting(true));
//        gamepadEx2.getGamepadButton(GamepadKeys.Button.B).whenPressed(() -> updateMovingshooting(false));
//        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(() -> shooter.changeoffset(10));
//        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(() -> shooter.changeoffset(-10));
//
//        gamepadEx1.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).whenPressed(() -> togglesafeMode());
//    }
//
//    public void updateMovingshooting(boolean flag) {
//        MovingshootingMode = flag;
//    }
//
//    public void togglesafeMode() {
//        if (gamepad1.left_stick_button) {
//            drivetrain.safeMode = !drivetrain.safeMode;
//        }
//    }
//
//    public void trimvalue(boolean isleft) {
//        if (drivetrain.TredFblue) {
//            if (isleft) {
//                drivetrain.xposChange(-0.5);
//            } else {
//                drivetrain.xposChange(0.5);
//            }
//        } else {
//            if (isleft) {
//                drivetrain.xposChange(0.5);
//            } else {
//                drivetrain.xposChange(-0.5);
//            }
//        }
//    }
//
//    @SuppressLint("DefaultLocale")
//    @Override
//    public void run() {
//        CommandScheduler.getInstance().run();
//
//        if (gamepad2.left_trigger > 0.5) {
//            shooter.idleSpeed = 2500;
//        }
//        if (gamepad2.right_trigger > 0.5) {
//            shooter.idleSpeed = 3000;
//        }
//
//        if (gamepad1.dpad_left) {
//            Drivetrain.TredFblue = false;
//        }
//        if (gamepad1.dpad_right) {
//            Drivetrain.TredFblue = true;
//        }
//        if (gamepad1.dpad_up) {
//            drivetrain.zeroPose();
//        }
//
//        // aim assist: dpad_down toggles the master switch;
//        // the drivetrain auto-aims while the shooter is Shooting
//        if (gamepad1.dpad_down) {
//            if (!dDownHolding) {
//                dDownJustPressed = true;
//                dDownHolding = true;
//            }
//        } else {
//            dDownHolding = false;
//            dDownJustPressed = false;
//        }
//        if (dDownJustPressed) {
//            aimAssistOn = !aimAssistOn;
//            gamepad1.rumble(150);
//            dDownJustPressed = false;
//        }
//        drivetrain.aimingMode = aimAssistOn && shooter.shooterStatus == Shooter.ShooterStatus.Shooting;
//
//        // shooter <-> intake auto-feed correlation:
//        // forceShooting defaults on while Shooting, so once the flywheel
//        // reaches target RPM once (rpmreached latch), isAtTargetRPM() stays
//        // true and the intake feeds at full speed until shooting stops
//        if (shooter.shooterStatus == Shooter.ShooterStatus.Shooting) {
//            shooter.forceShooting = true;
//            intake.updateAutoshoot(true);
//            intake.updateautotranse(shooter.isAtTargetRPM()&& drivetrain.aimed());
//        } else {
//            intake.updateAutoshoot(false);
//        }
//
//        if (shooter.shooterStatus != Shooter.ShooterStatus.Stop) {
//            if (MovingshootingMode) {
//                drivetrain.ifMovingShooting = true;
//                shooter.ododis = drivetrain.getdis_TWO();
//            } else {
//                drivetrain.ifMovingShooting = false;
//                shooter.ododis = drivetrain.getdis();
//            }
//        }
//
//        shooter.forceShooting = (gamepad1.right_trigger > 0.3 && shooter.shooterStatus == Shooter.ShooterStatus.Shooting);
//
//        if (gamepad1.x || gamepad1.left_bumper) {
//            if (!xholding) {
//                xjustpressed = true;
//                xholding = true;
//            }
//        } else {
//            xholding = false;
//            xjustpressed = false;
//        }
//
//        if (gamepad1.y) {
//            if (!yholding) {
//                yjustpressed = true;
//                yholding = true;
//            }
//        } else {
//            yholding = false;
//            yjustpressed = false;
//        }
//
//        if (gamepad2.y) {
//            if (!y2holding) {
//                y2justpressed = true;
//                y2holding = true;
//            }
//        } else {
//            y2holding = false;
//            y2justpressed = false;
//        }
//
//        if (yjustpressed && shooter.shooterStatus != Shooter.ShooterStatus.Shooting) {
//            if (shooter.shooterStatus == Shooter.ShooterStatus.Idling) {
//                shooter.setShooterStatus(Shooter.ShooterStatus.Stop);
//            } else {
//                shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
//            }
//            yjustpressed = false;
//        }
//
//        if (xjustpressed) {
//            gamepad1.rumble(200);
//            if (shooter.shooterStatus == Shooter.ShooterStatus.Shooting) {
//                shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
//            } else {
//                shooter.setShooterStatus(Shooter.ShooterStatus.Shooting);
//            }
//            xjustpressed = false;
//        }
//
//        if (y2justpressed && shooter.shooterStatus != Shooter.ShooterStatus.Shooting) {
//            if (shooter.shooterStatus == Shooter.ShooterStatus.Idling) {
//                shooter.setShooterStatus(Shooter.ShooterStatus.Stop);
//            } else {
//                shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
//            }
//            y2justpressed = false;
//        }
//
//        // run the subsystem state machines (flywheel PID, updateAim,
//        // shoot bar, intake auto-feed) -- nothing runs without these
//        shooter.periodic();
//        intake.periodic();
//
//        telemetry.addData("Shooter Target RPM", shooter.getTargetRPM());
//        telemetry.addData("Shooter Current RPM", shooter.getFlyWheelRPM());
//        telemetry.addData("Pidoutput", shooter.getCurrentPIDOutput());
//        telemetry.addData("dis", drivetrain.getdis());
//        telemetry.addData("Shooterdis", shooter.ododis);
//        telemetry.addData("Looptime", drivetrain.looptime);
//        telemetry.addData("shootervel", drivetrain.getdis_TWO());
//        telemetry.addData("AutoFeed", intake.autoFeeding() ? "FEEDING" : (intake.shooterAuto ? "waiting RPM" : "off"));
//        telemetry.addData("Aim assist", aimAssistOn ? (drivetrain.aimingMode ? "AIMING" : "armed") : "OFF");
//        telemetry.addData("Aim pos", String.format("(%.2f, %.2f)", drivetrain.aimX(), drivetrain.aimY()));
//        telemetry.addData("Aim statics", String.format("(%.2f, %.2f)", Drivetrain.aimPointX, Drivetrain.aimPointY));
//        telemetry.addData("Aim err (deg)", String.format("%.1f", Math.toDegrees(drivetrain.aimError())));
//        telemetry.update();
//
//        // live pose on the FTC Dashboard field view (http://192.168.43.1:8080)
//        // Pedro field coords: origin at corner, x 0..144 along the wall, y 0..-144 into the field.
//        // Dashboard field view: origin at center, +/-72 -> shift x by -72 and flip+shift y.
//        TelemetryPacket packet = new TelemetryPacket();
//        Canvas field = packet.fieldOverlay();
//        double px = drivetrain.pose().x();
//        double py = drivetrain.pose().y();
//        double ph = drivetrain.pose().heading();
//        double dx = px - 72;
//        double dy = -py - 72;
//        field.setStroke("blue").setFill("blue");
//        field.strokeCircle(dx, dy, 9);
//        field.fillCircle(dx, dy, 3);
//        field.setStroke("black");
//        field.strokeLine(dx, dy, dx + 9 * Math.cos(ph), dy + 9 * Math.sin(ph));
//        dashboard.sendTelemetryPacket(packet);
//
//        drivetrain.period();
//
//        for (LynxModule hub : allHubs) {
//            hub.clearBulkCache();
//        }
//    }
//}
