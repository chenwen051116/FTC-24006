package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.lynx.LynxModule;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.config.reflection.ReflectionConfig;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import java.util.LinkedHashMap;
import java.util.Map;

@Config
@TeleOp(name = "ShooterorMotorTest", group = "Test")
public class ShooterorMotorTest extends LinearOpMode {
    public static volatile boolean SHOOTER_MODE = false;
    public static volatile String MOTOR_NAME = "left"; // Set before INIT.
    public static volatile double POWER = 0; // Motor mode: -1..1; zero stops.
    public static volatile double nectarHoodPos = 0.5; // Servo position: 0.44..0.53.

    private static final double TICKS_PER_REV = 28.0;

    @Override
    public void runOpMode() throws InterruptedException {
        POWER = 0;
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.withConfigRoot(root -> root.putVariable("ShooterorMotorTest",
                ReflectionConfig.createVariableFromClass(getClass())));
        final String motorName = MOTOR_NAME;
        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();
        Servo nectarHood = hardwareMap.tryGet(Servo.class, "nectarHood");
        DcMotorEx motor = null, shooterLeft = null, shooterRight = null;
        Shooter shooter = null;
        boolean activeShooterMode = false, modeInitialized = false, modeReady = false;
        boolean previousX = false, previousY = false, previousY2 = false;
        String modeError = "";
        int xPresses = 0, yPresses = 0, y2Presses = 0;
        long previousLoopStart = System.nanoTime();
        Map<LynxModule, LynxModule.BulkCachingMode> previousBulkModes = new LinkedHashMap<>();
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            previousBulkModes.put(hub, hub.getBulkCachingMode());
        }

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry.setMsTransmissionInterval(50);
        telemetry.addLine("SHOOTER_MODE=false: set POWER and Save. B stops motor.");
        telemetry.addLine("SHOOTER_MODE=true: X toggles shooting/idle; Y or gamepad2 Y toggles idle/stop.");
        telemetry.addLine("Set Shooter.aimRPM in Dashboard. Shooter needs shooterLeft, shooterRight, shootLimit.");
        telemetry.addLine("nectarHoodPos: 0.44..0.53; applies after START in both modes.");
        telemetry.addData("Target_RPM", 0.0);
        telemetry.addData("Current_RPM", 0.0);
        telemetry.update();

        try {
            for (LynxModule hub : previousBulkModes.keySet()) {
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            }
            waitForStart();
            previousLoopStart = System.nanoTime();
            while (opModeIsActive()) {
                // Detect and remember edges from the SAME sample; never reread buttons at loop end.
                boolean currentX = gamepad1.x;
                boolean currentY = gamepad1.y;
                boolean currentY2 = gamepad2.y;
                boolean currentB = gamepad1.b;
                long loopStart = System.nanoTime();
                double loopMs = (loopStart - previousLoopStart) / 1e6;
                previousLoopStart = loopStart;
                for (LynxModule hub : previousBulkModes.keySet()) hub.clearBulkCache();
                // Clamp the Dashboard value as well as the actual servo command.
                double position = nectarHoodPos;
                double limitedPosition = Double.isFinite(position)
                        ? Math.max(0.44, Math.min(0.53, position)) : 0.5;
                if (Double.compare(position, limitedPosition) != 0) {
                    nectarHoodPos = limitedPosition;
                    dashboard.updateConfig();
                }
                if (nectarHood != null) {
                    nectarHood.setPosition(limitedPosition);
                    telemetry.addData("nectarHoodPos", limitedPosition);
                } else {
                    telemetry.addData("nectarHood", "Not configured");
                }
                boolean selectedShooterMode = SHOOTER_MODE;
                if (!modeInitialized || selectedShooterMode != activeShooterMode) {
                    // Stop the previous mode before giving the other mode control.
                    if (modeInitialized) POWER = 0;
                    if (motor != null) motor.setPower(0);
                    if (shooter != null) {
                        shooter.resetTeleop();
                        shooter.shootbarOn();
                    }
                    activeShooterMode = selectedShooterMode;
                    modeInitialized = true;
                    modeReady = false;
                    modeError = "";
                    // A held button must not start the shooter during a mode switch.
                    previousX = currentX;
                    previousY = currentY;
                    previousY2 = currentY2;
                    try {
                        if (activeShooterMode) {
                            if (shooter == null) {
                                shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
                                shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
                                shooter = new Shooter(hardwareMap) {
                                    @Override
                                    public void updateAim() {
                                        // Use the existing manual RPM entry point instead of distance.
                                        toggleRPM();
                                    }
                                };
                            }
                            shooter.resetTeleop();
                            shooter.shootbarOn();
                        } else {
                            if (motor == null) motor = hardwareMap.get(DcMotorEx.class, motorName);
                            motor.setPower(0);
                            motor.setDirection(DcMotor.Direction.FORWARD);
                            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        }
                        modeReady = true;
                    } catch (IllegalArgumentException error) {
                        modeError = error.getMessage();
                    }
                    dashboard.updateConfig();
                }

                telemetry.addData("Code_version", 9);
                telemetry.addData("Mode", activeShooterMode ? "Shooter" : "Motor");
                telemetry.addData("Loop_ms", loopMs);
                telemetry.addData("Battery_V", battery.getVoltage());
                if (!modeReady) {
                    telemetry.addData("Mode_error", modeError);
                    telemetry.addLine("Check hardware configuration, or switch SHOOTER_MODE back.");
                    telemetry.update();
                    sleep(50);
                    continue;
                }

                if (activeShooterMode) {
                    // Same X/Y controls and order as AlbertTele, without other subsystems.
                    if (currentY && !previousY) {
                        yPresses++;
                        toggleIdle(shooter);
                    }
                    if (currentX && !previousX) {
                        xPresses++;
                        gamepad1.rumble(200);
                        shooter.setShooterStatus(shooter.shooterStatus == Shooter.ShooterStatus.Shooting
                                ? Shooter.ShooterStatus.Idling : Shooter.ShooterStatus.Shooting);
                    }
                    if (currentY2 && !previousY2) {
                        y2Presses++;
                        toggleIdle(shooter);
                    }

                    shooter.forceShooting = shooter.shooterStatus == Shooter.ShooterStatus.Shooting;
                    // All state, PID and shoot-limit control stays in the existing subsystem.
                    shooter.periodic();

                    telemetry.addData("State", shooter.shooterStatus);
                    double currentRpm = shooter.getFlyWheelRPM();
                    telemetry.addData("Target_RPM", shooter.getTargetRPM());
                    telemetry.addData("Current_RPM", currentRpm);
                    telemetry.addData("RPM", currentRpm);
                    telemetry.addData("Power", shooterLeft.getPower());
                    telemetry.addData("Right_power", shooterRight.getPower());
                    telemetry.addData("Current_A", shooterLeft.getCurrent(CurrentUnit.AMPS));
                    telemetry.addData("Right_current_A", shooterRight.getCurrent(CurrentUnit.AMPS));
                    telemetry.addData("PID_Output", shooter.shooterStatus == Shooter.ShooterStatus.Stop
                            ? 0 : shooter.getCurrentPIDOutput());
                    telemetry.addData("At_target", shooter.isAtTargetRPM());
                } else {
                    if (currentB) POWER = 0;
                    double requestedPower = POWER;
                    double power = Double.isFinite(requestedPower)
                            ? Math.max(-1, Math.min(1, requestedPower)) : 0;
                    motor.setPower(power);
                    double rpm = motor.getVelocity() * 60.0 / TICKS_PER_REV;
                    double volts = battery.getVoltage();
                    telemetry.addData("Motor_name", motorName);
                    telemetry.addData("Motor_port", motor.getPortNumber());
                    telemetry.addData("Power", power);
                    telemetry.addData("Target_RPM", 0.0); // No RPM target in direct-power motor mode.
                    telemetry.addData("Current_RPM", rpm);
                    telemetry.addData("RPM", rpm);
                    telemetry.addData("Current_A", motor.getCurrent(CurrentUnit.AMPS));
                    telemetry.addData("RPM_at_12V_approx", volts > 0 ? rpm * 12.0 / volts : 0);
                }
                previousX = currentX;
                previousY = currentY;
                previousY2 = currentY2;
                telemetry.addData("X_presses", xPresses);
                telemetry.addData("Y_presses", yPresses);
                telemetry.addData("Y2_presses", y2Presses);
                telemetry.update();
                sleep(activeShooterMode ? 1 : 50);
            }
        } finally {
            POWER = 0;
            if (motor != null) motor.setPower(0);
            if (shooter != null) {
                shooter.resetTeleop();
                shooter.shootbarOn();
            }
            for (Map.Entry<LynxModule, LynxModule.BulkCachingMode> entry : previousBulkModes.entrySet()) {
                entry.getKey().setBulkCachingMode(entry.getValue());
            }
            dashboard.updateConfig();
        }
    }

    private void toggleIdle(Shooter shooter) {
        if (shooter.shooterStatus != Shooter.ShooterStatus.Shooting) {
            shooter.setShooterStatus(shooter.shooterStatus == Shooter.ShooterStatus.Idling
                    ? Shooter.ShooterStatus.Stop : Shooter.ShooterStatus.Idling);
        }
    }
}
