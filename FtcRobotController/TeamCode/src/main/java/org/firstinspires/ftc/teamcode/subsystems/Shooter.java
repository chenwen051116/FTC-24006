package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Shooter extends SubsystemBase {

    private final DcMotorEx catapult1;
    private final DcMotorEx catapult2;

    // Same powers as your TeleOp
    public static double CATAPULT_UP_POWER   = 1.0;
    public static double CATAPULT_DOWN_POWER = -1.0;
    public static double CATAPULT_HOLD_POWER = -0.1;

    // Same timings as your TeleOp (seconds)
    public static double UP_TIME_SEC   = 0.5;
    public static double DOWN_TIME_SEC = 0.5;

    private enum CatapultMode { UP, DOWN, HOLD }
    private CatapultMode mode = CatapultMode.HOLD;

    private final ElapsedTime stageTimer = new ElapsedTime();

    // This is the "button" input (set each loop from OpMode)
    private boolean fireButtonPressed = false;

    public Shooter(HardwareMap hardwareMap) {
        catapult1 = hardwareMap.get(DcMotorEx.class, "catapult1");
        catapult2 = hardwareMap.get(DcMotorEx.class, "catapult2");

        // Match your TeleOp motor directions
        catapult1.setDirection(DcMotor.Direction.REVERSE);
        catapult2.setDirection(DcMotor.Direction.FORWARD);

        catapult1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        catapult2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Match your TeleOp modes (no encoder control)
        catapult1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        catapult2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        catapult1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        catapult2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Start stowed/held (recommended). If you truly want "no power until first fire",
        // change this to setMode(CatapultMode.HOLD, 0.0) or call stop().
        setMode(CatapultMode.HOLD);
        stageTimer.reset();
    }

    /**
     * Call this every loop from your OpMode:
     * shooter.setFireButton(gamepad1.right_bumper);
     *
     * This replicates your original behavior exactly:
     * - while pressed: force UP and keep resetting the UP timer (so it won't auto-transition)
     * - after release: UP->DOWN after UP_TIME_SEC, then DOWN->HOLD after DOWN_TIME_SEC
     */
    public void setFireButton(boolean pressed) {
        this.fireButtonPressed = pressed;
    }

    public String getModeString() {
        return mode.name();
    }

    public int getCatapult1Pos() {
        return catapult1.getCurrentPosition();
    }

    public int getCatapult2Pos() {
        return catapult2.getCurrentPosition();
    }

    public double getCatapult1Power() {
        return catapult1.getPower();
    }

    public double getCatapult2Power() {
        return catapult2.getPower();
    }

    /** Optional: clean start for TeleOp */
    public void resetTeleop() {
        fireButtonPressed = false;
        stageTimer.reset();
        setMode(CatapultMode.HOLD);
    }

    /** Optional: fully stop catapult motors */
    public void stop() {
        catapult1.setPower(0.0);
        catapult2.setPower(0.0);
    }

    @Override
    public void periodic() {
        // EXACT same logic ordering as your TeleOp:
        // 1) If button pressed -> UP, reset UP timer every loop
        // 2) Else handle timed transitions
        if (fireButtonPressed) {
            setMode(CatapultMode.UP);
            stageTimer.reset();   // holding button keeps resetting, exactly like your code
            return;
        }

        if (mode == CatapultMode.UP && stageTimer.seconds() > UP_TIME_SEC) {
            setMode(CatapultMode.DOWN);
            stageTimer.reset();
        } else if (mode == CatapultMode.DOWN && stageTimer.seconds() > DOWN_TIME_SEC) {
            setMode(CatapultMode.HOLD);
            // no reset needed (your original code stops timing after HOLD)
        }
    }

    private void setMode(CatapultMode newMode) {
        mode = newMode;
        switch (mode) {
            case UP:
                catapult1.setPower(CATAPULT_UP_POWER);
                catapult2.setPower(CATAPULT_UP_POWER);
                break;
            case DOWN:
                catapult1.setPower(CATAPULT_DOWN_POWER);
                catapult2.setPower(CATAPULT_DOWN_POWER);
                break;
            case HOLD:
            default:
                catapult1.setPower(CATAPULT_HOLD_POWER);
                catapult2.setPower(CATAPULT_HOLD_POWER);
                break;
        }
    }
}
