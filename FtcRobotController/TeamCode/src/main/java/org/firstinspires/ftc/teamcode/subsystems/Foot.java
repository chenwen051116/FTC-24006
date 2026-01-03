package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Foot extends SubsystemBase {

    private final DcMotorEx foot;

    // Same powers as your TeleOp
    public static double FOOT_UP_POWER   = 1.0;
    public static double FOOT_DOWN_POWER = -0.85;
    public static double FOOT_OFF_POWER  = 0.0;

    public enum FootMode { UP, DOWN, BRAKE }
    private FootMode mode = FootMode.BRAKE;

    // Inputs (set each loop from OpMode)
    private boolean downButton = false; // gamepad1.a
    private boolean upButton = false;   // gamepad1.b

    // Last applied power (for telemetry)
    private double footPower = FOOT_OFF_POWER;

    public Foot(HardwareMap hardwareMap) {
        foot = hardwareMap.get(DcMotorEx.class, "foot");

        // Match your TeleOp configuration
        foot.setDirection(DcMotor.Direction.REVERSE);
        foot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        foot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        foot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        setMode(FootMode.BRAKE);
    }

    /**
     * Replicates your exact button ambiguity rule:
     * if both pressed, DOWN is ignored (same as: if (footOutButton && footUpButton) footOutButton=false)
     */
    public void setButtons(boolean downPressed, boolean upPressed) {
        if (downPressed && upPressed) {
            downPressed = false;
        }
        this.downButton = downPressed;
        this.upButton = upPressed;
    }

    public void setMode(FootMode newMode) {
        mode = newMode;
        switch (mode) {
            case DOWN:
                footPower = FOOT_DOWN_POWER;
                break;
            case UP:
                footPower = FOOT_UP_POWER;
                break;
            case BRAKE:
            default:
                footPower = FOOT_OFF_POWER;
                break;
        }
        foot.setPower(footPower);
    }

    public FootMode getMode() {
        return mode;
    }

    public double getPower() {
        return footPower;
    }

    public int getPosition() {
        return foot.getCurrentPosition();
    }

    public void stop() {
        setMode(FootMode.BRAKE);
    }

    public void resetTeleop() {
        downButton = false;
        upButton = false;
        setMode(FootMode.BRAKE);
    }

    @Override
    public void periodic() {
        // EXACT same logic as your TeleOp FOOT CODE
        if (downButton) {
            setMode(FootMode.DOWN);
        } else if (upButton) {
            setMode(FootMode.UP);
        } else {
            setMode(FootMode.BRAKE);
        }
    }
}
