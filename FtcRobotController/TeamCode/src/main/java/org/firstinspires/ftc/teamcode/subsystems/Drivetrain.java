package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Drivetrain extends SubsystemBase {

    private final DcMotorEx leftFrontDrive;
    private final DcMotorEx leftBackDrive;
    private final DcMotorEx rightFrontDrive;
    private final DcMotorEx rightBackDrive;

    // Optional overall scaling (tunable on Dashboard)
    public static double DRIVE_SCALE = 1.0;

    // Store latest requested drive inputs (so periodic() can apply them)
    private double axialCmd = 0.0;
    private double lateralCmd = 0.0;
    private double yawCmd = 0.0;

    // Last computed motor powers (useful for telemetry)
    private double leftFrontPower = 0.0;
    private double rightFrontPower = 0.0;
    private double leftBackPower = 0.0;
    private double rightBackPower = 0.0;

    public Drivetrain(HardwareMap hardwareMap) {
        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, "left_front_drive");
        leftBackDrive   = hardwareMap.get(DcMotorEx.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "right_front_drive");
        rightBackDrive  = hardwareMap.get(DcMotorEx.class, "right_back_drive");

        // Match your TeleOp motor directions exactly
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Match your TeleOp BRAKE behavior
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Match your TeleOp modes
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        stop();
    }

    /**
     * Set drive commands in the SAME meaning as your original code:
     * leftFront = axial + lateral + yaw
     * rightFront = axial - lateral - yaw
     * leftBack = axial - lateral + yaw
     * rightBack = axial + lateral - yaw
     */
    public void setDriveCommands(double axial, double lateral, double yaw) {
        this.axialCmd = axial;
        this.lateralCmd = lateral;
        this.yawCmd = yaw;
    }

    /**
     * Convenience method that matches your EXACT gamepad mapping from the LinearOpMode:
     *
     * double yaw     = gamepad1.left_stick_y;
     * double axial   = -gamepad1.right_stick_x;
     * double lateral = -gamepad1.left_stick_x;
     */
    public void setFromGamepad(double leftStickY, double leftStickX, double rightStickX) {
        double yaw = leftStickY;
        double axial = -rightStickX;
        double lateral = -leftStickX;
        setDriveCommands(axial, lateral, yaw);
    }

    public void stop() {
        axialCmd = 0.0;
        lateralCmd = 0.0;
        yawCmd = 0.0;

        leftFrontDrive.setPower(0.0);
        rightFrontDrive.setPower(0.0);
        leftBackDrive.setPower(0.0);
        rightBackDrive.setPower(0.0);

        leftFrontPower = rightFrontPower = leftBackPower = rightBackPower = 0.0;
    }

    public double getLeftFrontPower()  { return leftFrontPower; }
    public double getRightFrontPower() { return rightFrontPower; }
    public double getLeftBackPower()   { return leftBackPower; }
    public double getRightBackPower()  { return rightBackPower; }

    @Override
    public void periodic() {
        // Compute powers exactly like your TeleOp
        leftFrontPower  = axialCmd + lateralCmd + yawCmd;
        rightFrontPower = axialCmd - lateralCmd - yawCmd;
        leftBackPower   = axialCmd - lateralCmd + yawCmd;
        rightBackPower  = axialCmd + lateralCmd - yawCmd;

        // Normalize exactly like your TeleOp
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        // Optional scaling (still preserves normalization behavior)
        leftFrontPower  *= DRIVE_SCALE;
        rightFrontPower *= DRIVE_SCALE;
        leftBackPower   *= DRIVE_SCALE;
        rightBackPower  *= DRIVE_SCALE;

        // Apply
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }
}
