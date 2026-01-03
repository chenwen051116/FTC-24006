package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
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
    public void teleDrive (double frontBackVelocity, double strafeVelocity, double turnVelocity){
        double y = frontBackVelocity;
        double x = strafeVelocity;
        double rx = turnVelocity;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        leftFrontDrive.setPower(frontLeftPower);
        rightFrontDrive.setPower(frontRightPower);
        leftBackDrive.setPower(backLeftPower);
        rightBackDrive.setPower(backRightPower);
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

    }
}
