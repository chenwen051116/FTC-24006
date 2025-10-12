package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter extends SubsystemBase {
    private final DcMotorEx shooterLeft;
    private final DcMotorEx shooterRight;
    private final PIDController pidController;
    
    // Tunable PID parameters - can be adjusted via FTC Dashboard
    public static double Kp = 0.001;  // Proportional gain
    public static double Ki = 0.0001; // Integral gain  
    public static double Kd = 0.0;    // Derivative gain
    public static double pidThreshold = 200.0; // RPM threshold for PID vs full power control
    
    // Target RPM for the flywheel
    private double targetRPM = 0.0;

    public Shooter(HardwareMap hardwareMap) {
        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
        
        // Initialize PID controller
        pidController = new PIDController(Kp, Ki, Kd);

        // Configure motors
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterLeft.setDirection(DcMotor.Direction.FORWARD);
        shooterRight.setDirection(DcMotor.Direction.REVERSE);

        // Configure motor modes - only shooterLeft has encoder
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);  // Has encoder
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // No encoder
        
        // Set PID tolerance (adjust as needed)
        pidController.setTolerance(50.0); // 50 RPM tolerance
    }
    
    /**
     * Get current flywheel velocity in rad/s
     * Uses shooterLeft (the motor with encoder) for velocity feedback
     */
    public double getFlyWheelVelocity() {
        return shooterLeft.getVelocity() * (2.0 * Math.PI) / 60.0; // Convert RPM to rad/s
    }
    
    /**
     * Get current flywheel RPM
     * Uses shooterLeft (the motor with encoder) for velocity feedback
     * shooterRight runs in open-loop mode (no encoder)
     */
    public double getFlyWheelRPM() {
        // shooterLeft has encoder, so we use its velocity as representative
        // of the entire flywheel speed (both motors should spin at same speed)
        // getVelocity() returns encoder ticks per second, convert to RPM
        return shooterLeft.getVelocity() * 60.0 / 28.0; // 28 ticks per revolution
    }
    public void setTargetRPM(double targetRPM) {
        this.targetRPM = targetRPM;
        pidController.setSetPoint(targetRPM);
    }
    public double getTargetRPM() {
        return targetRPM;
    }
    public boolean isAtTargetRPM() {
        return pidController.atSetPoint();
    }
    
    /**
     * Update PID controller and set motor powers
     * Call this method in main loop for continuous control
     */
    public void updateFlywheelPID() {
        if (targetRPM > 0) {
            double currentRPM = getFlyWheelRPM();
            double rpmDifference = targetRPM - currentRPM;
            
            double power;
            
            if (Math.abs(rpmDifference) <= pidThreshold) {
                // Use PID control for fine-tuning within ±pidThreshold RPM
                pidController.setPID(Kp, Ki, Kd);
                double output = pidController.calculate(currentRPM);
                power = Math.max(0.0, Math.min(1.0, output)); //smart brahhh
            } else if (rpmDifference > pidThreshold) {
                // Large speed increase needed - use full power
                power = 1.0;
            } else {
                // Large speed decrease needed - use no power (let inertia slow it down)
                power = 0.0;
            }
            
            // Apply power to both motors
            shooterLeft.setPower(power);
            shooterRight.setPower(power);
        } else {
            // Stop motors if no target set
            shooterLeft.setPower(0);
            shooterRight.setPower(0);
        }
    }
    
    /**
     * Set flywheel power directly (bypasses PID)
     */
    public void setFlywheelPower(double power) {
        shooterLeft.setPower(power);
        shooterRight.setPower(power);
        // Reset target when using manual power
        targetRPM = 0;
    }

    public void completeStop() {
        setFlywheelPower(0);
        pidController.reset();
    }

    public void updateTelemetry() {
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Current RPM", getFlyWheelRPM());
        telemetry.addData("At Target", isAtTargetRPM());
        telemetry.addData("Kp", Kp);
        telemetry.addData("Ki", Ki);
        telemetry.addData("Kd", Kd);
        telemetry.addData("PID Threshold", pidThreshold);
    }
}
