package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import static java.lang.Math.abs;

import android.health.connect.datatypes.units.Power;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class Shooter extends SubsystemBase {
    private final DcMotorEx shooterLeft;
    private final DcMotorEx shooterRight;
    private final PIDController pidController;

    // Tunable PID parameters - can be adjusted via FTC Dashboard
    public static double Kp = 0.006;  // Proportional gain
    public static double Ki = 0.0; // Integral gain
    public static double Kd = 0.0;    // Derivative gain
    public static double pidThreshold = 1000.0; // RPM threshold for PID vs full power control
    public static double tolerance = 0.3; // RPM tolerance for "at target" determination

    public static double aimRPM = 0;

    // Target RPM for the flywheel
    private double targetRPM = 0.0;

    public double distance = 0;

    public boolean idelOn = false;

    public boolean focused = false;

    public boolean automode = false;

    public boolean autoLonger = true;

    public double PIDoutput;



    public enum ShooterStatus {
        Stop,Idling,Shooting
    }



    public ShooterStatus shooterStatus = ShooterStatus.Stop;



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
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // No encoder

        // Set PID tolerance (adjustable via static parameter)
        pidController.setTolerance(tolerance);
    }

    /**
     * Get current flywheel velocity in rad/s
     * Uses shooterLeft (the motor with encoder) for velocity feedback
     */
    public void updateFocused(boolean focus){
        focused = focus;
    }
    public void setShooterStatus(ShooterStatus status){
        shooterStatus = status;
    }
    public double getFlyWheelVelocity() {
        return shooterLeft.getVelocity() * (2.0 * Math.PI) / 60.0; // Convert RPM to rad/s

    }

    public void updateDis(double dis){
        distance = dis;
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
        pidController.setSetPoint(0);


    }
    public double getTargetRPM() {
        return targetRPM;
    }
    public boolean isAtTargetRPM() {
        return (getTargetRPM() < getFlyWheelRPM() + 50 && getTargetRPM() > getFlyWheelRPM() - 50);
    }

    // Store current motor power for telemetry/graphing
    private double currentMotorPower = 0.0;
    private double currentPIDOutput = 0.0;

    /**
     * Update PID controller and set motor powers
     * Call this method in main loop for continuous control
     */
    public void settoShooting(){
        shooterStatus = ShooterStatus.Shooting;
    }
    public void settoStop(){
        shooterStatus = ShooterStatus.Stop;
    }


    public void settoIdle(){
        shooterStatus = ShooterStatus.Idling;
    }
    public void updateFlywheelPID() {
        shooterLeft.setVelocity(targetRPM*60.0, AngleUnit.DEGREES);
        shooterRight.setVelocity(targetRPM*60.0, AngleUnit.DEGREES);
//        if (targetRPM > 0) {
//            // Update PID parameters and tolerance in case they were changed via dashboard
//            pidController.setPID(Kp, Ki, Kd);
//            pidController.setTolerance(tolerance);
//
//            double currentRPM = getFlyWheelRPM();
//            double rpmDifference = currentRPM - targetRPM;
//
//            double pidinput = rpmDifference/100.0;
//            double power;
//            double pidOutput = 0.0;
//
//            if (abs(rpmDifference) <= pidThreshold) {
//                // Use PID control for fine-tuning within ±pidThreshold RPM
//                pidOutput = pidController.calculate(pidinput)+0.5;
//                power = Math.max(0.0, Math.min(1.0, pidOutput)); //smart brahhh
//            } else if (rpmDifference < pidThreshold) {
//                // Large speed increase needed - use full power
//                power = 1.0;
//                pidOutput = 1.0; // PID would output 1.0 but we're overriding
//            } else {
//                // Large speed decrease needed - use no power (let inertia slow it down)
//                power = 0.0;
//                pidOutput = 0.0; // PID would output negative but we're overriding
//            }
//            PIDoutput = power;
//            // Store values for telemetry/graphing
//            currentMotorPower = power;
//            currentPIDOutput = pidOutput;
//
//            // Apply power to both motors
//            shooterLeft.setPower(power);
//            shooterRight.setPower(power);
//        } else {
//            // Stop motors if no target set
//            currentMotorPower = 0.0;
//            currentPIDOutput = 0.0;
//            shooterLeft.setPower(0);
//            shooterRight.setPower(0);
//        }
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

    public void toggleRPM() {
        setTargetRPM(aimRPM);
        shooterStatus = ShooterStatus.Shooting;

    }

    public void updateAim() {
        distance = abs(distance);
        if (distance > 3.25){
            setTargetRPM(3850);
        }
        else if (distance < 1.5){
            setTargetRPM(200*distance+2750);
        }
        else{
            setTargetRPM(300*distance+2750);
        }

        if (distance < 0.01){
            setTargetRPM(3500);
        }

        if(automode&&autoLonger){
            setTargetRPM(3500);
        }
        else if(automode&&!autoLonger){
            setTargetRPM(3100);
        }
    }




    /**
     * Get current motor power (for graphing/telemetry)
     */
    public double getCurrentMotorPower() {
        return currentMotorPower;
    }

    /**
     * Get current PID output (for graphing/telemetry)
     */
    public double getCurrentPIDOutput() {
        return currentPIDOutput;
    }
    @Override
    public void periodic(){
        updateFlywheelPID();
        if(shooterStatus == ShooterStatus.Shooting && focused){
            updateAim();
        }
        else if(shooterStatus == ShooterStatus.Stop){
            completeStop();
        }
        else if(shooterStatus == ShooterStatus.Idling) {
            setTargetRPM(3000);
        }
    }
    public void updateTelemetry() {
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Current RPM", getFlyWheelRPM());
        telemetry.addData("At Target", isAtTargetRPM());
        telemetry.addData("Motor Power", currentMotorPower);
        telemetry.addData("PID Output", PIDoutput);
        telemetry.addData("Kp", Kp);
        telemetry.addData("Ki", Ki);
        telemetry.addData("Kd", Kd);
        telemetry.addData("PID Threshold", pidThreshold);
        telemetry.addData("Tolerance", tolerance);
    }
}
