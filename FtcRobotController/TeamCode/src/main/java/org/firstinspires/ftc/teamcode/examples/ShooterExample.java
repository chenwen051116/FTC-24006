package org.firstinspires.ftc.teamcode.examples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;

/**
 * Example OpMode showing how to use the PID-controlled shooter
 */
@TeleOp(name = "Shooter Example")
public class ShooterExample extends LinearOpMode {
    
    private Shooter shooter;
    
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize shooter
        shooter = new Shooter(hardwareMap);
        
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Instructions", "A = 1000 RPM, B = 1500 RPM, X = Stop, Y = Manual Mode");
        telemetry.update();
        
        waitForStart();
        
        while (opModeIsActive()) {
            // Gamepad controls
            if (gamepad1.a) {
                shooter.setTargetRPM(1000); // Low speed
            } else if (gamepad1.b) {
                shooter.setTargetRPM(1500); // High speed
            } else if (gamepad1.x) {
                shooter.completeStop(); // Stop
            } else if (gamepad1.y) {
                // Manual mode - use right stick
                double manualPower = gamepad1.right_stick_y;
                shooter.setFlywheelPower(manualPower);
            }
            
            // Update PID controller
            shooter.updateFlywheelPID();
            
            // Display telemetry
            shooter.updateTelemetry();
            telemetry.addData("Gamepad A", "Set 1000 RPM");
            telemetry.addData("Gamepad B", "Set 1500 RPM");
            telemetry.addData("Gamepad X", "Stop");
            telemetry.addData("Gamepad Y", "Manual Mode");
            telemetry.addData("Right Stick", "Manual Power (when Y pressed)");
            
            telemetry.update();
        }
    }
}

