package org.firstinspires.ftc.teamcode.examples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * Demonstrates the difference between power control and position control
 */
@TeleOp(name = "Power vs Position Control")
public class PowerVsPosition extends LinearOpMode {
    
    private DcMotorEx testMotor;
    
    @Override
    public void runOpMode() throws InterruptedException {
        testMotor = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        
        telemetry.addData("Status", "Ready");
        telemetry.addData("A Button", "Power Control Mode");
        telemetry.addData("B Button", "Position Control Mode");
        telemetry.addData("Right Stick", "Control Input");
        telemetry.update();
        
        waitForStart();
        
        while (opModeIsActive()) {
            // POWER CONTROL MODE
            if (gamepad1.a) {
                testMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                
                // You still control with POWER, not position:
                double power = gamepad1.right_stick_y;
                testMotor.setPower(power);  // ← POWER control
                
                telemetry.addData("Mode", "POWER CONTROL");
                telemetry.addData("Power Set", power);
                telemetry.addData("Current Power", testMotor.getPower());
                telemetry.addData("Velocity", testMotor.getVelocity());
                telemetry.addData("Position", testMotor.getCurrentPosition());
            }
            
            // POSITION CONTROL MODE (for comparison)
            else if (gamepad1.b) {
                testMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                
                // Position control - motor tries to reach specific encoder position:
                int targetPos = testMotor.getCurrentPosition() + (int)(gamepad1.right_stick_y * 1000);
                testMotor.setTargetPosition(targetPos);  // ← POSITION control
                testMotor.setPower(0.5); // Power is just speed limit for position control
                
                telemetry.addData("Mode", "POSITION CONTROL");
                telemetry.addData("Target Position", targetPos);
                telemetry.addData("Current Position", testMotor.getCurrentPosition());
                telemetry.addData("Power", testMotor.getPower());
                telemetry.addData("Is Busy", testMotor.isBusy());
            }
            
            telemetry.addLine();
            telemetry.addData("A Button", "Power Control (what you want)");
            telemetry.addData("B Button", "Position Control (for comparison)");
            telemetry.addData("Right Stick", "Control Input");
            
            telemetry.update();
        }
    }
}

