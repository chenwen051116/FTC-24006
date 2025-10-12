package org.firstinspires.ftc.teamcode.examples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * Test to demonstrate velocity reading differences between motor modes
 */
@TeleOp(name = "Velocity Test")
public class VelocityTest extends LinearOpMode {
    
    private DcMotorEx testMotor;
    private boolean usingEncoder = false;
    
    @Override
    public void runOpMode() throws InterruptedException {
        testMotor = hardwareMap.get(DcMotorEx.class, "shooterLeft"); // Use your motor name
        
        telemetry.addData("Status", "Ready");
        telemetry.addData("A Button", "Toggle Motor Mode");
        telemetry.addData("B Button", "Toggle Power (0.5/0.0)");
        telemetry.update();
        
        waitForStart();
        
        boolean motorRunning = false;
        
        while (opModeIsActive()) {
            // Toggle motor mode
            if (gamepad1.a) {
                usingEncoder = !usingEncoder;
                if (usingEncoder) {
                    testMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                } else {
                    testMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
                sleep(200); // Debounce
            }
            
            // Toggle motor power
            if (gamepad1.b) {
                motorRunning = !motorRunning;
                testMotor.setPower(motorRunning ? 0.5 : 0.0);
                sleep(200); // Debounce
            }
            
            // Display velocity readings
            telemetry.addData("Motor Mode", usingEncoder ? "RUN_USING_ENCODER" : "RUN_WITHOUT_ENCODER");
            telemetry.addData("Motor Power", testMotor.getPower());
            telemetry.addData("Raw Velocity", testMotor.getVelocity());
            telemetry.addData("Motor Running", motorRunning);
            
            telemetry.addLine();
            telemetry.addData("Instructions", "A = Toggle Mode, B = Toggle Power");
            telemetry.addData("Notice", "Compare velocity stability between modes");
            
            telemetry.update();
        }
    }
}

