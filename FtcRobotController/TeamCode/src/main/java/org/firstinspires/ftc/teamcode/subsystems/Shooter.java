package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.util.Timer;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class Shooter extends SubsystemBase {
    private final DcMotorEx shooterLeft;
    private final DcMotorEx shooterRight;


    // Tunable PID parameters - can be adjusted via FTC Dashboard
    public static int backpos = 80;



    // Create a new SimpleMotorFeedforward with gains kS, kV, and kA


    public Shooter(HardwareMap hardwareMap) {
        shooterLeft = hardwareMap.get(DcMotorEx.class, "catapult1");
        shooterRight = hardwareMap.get(DcMotorEx.class, "catapult2");


        // Configure motors
       shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

       shooterLeft.setDirection(DcMotor.Direction.FORWARD);
        shooterRight.setDirection(DcMotor.Direction.FORWARD);

        // Configure motor modes - only shooterLeft has encoder
        shooterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // Has encoder
        shooterRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // No encoder
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);  // Has encoder
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // No encoder


        // Set PID tolerance (adjustable via static parameter)

    }

    /**
     * Get current flywheel velocity in rad/s
     * Uses shooterLeft (the motor with encoder) for velocity feedback
     *
     * Clear auto flags and stop motors for a clean TeleOp start.
     */
    public void back(){

        shooterLeft.setTargetPosition(backpos);
        shooterRight.setTargetPosition(backpos);
        shooterRight.setPower(1);
        shooterLeft.setPower(1);
        shooterLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shooterRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public void shoot(){
        if(abs(shooterLeft.getCurrentPosition())<5){
            shooterLeft.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
            shooterRight.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        }
        else{
            shooterLeft.setTargetPosition(0);
            shooterRight.setTargetPosition(0);
            shooterRight.setPower(1);
            shooterLeft.setPower(1);
            shooterLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            shooterRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
    public void resetTeleop() {

    }
}
