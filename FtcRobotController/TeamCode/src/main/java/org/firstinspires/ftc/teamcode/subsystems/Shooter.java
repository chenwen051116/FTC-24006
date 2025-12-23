package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.util.Timer;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class Shooter extends SubsystemBase {
    private final Servo shootLimit;
    private final DcMotorEx shooterLeft;
    private final DcMotorEx shooterRight;
    private final PIDController pidController;

    private final DistanceSensor distanceSensor;

    // Tunable PID parameters - can be adjusted via FTC Dashboard
    public static double Kp = 0.15;  // Proportional gain
    public static double Ki = 0; // Integral gain
    public static double Kd = 0;    // Derivative gain

    public static double Kf = 1.8;    // Friction gain

    public static double kv = 0.000205; // FeedForward velocity gain
    public static double pidThreshold = 1000.0; // RPM threshold for PID vs full power control
    public static double tolerance = 0.3; // RPM tolerance for "at target" determination

    public  double aimRPM = 0;

    // Target RPM for the flywheel
    private double targetRPM = 0.0;

    public double distance = 0;

    public boolean idelOn = false;

    public boolean focused = false;

    public boolean automode = false;

    public boolean autoLonger = true;
    public  double shootInterval = 0;

    public double PIDoutput;

    public double RPMThresh = 50;

    public  double Autoshort = 2620;
    public  double Autolong = 3090;

    //public double AutoStartlong = 3125;

    public  double shootLowbar = 200;

    public double rpmdiff = 0;

    public  double rpmdiffthresh = 50;

    public double realtargetRPM = 0;

    public  double lowerrpmDiffThresh = -100;

    public double lastrpm = 0;
    public boolean reverIntake = false;
    public Timer shootTimer;

    public double shootlimitpos = 0;

    public double ododis = 0;

    public boolean rpmreached = false;

    public boolean forceShooting = false;
    public enum ShooterStatus {
        
        Stop,Idling,Shooting
    }


    // Create a new SimpleMotorFeedforward with gains kS, kV, and kA
    public ShooterStatus shooterStatus = ShooterStatus.Stop;



    public Shooter(HardwareMap hardwareMap) {
        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "transferdis");
        shootLimit = hardwareMap.get(Servo.class,"shootLimit");
        shootTimer = new Timer();
        // Initialize PID controller
        pidController = new PIDController(Kp, Ki, Kd);

        // Configure motors
       shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

       shooterLeft.setDirection(DcMotor.Direction.FORWARD);
        shooterRight.setDirection(DcMotor.Direction.FORWARD);

        // Configure motor modes - only shooterLeft has encoder
        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);  // Has encoder
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // No encoder


        // Set PID tolerance (adjustable via static parameter)
        pidController.setTolerance(tolerance);
        shootbarOn();

        focused = false;

        automode = false;

        autoLonger = true;
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
//    public double getFlyWheelVelocity() {
//        return shooterLeft.getVelocity() * (2.0 * Math.PI) / 60.0; // Convert RPM to rad/s
//
//    }

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

    public double getTransDis(){
        return distanceSensor.getDistance(DistanceUnit.CM);
    }
    public double getTargetRPM() {
        return targetRPM;
    }
    public boolean isAtTargetRPM() {
        if(getTargetRPM() < getFlyWheelRPM()+RPMThresh&& getTargetRPM() > getFlyWheelRPM()-RPMThresh&& targetRPM>2500){
            rpmreached = true;
        }
//        reverIntake = shootTimer.getElapsedTimeSeconds() < shootInterval;
//        if(isDeccel()){
//            shootTimer.resetTimer();
//            return false;
//        }
        return ((getTargetRPM() < getFlyWheelRPM() + RPMThresh && getTargetRPM() > getFlyWheelRPM()-RPMThresh)&&getFlyWheelRPM()>2500&&(focused||automode))||(forceShooting&&rpmreached);

//        else{
//
//            return shootTimer.getElapsedTimeSeconds() > shootInterval && abs(rpmdiff)<rpmdiffthresh&&getFlyWheelRPM()>1000&&(focused||automode);
//        }

    }

    public boolean isDeccel(){
        return rpmdiff<lowerrpmDiffThresh;
    }

    public void shootbarOn(){
        shootLimit.setPosition(0.97);
    }

    public void shootbarOff(){
        shootLimit.setPosition(0.725);
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

//        if(forceShooting){
//            if(shooterLeft.getMode()!= DcMotor.RunMode.RUN_WITHOUT_ENCODER){
//                shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            }
//            shooterLeft.setPower(1);
//            shooterRight.setPower(-1);
//        }
//        else {
//            if(shooterLeft.getMode()!= DcMotor.RunMode.RUN_USING_ENCODER){
//                shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            }
//
//            shooterLeft.setVelocityPIDFCoefficients(Kp, Ki, Kd, Kf);
//            shooterRight.setVelocityPIDFCoefficients(Kp, Ki, Kd, Kf);
//
//            shooterLeft.setVelocity(targetRPM * 28 / 60);
//            shooterRight.setVelocity(-targetRPM * 28 / 60);
            rpmdiff = lastrpm - getFlyWheelRPM();
            lastrpm = getFlyWheelRPM();
            if (rpmdiff < rpmdiffthresh) {
                realtargetRPM = getFlyWheelRPM();
            }
        if (targetRPM > 0) {
            // Update PID parameters and tolerance in case they were changed via dashboard
            pidController.setPID(Kp, Ki, Kd);
            pidController.setTolerance(tolerance);

            double currentRPM = getFlyWheelRPM();
            double rpmDifference = currentRPM - targetRPM;

            double pidinput = rpmDifference/100.0;
            double power;
            double pidOutput = 0.0;

            if (abs(rpmDifference) <= pidThreshold) {
                // Use PID control for fine-tuning within ±pidThreshold RPM
                pidOutput = pidController.calculate(pidinput)+kv*targetRPM;
                power = Math.max(-1.0, Math.min(1.0, pidOutput)); //smart brahhh
            } else if (rpmDifference < pidThreshold) {
                // Large speed increase needed - use full power
                power = 1.0;
                pidOutput = 1.0; // PID would output 1.0 but we're overriding
            } else {
                // Large speed decrease needed - use no power (let inertia slow it down)
                power = 0.0;
                pidOutput = 0.0; // PID would output negative but we're overriding
            }
            PIDoutput = power;
            // Store values for telemetry/graphing
            currentMotorPower = power;
            currentPIDOutput = pidOutput;

            // Apply power to both motors
            shooterLeft.setPower(power);
            shooterRight.setPower(-power);
        } else {
            // Stop motors if no target set
            currentMotorPower = 0.0;
            currentPIDOutput = 0.0;
            shooterLeft.setPower(0);
            shooterRight.setPower(0);
        }
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
        setTargetRPM(0);
        setFlywheelPower(0);

        pidController.reset();
    }

    public void toggleRPM() {
        setTargetRPM(aimRPM);
        shooterStatus = ShooterStatus.Shooting;
    }

    public void updateAim() {
        double dis = abs(ododis);
        if (dis > 60){
            setTargetRPM(7.3743*dis+2148.4);
        }
        else if (dis <= 60){
            setTargetRPM(-7.3743*dis+3049);
        }


        if(automode&&autoLonger){
            setTargetRPM(Autolong);
        }
        else if(automode&&!autoLonger){
            setTargetRPM(Autoshort);
        }
        //setTargetRPM(aimRPM);
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
//        shootLimit.setPosition(shootlimitpos);
        updateFlywheelPID();
        if(shooterStatus == ShooterStatus.Shooting){
            updateAim();
                shootbarOff();
        }
        else if(shooterStatus == ShooterStatus.Stop){
            rpmreached = false;
            completeStop();
            shootbarOn();
        }
        else if(shooterStatus == ShooterStatus.Idling) {
            rpmreached = false;
            setTargetRPM(2400);
            shootbarOn();
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

    /**
     * Clear auto flags and stop motors for a clean TeleOp start.
     */
    public void resetTeleop() {
        automode = false;
        forceShooting = false;
        autoLonger = true;
        rpmreached = false;
        setShooterStatus(ShooterStatus.Stop);
        completeStop();
    }
}
