package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import static java.lang.Math.abs;
import static java.lang.Math.floor;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.util.Timer;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class Shooter extends SubsystemBase {
    private final Servo shootLimit,hood;
    private final DcMotorEx shooterLeft;
    private final DcMotorEx shooterRight;
    private final PIDController pidController;
    private final VoltageSensor v;

    //private final DistanceSensor distanceSensor;

    // Tunable PID parameters - can be adjusted via FTC Dashboard
    public static double Kp = 0.34;  // Proportional gain
    public static double Ki = 0; // Integral gain
    public static double Kd = 0;    // Derivative gain

    public static double Kf = 0;    // Friction gain

    public static double kv = 0.000210; // FeedForward velocity gain

    public double kvoff  = 0;
    public static double pidThreshold = 300.0; // RPM threshold for PID vs full power control
    public static double tolerance = 0.3; // RPM tolerance for "at target" determination

    public static double aimRPM = 0;

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

    public  double Autoshort = 2580;
    public  double Autolong = 3110;

    public double idleSpeed = 2500;

    public boolean isfocused = false;

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

    public boolean Movingshooting = false;

    public double vol = 14;

    public double offset = 10;

    public static double transFactor = 1;

    public static double hoodpos = 0.2;//-0.61

    public boolean sortingMode = false;
   // public boolean sortedOut = false;
    public enum ShooterStatus {
        
        Stop,Idling,Shooting
    }


    // Create a new SimpleMotorFeedforward with gains kS, kV, and kA
    public ShooterStatus shooterStatus = ShooterStatus.Stop;



    public Shooter(HardwareMap hardwareMap) {
        v=hardwareMap.get(VoltageSensor.class,"Control Hub");
        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
      //  distanceSensor = hardwareMap.get(DistanceSensor.class, "transferdis");
        shootLimit = hardwareMap.get(Servo.class,"shootLimit");
        hood = hardwareMap.get(Servo.class,"hood");
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
        vol = v.getVoltage();
        kv = 0.00023-0.0000086666667*(vol-12.7);

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

    public double getTargetRPM() {
        return targetRPM;
    }
    public boolean isAtTargetRPM() {
        if(getTargetRPM() < getFlyWheelRPM()+RPMThresh&& getTargetRPM() > getFlyWheelRPM()-RPMThresh&& targetRPM!=2000&&targetRPM>1800){
            rpmreached = true;
        }
//        reverIntake = shootTimer.getElapsedTimeSeconds() < shootInterval;
//        if(isDeccel()){
//            shootTimer.resetTimer();
//            return false;
//        }
        return ((getTargetRPM() < getFlyWheelRPM() + RPMThresh && getTargetRPM() > getFlyWheelRPM()-RPMThresh)&&isfocused&&getFlyWheelRPM()>1800)||(forceShooting&&rpmreached);

//        else{
//
//            return shootTimer.getElapsedTimeSeconds() > shootInterval && abs(rpmdiff)<rpmdiffthresh&&getFlyWheelRPM()>1000&&(focused||automode);
//        }

    }

    public boolean isAtFar(){
        return ododis>120;
    }
    public boolean isDeccel(){
        return rpmdiff<lowerrpmDiffThresh;
    }

    public void shootbarOn(){
        shootLimit.setPosition(0.915);
    }

    public void shootbarOff(){
        shootLimit.setPosition(0.99);
    }
    // Store current motor power for telemetry/graphing
    private double currentMotorPower = 0.0;
    private double currentPIDOutput = 0.0;

    public double sortingSpeed = 500;

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
            shooterRight.setPower(power);
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

//    public double[] shortdis = {40
//            ,
//            ,63.6934
//            ,68.3024
//            ,73.3103
//            ,78.3967
//            ,83.3191
//            ,88.4699
//            ,93.2528
//            ,98.4668
//            ,103.132
//            ,108.2516};

    public int[] shortrpm = {
            2000,
            2000,
            2000,
            2100,
            2200,
            2300,
            2400,
            2400,
            2400,
            2500,
            2500,
            2600,
            2650,
            2750,
            2750,
            2850,
            2950,
            3050,
            3050,
            3100,
            3150,
            3150,
            3200,
            3250


    };

    public double[] shorthood = {
            0.2,
            0.2,
            0.2,
            0.2,
            0.3,
            0.4,
            0.5,
            0.5,
            0.5,
            0.5,
            0.5,
            0.5,
            0.5,
            0.5,
            0.5,
            0.5,
            0.6,
            0.6,
            0.6,
            0.6,
            0.6,
            0.6,
            0.6,
            0.6



    };

    public double[] shorttrans = {
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            0.7,
            0.7,
            0.7,
            0.5,
            0.5,
            0.5,
            0.5,
            0.5,



    };

    public double calculateRPM(){
        double dis = abs(ododis);
        double hoodpos = 0.2;


        if (dis <155&&dis>40) {
            int index = (int) floor((dis - 40) / 5.0);
            if (index < 0) {
                index = 0;
            }
            if (index > 24) {
                index = 24;
            }
            double slope = (shortrpm[index + 1] - shortrpm[index]) / 5.0;
            double target = slope * (dis - index * 5 -40) + shortrpm[index];


            double slopehood = (shorthood[index + 1] - shorthood[index]) / 5.0;
            hoodpos = slopehood * (dis - index * 5 - 40) + shorthood[index];
            if (hoodpos < 0.2) {
                hoodpos = 0.2;
            } else if (hoodpos > 0.6) {
                hoodpos = 0.6;
            }
            hood.setPosition(hoodpos);

            //  double slopetrans = (shorttrans[index + 1] - shorttrans[index]) / 5.0;
            transFactor = shorttrans[index];
            if(target+offset<=idleSpeed) {
                return (target + offset);
            }
            else{
                    return idleSpeed;
            }

        }
        else{
            hood.setPosition(0.5);
            transFactor = 1;
            return idleSpeed;
        }

    }

    public void updateAim() {
        double dis = abs(ododis);
        double hoodpos = 0.2;


        if (dis <155&&dis>40) {
            int index = (int) floor((dis - 40) / 5.0);
            if (index < 0) {
                index = 0;
            }
            if (index > 24) {
                index = 24;
            }
            double slope = (shortrpm[index + 1] - shortrpm[index]) / 5.0;
            double target = slope * (dis - index * 5 -40) + shortrpm[index];
            setTargetRPM(target + offset);

            double slopehood = (shorthood[index + 1] - shorthood[index]) / 5.0;
            hoodpos = slopehood * (dis - index * 5 - 40) + shorthood[index];
            if (hoodpos < 0.2) {
                hoodpos = 0.2;
            } else if (hoodpos > 0.6) {
                hoodpos = 0.6;
            }
            hood.setPosition(hoodpos);

          //  double slopetrans = (shorttrans[index + 1] - shorttrans[index]) / 5.0;
            transFactor = shorttrans[index];
        }
        else{
            hood.setPosition(0.5);
            transFactor = 1;
            setTargetRPM(idleSpeed);
        }

      //  }
//        else {
//            int index = (int)floor((dis-123)/5.0);
//            if(index<0){
//                index = 0;
//            }
//            if(index >6){
//                index = 6;
//            }
//            double slope = (longrpm[index+1]-longrpm[index])/(longdis[index+1]-longdis[index]);
//            double target = slope*(dis-longdis[index])+longrpm[index]+20;
//                setTargetRPM(target + offset + 20);
//
//        }


//        if(sortingMode){
//            setTargetRPM(sortingSpeed);
//        }
//        if(automode&&autoLonger){
//            setTargetRPM(Autolong);
//        }
//        else if(automode&&!autoLonger){
//            setTargetRPM(Autoshort);
//        }
      //  setTargetRPM(aimRPM);
    }




    /**
     * Get current motor power (for graphing/telemetry)
     */

    public void changeoffset(double change){
        offset += change;
        if(offset>0){
            kvoff+=0.00001;
        }
        if(offset<0){
            kvoff-=0.00001;
        }
    }
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
 //       hood.setPosition(hoodpos);
//        shootLimit.setPosition(shootlimitpos);
        updateFlywheelPID();
        if(shooterStatus == ShooterStatus.Shooting){
            updateAim();
            if(abs(targetRPM-getFlyWheelRPM())<500) {
                shootbarOff();
            }


        }
        else if(shooterStatus == ShooterStatus.Stop){
            rpmreached = false;
            completeStop();
            shootbarOn();
        }
        else if(shooterStatus == ShooterStatus.Idling) {
            rpmreached = false;
//            if()
            setTargetRPM(calculateRPM());
     //       setTargetRPM(2000);
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
