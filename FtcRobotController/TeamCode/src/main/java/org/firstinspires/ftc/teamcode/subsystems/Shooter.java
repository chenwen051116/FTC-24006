package org.firstinspires.ftc.teamcode.subsystems;

import android.annotation.SuppressLint;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Base64;
import java.util.List;

public class Shooter extends SubsystemBase {
    private final Servo hoodServo;
    private final DcMotorEx upShooter;
    private final DcMotorEx downShooter;
    public boolean shooterOn = false;
    //timer for tests
    private long startTime = 0;
    private boolean timerActive = false;
    private double targetRPM = 6450.0;
    private double finalTime = 0.0;
    private boolean targetReached = false;
    private static final String TIME_FORMAT = "%.2f";
    //
    public Shooter(HardwareMap hardwareMap) {
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        upShooter = hardwareMap.get(DcMotorEx.class, "upShooter");
        downShooter = hardwareMap.get(DcMotorEx.class, "downShooter");

        downShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        upShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        downShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    public void updateShooter() {
        double power = shooterOn ? 1.0 : 0.0;
        upShooter.setPower(power);
        downShooter.setPower(power);
    }

    public void setShooterPower(double power) {
        upShooter.setPower(power);
        downShooter.setPower(power);
    }

    public void increaseHoodDeg(double increment){
        if (0 <= hoodServo.getPosition() || hoodServo.getPosition() <= 1){
            hoodServo.setPosition(hoodServo.getPosition() + Math.abs(increment));
        }
    }
    public void decreaseHoodDeg(double decrement){
        if (0 <= hoodServo.getPosition() || hoodServo.getPosition() <= 1){
            hoodServo.setPosition(hoodServo.getPosition() - Math.abs(decrement));
        }
    }
    public void setHoodServoPos(double pos){
        hoodServo.setPosition(pos);
    }
     public double getHoodServoPos(){
        return hoodServo.getPosition();
     }

    public double getFlyWheelVelocity(){
        return downShooter.getVelocity() * (2.0 * Math.PI) * 14 /(28.0 * 10); // Rad/s
    }

    public double getFlyWheelRPM(){
        return downShooter.getVelocity() * 60.0 * 14 /(28.0 * 10); // RPM
    }
    public void startRPMTimer() {
        if (!timerActive) {
            startTime = System.currentTimeMillis();
            timerActive = true;
        }
    }
    //TIMER stuff
    public void resetRPMTimer() {
        timerActive = false;
        targetReached = false;
        startTime = 0;
        finalTime = 0.0;
    }

    @SuppressLint("DefaultLocale")
    public String getTimeToTargetRPM() {
        if (!timerActive && !targetReached) {
            return "Timer not active";
        }

        if (targetReached) {
            return String.format(TIME_FORMAT, finalTime) + "s";
        }

        double currentRPM = getFlyWheelRPM();
        if (currentRPM >= targetRPM) {
            finalTime = (System.currentTimeMillis() - startTime) / 1000.0;
            targetReached = true;
            timerActive = false;
            return String.format(TIME_FORMAT, finalTime) + "s";
        } else {
            double timeElapsed = (System.currentTimeMillis() - startTime) / 1000.0;
            return String.format(TIME_FORMAT, timeElapsed) + "s (Target: " + targetRPM + " RPM)";
        }
    }

    public boolean hasReachedTargetRPM() {
        return getFlyWheelRPM() >= targetRPM;
    }
    //End of timer stuff
    @SuppressLint("DefaultLocale")
    public List<String> getMotorCurrents(){
        double upCurrent = upShooter.getCurrent(CurrentUnit.AMPS);
        double downCurrent = downShooter.getCurrent(CurrentUnit.AMPS);
        return Arrays.asList(
                String.format("%.3f", upCurrent),
                String.format("%.3f", downCurrent)
        );
    }
}
