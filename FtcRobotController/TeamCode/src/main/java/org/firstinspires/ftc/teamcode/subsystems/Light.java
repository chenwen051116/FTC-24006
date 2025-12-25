package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Math.abs;
import static java.lang.Math.sqrt;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

public class Light extends SubsystemBase {
    // Hardware (motor servo...)
    private final Servo LightLeft;
    private final Servo LightRight;

    public Light(HardwareMap hardwareMap) {
        LightLeft = hardwareMap.get(Servo.class,"lightleft");
        LightRight = hardwareMap.get(Servo.class,"lightright");
    }

    public enum Color{
        Red(0.29),
        Yellow(0.388),
        Green(0.500),
        Blue(0.611),
        Violet(0.722),
        Orange(0.333),
        Off(0);
        private final double value;
        Color(double value){
            this.value = value;
        }
    }

    public void setLight(Color left, Color right){
        LightLeft.setPosition(left.value);
        LightRight.setPosition(right.value);
    }
    @Override
    public void periodic() {
        // This is called automatically by FTCLib’s scheduler every cycle
    }
}


