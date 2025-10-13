package org.firstinspires.ftc.teamcode.subsystems;

import android.hardware.Sensor;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Intake extends SubsystemBase {
    private final DcMotor intake, transfer;
    private final DistanceSensor transferBreakBeam;
    public boolean ballAtTransfer = false;

    public Intake(HardwareMap hardwareMap) {      //Constructor,新建对象时需要
        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotor.class, "transfer");
        transferBreakBeam = hardwareMap.get(DistanceSensor.class, "transferBreakBeam");

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

       intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public double getDist(){
        return transferBreakBeam.getDistance(DistanceUnit.MM);
    }

    public void setIntakePower(double power) {
        intake.setPower(power);
    }

    public void setTransferPower(double power) {
        transfer.setPower(power);
    }
    @Override
    public void periodic() { // FTC 0.001s cycle
//        if (getDist() < 100) {
//            ballAtTransfer = true;
//        }else{
//            ballAtTransfer = false;
//        }
    }
}
