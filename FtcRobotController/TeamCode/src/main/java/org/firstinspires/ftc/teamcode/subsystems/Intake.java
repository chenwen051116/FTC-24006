package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config

public class Intake extends SubsystemBase {
    private final DcMotor intake, transfer;
    private final Servo swingBar;

    private final DistanceSensor dis;
    //private final DistanceSensor transferBreakBeam;
    public IntakeTransferState intakeCurrentState = IntakeTransferState.Intake_Steady;

    public boolean shooterauto = false;
    public boolean autotrans = false;

    public static double servopos = 0;

    public boolean autoforce = false;
    public Intake(HardwareMap hardwareMap) {      //Constructor,新建对象时需要
        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotor.class, "transfer");
        swingBar = hardwareMap.get(Servo.class, "swingBar");
        dis = hardwareMap.get(DistanceSensor.class,"distance");
       // transferBreakBeam = hardwareMap.get(DistanceSensor.class, "transferBreakBeam");

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        autoforce = false;
        autotrans = false;
        shooterauto = false;
    }
    public void setSwingBarPos(double pos) {
        swingBar.setPosition(pos);
    }

//    public double getDist(){
//        return transferBreakBeam.getDistance(DistanceUnit.MM);
//    }

    public double disRead(){
        return dis.getDistance(DistanceUnit.CM);
    }
    public void setIntakePower(double power) {

        intake.setPower(power);

    }

    public void setTransferPower(double power) {
        transfer.setPower(power);
    }
    public enum IntakeTransferState {
        Suck_In(1,0),
        Split_Out(-0.7, -1),
        Send_It_Up(1,1),
        Intake_Steady(0,0);
        private final double intakePower;
        private final double transferPower;
        IntakeTransferState(double InPower, double TrPower) {
            this.intakePower = InPower;
            this.transferPower = TrPower;
        }
    }
    public void setIntakeState(IntakeTransferState intakeTransferState) {
        intakeCurrentState = intakeTransferState;
        if(!shooterauto || autoforce) {
            servopos = 0.35;
            if(intakeCurrentState == IntakeTransferState.Suck_In){
                if(disRead()<14.5){
                    intake.setPower(IntakeTransferState.Suck_In.intakePower);
                    transfer.setPower(IntakeTransferState.Suck_In.transferPower);
                }
                else{
                    intake.setPower(IntakeTransferState.Send_It_Up.intakePower);
                    transfer.setPower(IntakeTransferState.Send_It_Up.transferPower);
                }
            }
            else {
                intake.setPower(intakeCurrentState.intakePower);
                transfer.setPower(intakeCurrentState.transferPower);
            }
        }
        else{
            servopos = 0.185;
            if(autotrans){
                intakeCurrentState = IntakeTransferState.Send_It_Up;
            }
            else{
                intakeCurrentState = IntakeTransferState.Intake_Steady;
            }
            intake.setPower(intakeCurrentState.intakePower);
            transfer.setPower(intakeCurrentState.transferPower);
        }


    }

    public void updateAutoshoot(boolean auto){
        shooterauto = auto;
    }

    public void updateautotranse(boolean auto){
        autotrans = auto;
    }

    @Override
    public void periodic() { // FTC 0.001s cycle
        if(!shooterauto || autoforce) {
            servopos = 0.35;
            if(intakeCurrentState == IntakeTransferState.Suck_In){
                if(disRead()<14.5){
                    intake.setPower(IntakeTransferState.Suck_In.intakePower);
                    transfer.setPower(IntakeTransferState.Suck_In.transferPower);
                }
                else{
                    intake.setPower(IntakeTransferState.Send_It_Up.intakePower);
                    transfer.setPower(IntakeTransferState.Send_It_Up.transferPower);
                }
            }
            else {
                intake.setPower(intakeCurrentState.intakePower);
                transfer.setPower(intakeCurrentState.transferPower);
            }
        }
        else{
            servopos = 0.185;
            if(autotrans){
                intakeCurrentState = IntakeTransferState.Send_It_Up;
            }
            else{
                intakeCurrentState = IntakeTransferState.Intake_Steady;
            }
            intake.setPower(intakeCurrentState.intakePower);
            transfer.setPower(intakeCurrentState.transferPower);
        }
    }

    public void resetTeleop() {
        shooterauto= false;
        autotrans = false;
        autoforce = false;
        intakeCurrentState = IntakeTransferState.Intake_Steady;
        intake.setPower(0);
        //setServoPos(intakeCurrentState.transServer);
       // gatepos = false;
    }
}
