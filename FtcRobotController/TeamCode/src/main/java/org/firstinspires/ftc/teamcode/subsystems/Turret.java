package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Math.abs;
import static java.lang.Math.floor;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


// TODO: Adapt the system into our robot
@Config
public class Turret extends SubsystemBase {
    // battery is not yet installed and configured
    // shooter is not yet installed and configured

    private final DcMotorEx turretMotor;

    public boolean shooterAuto = false;

    public boolean autoForce = false;
    public static double pidDiff = 0;

    public static double kp = -0.015;
    public static double kd = 0.0000;
    public static double ki = 0.001;

    public static double kf = 0;
    public static double highkp = -2;
    public static double txbar = 5;

    public static int targetpos = 0;

    public int currentpos = 0;
    public double aimangle = 0;

    public double tx =0;
    public double turretpidOut;

    private final PIDController pidController;

    public static double tolerance = 1;

    public static double arctoDegree = 162.42;

    public static double llbar = 8;

    public DigitalChannel magLim;

    public boolean isManeulCentering = false;

    public boolean centeringDir = false;

    private boolean maneulCenteringFlag = false;
    public static double centerVel = 80;

    public boolean automode = false;

    public int autopos = 0;

    // Constructor for intake motors

    public Turret(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        magLim = hardwareMap.get(DigitalChannel.class,"maglim");
        magLim.setMode(DigitalChannel.Mode.INPUT);

        // We do not have distance sensor thus the following object should be removed
        // in future updates
        // The intake does not need to necessarily move at steady
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // The transfer has to be steady for the case where there are already balls in the
        // transfer stage
        pidController = new PIDController(kp,ki,kd);
        automode = false;
        //intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    // Enum which stores all the power needed for each state of the intake motors
//    public void initEncoder(){
//        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }

    public int getPos(){
        return turretMotor.getCurrentPosition();
    }

    public boolean isCentered(){
        return !magLim.getState();
    }

    public void settoangle(double arcangle){

        if(turretMotor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER && turretMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION){
            turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        turretMotor.setPower(1);
        turretMotor.setTargetPosition((int) floor(arcangle*arctoDegree));
        if(turretMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public void focusMode(){


        pidController.setPIDF(kp,ki,kd,kf);
        pidController.setTolerance(tolerance);
        pidController.setSetPoint(0);
        if(turretMotor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER){
            turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        turretpidOut=pidController.calculate(tx);
        if(turretpidOut>0){
            turretpidOut+=pidDiff;
        }
        else{
            turretpidOut-=pidDiff;
        }
        if(turretpidOut >1){
            turretpidOut = 1;
        }
        if(turretpidOut<-1){
            turretpidOut = -1;
        }

        turretMotor.setPower(turretpidOut);
//        if(abs(tx) < txbar){
//            turretMotor.setPower(0.6);
//            int dpos = (int) floor(kp*tx);
//            targetpos += dpos;
//            turretMotor.setTargetPosition(targetpos);
//            turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }
//        else{
//            turretMotor.setPower(1);
//            int dpos = (int) floor(highkp*tx);
//            targetpos += dpos;
//            turretMotor.setTargetPosition(targetpos);
//            turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }

    }

    public void centering(){

            if (turretMotor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER && turretMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            turretMotor.setPower(0.6);
            targetpos = 0;
            turretMotor.setTargetPosition(targetpos);
            if (turretMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }


    }
    public void settopos(int pos){
        if (turretMotor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER && turretMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        turretMotor.setPower(1);
        targetpos = pos;
        turretMotor.setTargetPosition(targetpos);
        if (turretMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public void manuelCenter(){
        if(centeringDir){
            if(turretMotor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER){
                turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            turretMotor.setVelocity(centerVel);
            if(isCentered()){
                turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                isManeulCentering = false;
            }
        }
        else{
            if(turretMotor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER){
                turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            turretMotor.setVelocity(-centerVel);
            if(isCentered()){
                maneulCenteringFlag = true;
            }
            if(!isCentered()&&maneulCenteringFlag){
                turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                isManeulCentering = false;
                maneulCenteringFlag = false;
            }
        }
    }

    // Standardization of the two functions
    public void updateAutoShoot(boolean auto){
        shooterAuto = auto;
    }


    @Override
    public void periodic() { // FTC 0.001s cycle

        if(!automode) {
            currentpos = turretMotor.getCurrentPosition();
            if (isManeulCentering) {
                manuelCenter();
            } else if (shooterAuto || autoForce) {
                // at shooterAuto or autoForce, the power of the DC motors are set separately
                // thus you will need to make sure that the robot is not in these two states
                //focusMode();
                if (tx > llbar || tx < -llbar || abs(tx) < 0.01) {
                    settoangle(aimangle);
                } else {
                    focusMode();
                }

            } else {
                centering();
            }
        }
        else{
            settopos(autopos);
        }

    }
}
