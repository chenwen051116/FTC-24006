package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Math.abs;
import static java.lang.Math.floor;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


// TODO: Adapt the system into our robot
@Config
public class Turret extends SubsystemBase {
    // battery is not yet installed and configured
    // shooter is not yet installed and configured

    private final DcMotorEx turretMotor;

    public boolean shooterAuto = false;

    public boolean autoForce = false;
    public static double pidDiff = 0;

    public double kp = 0.0;
    public double kd = 0.0000;
    public double ki = 0.00;

    public double kf = 0;

    public static double encoderkp = -0.00035;
    public static double encoderkd = -0.000015;
    public static double encoderki = -0.00;

    public static double encoderkf = -0.000009;

    private final PIDFController turretpidController;
    public double highkp = -2;
    public double txbar = 5;

    public int targetpos = 0;

    public int currentpos = 0;
    public double aimangle = 0;

    public double tx =0;
    public double turretpidOut;

    private final PIDController pidController;

    public double tolerance = 1;

    public double arctoDegree = 8184.9497;

    public double llbar = 8;

    public DigitalChannel magLim;

    public boolean isManeulCentering = false;

    public boolean centeringDir = false;

    private boolean maneulCenteringFlag = false;
    public double centerVel = 0.4;

    public boolean automode = false;

    public int autopos = 0;

    public double offset = 0;
    public boolean Movingshooting = false;
    public double output = 0;

    public double aimposition = 0;

    public double zerooff = 0;
    

    // Constructor for intake motors

    public Turret(HardwareMap hardwareMap, boolean isTele) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        magLim = hardwareMap.get(DigitalChannel.class,"maglim");
        magLim.setMode(DigitalChannel.Mode.INPUT);

        // We do not have distance sensor thus the following object should be removed
        // in future updates
        // The intake does not need to necessarily move at steady
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if(!isTele) {
            turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        // The transfer has to be steady for the case where there are already balls in the
        // transfer stage
        pidController = new PIDController(kp,ki,kd);
        turretpidController = new PIDFController(encoderkp,encoderki,encoderkd,encoderkf);
        turretpidController.setSetPoint(0);
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

        if(turretMotor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER){
            turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        turretpidController.setSetPoint((int) -floor(arcangle*arctoDegree) + zerooff);
        aimposition = (int) -floor(arcangle*arctoDegree);
        turretpidController.setPIDF(encoderkp,encoderki,encoderkd,encoderkf);
        output = turretpidController.calculate(turretMotor.getCurrentPosition());
        if(output >1){
            output =1;
        }
        else if(output<-1){
            output = -1;
        }
        turretMotor.setPower(output);
        //turretMotor.setTargetPosition((int) floor(arcangle*arctoDegree));
//        if(turretMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
//            turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }
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

        if(turretMotor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER){
            turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        turretpidController.setSetPoint(0+zerooff);
        turretpidController.setPIDF(encoderkp,encoderki,encoderkd,encoderkf);
        output = turretpidController.calculate(turretMotor.getCurrentPosition());
        if(output >1){
            output =1;
        }
        else if(output<-1){
            output = -1;
        }
        turretMotor.setPower(output);

    }
    public void settopos(int pos){
        if(turretMotor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER){
            turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        turretpidController.setSetPoint(pos+zerooff);
        turretpidController.setPIDF(encoderkp,encoderki,encoderkd,encoderkf);
        output = turretpidController.calculate(turretMotor.getCurrentPosition());
        if(output >1){
            output =1;
        }
        else if(output<-1){
            output = -1;
        }
        turretMotor.setPower(output);
    }

    public void manuelCenter(){
        if(centeringDir){
            if(turretMotor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER){
                turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            turretMotor.setPower(centerVel);
            if(isCentered()){
                maneulCenteringFlag = true;
            }
            if(!isCentered()&&maneulCenteringFlag){
                turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                zerooff = 1548;
                isManeulCentering = false;
                maneulCenteringFlag = false;
            }
        }
        else{
            if(turretMotor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER){
                turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            turretMotor.setPower(-centerVel);
            if(isCentered()){
                maneulCenteringFlag = true;
            }
            if(!isCentered()&&maneulCenteringFlag){
                turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                zerooff = -1094;
                isManeulCentering = false;
                maneulCenteringFlag = false;
            }
        }
//        else{
//            if(turretMotor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER){
//                turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            }
//            turretMotor.setPower(-centerVel);
//            if(isCentered()){
//                turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                isManeulCentering = false;
//            }
//        }
    }

    // Standardization of the two functions
    public void updateAutoShoot(boolean auto){
        shooterAuto = auto;
    }

    public void changeOffset(double change){
        offset+=change;
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

                    settoangle(aimangle+offset);
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

    /**
     * Clear auto flags and stop the motor so TeleOp starts clean.
     */
    public void resetTeleop() {
        automode = false;
        shooterAuto = false;
        autoForce = false;
        isManeulCentering = false;
        centeringDir = false;
        maneulCenteringFlag = false;

        turretpidOut = 0;
        if (turretMotor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
            turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        turretMotor.setPower(0);
    }
}
