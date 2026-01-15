package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Math.abs;
import static java.lang.Math.floor;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.PinpointLocalizer;


// TODO: Adapt the system into our robot
@Config
public class Turret extends SubsystemBase {
    // battery is not yet installed and configured
    // shooter is not yet installed and configured

    private final DcMotorEx turretMotor;

    public boolean shooterAuto = false;

    public boolean autoForce = false;
    public static double pidDiff = 0;

    public double kp = -0.015;
    public double kd = 0.0000;
    public double ki = 0.001;

    public double kf = 0;

    public static double encoderkp = 0.0;
    public static double encoderkd = 0.00;
    public static double encoderki = 0;

    public static double encoderkf = 0;

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

    public double arctoDegree = 162.42;

    public double llbar = 8;

    public DigitalChannel magLim;

    public boolean isManeulCentering = false;

    public boolean centeringDir = false;

    private boolean maneulCenteringFlag = false;
    public double centerVel = 250;

    public boolean automode = false;

    public int autopos = 0;

    public double offset = 0;
    public boolean Movingshooting = false;
    private double output = 0;

    public double aimposition = 0;
    public GoBildaPinpointDriver pin;

    public double baseHeading = 0;
    // Constructor for intake motors

    public Turret(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        magLim = hardwareMap.get(DigitalChannel.class,"maglim");
        pin =  hardwareMap.get(GoBildaPinpointDriver.class,"pinpointturret");
        pin.setHeading(0,AngleUnit.RADIANS);
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
        aimangle = arcangle;
        turretpidController.setSetPoint(calculateAim());
       
        aimposition = calculateAim();
        turretpidController.setPIDF(encoderkp,encoderki,encoderkd,encoderkf);
        output = turretpidController.calculate(readAngle());
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
        aimangle = 0;
        turretpidController.setSetPoint(calculateAim());
        aimposition = calculateAim();
        turretpidController.setPIDF(encoderkp,encoderki,encoderkd,encoderkf);
        output = turretpidController.calculate(readAngle());
        if(output >1){
            output =1;
        }
        else if(output<-1){
            output = -1;
        }
        turretMotor.setPower(output);

    }
    public void settoangleAuto(double angle){
        if(turretMotor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER){
            turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        aimangle = angle;
        turretpidController.setSetPoint(calculateAim());
        turretpidController.setPIDF(encoderkp,encoderki,encoderkd,encoderkf);
        output = turretpidController.calculate(readAngle());
        if(output >1){
            output =1;
        }
        else if(output<-1){
            output = -1;
        }
        turretMotor.setPower(output);
    }

    public double readAngle(){

        return pin.getHeading(AngleUnit.RADIANS);
    }

    public double calculateAim(){
        return compress(aimangle+baseHeading);
    }

    public double compress(double angle){
        if(angle>Math.PI){
            return angle-2*Math.PI;
        }
        else if(angle<-Math.PI){
            return angle+2*Math.PI;
        }
        else{
            return angle;
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
                pin.setHeading(3.1415926,AngleUnit.RADIANS);
                isManeulCentering = false;
            }
        }
//        else{
//            if(turretMotor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER){
//                turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            }
//            turretMotor.setVelocity(-centerVel);
//            if(isCentered()){
//                maneulCenteringFlag = true;
//            }
//            if(!isCentered()&&maneulCenteringFlag){
//                turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                isManeulCentering = false;
//                maneulCenteringFlag = false;
//            }
//        }
        else{
            if(turretMotor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER){
                turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            turretMotor.setVelocity(-centerVel);
            if(isCentered()){
                turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                pin.setHeading(3.1415926,AngleUnit.RADIANS);
                isManeulCentering = false;
            }
        }
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
        pin.update();
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
            settoangleAuto(autopos);
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
