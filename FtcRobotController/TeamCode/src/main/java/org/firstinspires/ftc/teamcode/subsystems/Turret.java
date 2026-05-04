package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Math.abs;
import static java.lang.Math.floor;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


// TODO: Adapt the system into our robot
@Config
public class Turret extends SubsystemBase {
    // battery is not yet installed and configured
    // shooter is not yet installed and configured

    private final ServoImplEx turretServoLeft;
    private final ServoImplEx turretServoRight;

    public boolean shooterAuto = false;

    public boolean autoForce = false;

    public int currentpos = 0;
    public double aimangle = 0;

    public double tx =0;
    public double turretpidOut;


    public double tolerance = 1;

    public static double arctoDegree = 0.155019;

    public double llbar = 8;

    public boolean isIndexing = false;

    public DigitalChannel magLim;


    public boolean automode = false;

    public int autopos = 0;

    public double offset = 0;
    public boolean Movingshooting = false;
    public double output = 0;

    public double aimposition = 0;

    public double zerooff = 0;

    public static double ServoLoff = 0;

    

    // Constructor for intake motors
    public void autoreset(){
                turretServoLeft.setPosition(0.5+ServoLoff);
        turretServoRight.setPosition(0.5);
    }

    public Turret(HardwareMap hardwareMap, boolean isTele) {


        turretServoLeft = hardwareMap.get(ServoImplEx.class, "turretServoL");
        turretServoRight = hardwareMap.get(ServoImplEx.class, "turretServoR");
        turretServoLeft.setPwmRange(new PwmControl.PwmRange(500,2500));
        turretServoRight.setPwmRange(new PwmControl.PwmRange(500,2500));
        // We do not have distance sensor thus the following object should be removed
        // in future updates
        // The intake does not need to necessarily move at steady
//        turretServoLeft.setPosition(0.5+ServoLoff);
//        turretServoRight.setPosition(0.5);
        // The transfer has to be steady for the case where there are already balls in the
        // transfer stage
        automode = false;
        //intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    // Enum which stores all the power needed for each state of the intake motors
//    public void initEncoder(){
//        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }


    public boolean isCentered(){
        return !magLim.getState();
    }

    public void settoangle(double arcangle){


        if(isIndexing){

        }
        aimposition = -arcangle*arctoDegree;
        if((0.5+ServoLoff+aimposition)>0.995){
            turretServoLeft.setPosition(0.995);
            turretServoRight.setPosition(0.995);
            return;
        }
        else if ((0.5+ServoLoff+aimposition)<0.005){
            turretServoLeft.setPosition(0.005);
            turretServoRight.setPosition(0.005);
            return;
        }
        turretServoLeft.setPosition(0.5+ServoLoff+aimposition);
        turretServoRight.setPosition(0.5+aimposition);
        //turretMotor.setTargetPosition((int) floor(arcangle*arctoDegree));
//        if(turretMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
//            turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }
    }



    public boolean isfocuedTu(){
        return true;
    }

    public void centering(){

        turretServoLeft.setPosition(0.5+ServoLoff);
        turretServoRight.setPosition(0.5);

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
            if (shooterAuto || autoForce) {
                settoangle(aimangle+offset);

            } else {
                centering();
            }
        }

    }

    /**
     * Clear auto flags and stop the motor so TeleOp starts clean.
     */
    public void resetTeleop() {
        automode = false;
        shooterAuto = false;
        autoForce = false;
    }
}
