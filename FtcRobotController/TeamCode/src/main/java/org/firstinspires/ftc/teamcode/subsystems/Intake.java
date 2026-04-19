package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


// TODO: Adapt the system into our robot
@Config
public class Intake extends SubsystemBase {

    private final DcMotor intake,trans;
    private final Servo transferLeft;
    private final Servo transferRight;

    public boolean autoIntakeUp = false;

    public IntakeTransferState intakeCurrentState = IntakeTransferState.Intake_Steady;

    // set the 3 status as false in default
    public boolean shooterAuto = false;
    public boolean autoTrans = false;

    public boolean autoForce = false;

    public double servoDiff = 0;

    public static double servoTestpos = 0.2;


    public double testmode = 0;



    public static boolean isFarTeleMode = true;
public  double FarTeleTransFactor = 1;

    public double transferSpeed = 1;
    public double hasball1sum,hasball2sum,hasball3sum = 0;



    private DigitalChannel breakbeam1,breakbeam2,breakbeam3;


    // Constructor for intake motors
    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake");
        trans = hardwareMap.get(DcMotor.class, "trans");
        transferLeft = hardwareMap.get(Servo.class, "transferL");
        transferRight = hardwareMap.get(Servo.class, "transferR");

        breakbeam1 = hardwareMap.get(DigitalChannel.class, "breakbeam1");
        breakbeam1.setMode(DigitalChannel.Mode.INPUT);
        breakbeam2 = hardwareMap.get(DigitalChannel.class, "breakbeam2");
        breakbeam2.setMode(DigitalChannel.Mode.INPUT);
        breakbeam3 = hardwareMap.get(DigitalChannel.class, "breakbeam3");
        breakbeam3.setMode(DigitalChannel.Mode.INPUT);

        // The intake does not need to necessarily move at steady
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // The transfer has to be steady for the case where there are already balls in the
        // transfer stage
        //transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        trans.setDirection(DcMotorSimple.Direction.REVERSE);
        //setServoPos(servoTestpos);
    }

    public boolean hasballCheck(int num){
        if(num == 1){
            return hasball1sum>0.15;
        }
        else if (num == 2){
            return hasball2sum>0.15;
        }
        else{
            return hasball3sum>0.15;
        }
    }

    public void updatehasballCondi(){
        if(!breakbeam1.getState()){
            hasball1sum= (hasball1sum*4+1)/5;
            if(hasball1sum<0.09){
                hasball1sum = 0;
            }
        }
        else{
            hasball1sum= (hasball1sum*4)/5;
            if(hasball1sum<0.09){
                hasball1sum = 0;
            }
        }
        if(!breakbeam2.getState()){
            hasball2sum= (hasball2sum*4+1)/5;
            if(hasball2sum<0.09){
                hasball2sum = 0;
            }
        }
        else{
            hasball2sum= (hasball2sum*4)/5;
            if(hasball2sum<0.09){
                hasball2sum = 0;
            }
        }
        if(!breakbeam3.getState()){
            hasball3sum= (hasball3sum*4+1)/5;
            if(hasball3sum<0.09){
                hasball3sum = 0;
            }
        }
        else{
            hasball3sum= (hasball3sum*4)/5;
            if(hasball3sum<0.09){
                hasball3sum = 0;
            }
        }
    }

    public void setServoPos(double pos){
        if(testmode>1){
            transferLeft.setPosition(servoTestpos);
            transferRight.setPosition(servoTestpos+servoDiff);
        }
        else {
            transferLeft.setPosition(pos);
            transferRight.setPosition(pos + servoDiff);
        }
    }
    public void setIntakeTransPower(double ipower,double tpower) {

        intake.setPower(ipower);
        trans.setPower(tpower);

    }

//    public void setTransferPower(double power) {
//        transfer.setPower(power);
//    }

    // Enum which stores all the power needed for each state of the intake motors
    public enum IntakeTransferState {
        Suck_In(1,1,0.72),
        Split_Out(-0.8,-1,0.25),
        Send_It_Up(1,1,0.25),
        Intake_Steady(0,0,0.25),

        Suck_In_slow(0.5,0.5,0.25),
        Send_It_Up_Slow(1,0.9,0.25);
        private final double intakePower;
        private final double transPower;
        private final double transServer;
        // Set update the transfer state
        IntakeTransferState(double InPower,double TransPower, double serverPos) {
            this.intakePower = InPower;
            this.transServer = serverPos;
            this.transPower = TransPower;
        }
    }

    public void setSwingBarPos(double i){
        return;
    }

    // This function is not necessary
    // used to update the state of the intake motors when called
    public void setIntakeState(IntakeTransferState intakeTransferState) {
        intakeCurrentState = intakeTransferState;
        if(!shooterAuto || autoForce) {
            intake.setPower(intakeCurrentState.intakePower);
            if((intakeCurrentState == IntakeTransferState.Suck_In||intakeCurrentState==IntakeTransferState.Suck_In_slow)&&hasballCheck(1)&&hasballCheck(2)) {
                trans.setPower(IntakeTransferState.Intake_Steady.transPower);
            }
            else{
                trans.setPower(intakeCurrentState.transPower);
            }
//            if((intakeCurrentState == IntakeTransferState.Suck_In)&&hasballCheck(1)&&hasballCheck(2)&&hasballCheck(3))
//            {
//                setServoPos(IntakeTransferState.Intake_Steady.transServer);
//            }
//            else {
                setServoPos(intakeCurrentState.transServer);
//            }

        }
        else{
            if(autoTrans){
                if(isFarTeleMode){
                    intakeCurrentState = IntakeTransferState.Send_It_Up_Slow;
                    intake.setPower(intakeCurrentState.intakePower);
                    trans.setPower(intakeCurrentState.transPower*FarTeleTransFactor);
                }
                else {
                    intakeCurrentState = IntakeTransferState.Send_It_Up;
                    intake.setPower(intakeCurrentState.intakePower);
                    trans.setPower(intakeCurrentState.transPower);
                }
                setServoPos(intakeCurrentState.transServer);
            }
            else{
                intakeCurrentState = IntakeTransferState.Intake_Steady;
                intake.setPower(intakeCurrentState.intakePower);
                trans.setPower(intakeCurrentState.transPower);
                setServoPos(intakeCurrentState.transServer);
            }
        }

    }

    public boolean bothHasBall(){
        return true;
    }
    // Standardization of the two functions
    public void updateAutoshoot(boolean auto){
        shooterAuto = auto;
    }

    public void updateautotranse(boolean auto){
        autoTrans = auto;
    }

    @Override
    public void periodic() {
        updatehasballCondi();

        // FTC 0.001s cycle
        if(!shooterAuto || autoForce) {
            // at shooterAuto or autoForce, the power of the DC motors are set separately
            // thus you will need to make sure that the robot is not in these two states
            intake.setPower(intakeCurrentState.intakePower);
            if((intakeCurrentState == IntakeTransferState.Suck_In||intakeCurrentState==IntakeTransferState.Suck_In_slow)&&hasballCheck(1)&&hasballCheck(2)) {
                trans.setPower(IntakeTransferState.Intake_Steady.transPower);
            }
            else{
                trans.setPower(intakeCurrentState.transPower);
            }
//            if((intakeCurrentState == IntakeTransferState.Suck_In)&&hasballCheck(1)&&hasballCheck(2)&&hasballCheck(3))
//                {
//                    setServoPos(IntakeTransferState.Intake_Steady.transServer);
//                }
//            else {
                setServoPos(intakeCurrentState.transServer);
//            }
        }
        else{
            if(autoTrans){
                // autoTrans is the state of sending the ball from intake position to shooting
                // position
                if(isFarTeleMode){
                    intakeCurrentState = IntakeTransferState.Send_It_Up_Slow;
                    intake.setPower(intakeCurrentState.intakePower);
                    trans.setPower(intakeCurrentState.transPower*FarTeleTransFactor);
                }
                else {
                    intakeCurrentState = IntakeTransferState.Send_It_Up;
                    intake.setPower(intakeCurrentState.intakePower);
                    trans.setPower(intakeCurrentState.transPower);
                }

                    intake.setPower(transferSpeed);

            }
            else{
                // if not, then the intake doesn't need to do anything
                intakeCurrentState = IntakeTransferState.Intake_Steady;
                intake.setPower(intakeCurrentState.intakePower);
                trans.setPower(intakeCurrentState.transPower);
            }
            // update the power to the motors

            setServoPos(intakeCurrentState.transServer);
        }
    }

    /**
     * Clear auto flags and stop motors for a clean TeleOp start.
     */
    public void resetTeleop() {
        shooterAuto = false;
        autoTrans = false;
        autoForce = false;
        intakeCurrentState = IntakeTransferState.Intake_Steady;
        intake.setPower(0);
    }
}
