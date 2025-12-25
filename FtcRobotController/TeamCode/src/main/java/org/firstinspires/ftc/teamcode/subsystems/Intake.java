package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


// TODO: Adapt the system into our robot
@Config
public class Intake extends SubsystemBase {

    private final DcMotor intake;
    private final Servo transferLeft;
    private final Servo transferRight;

    public IntakeTransferState intakeCurrentState = IntakeTransferState.Intake_Steady;

    // set the 3 status as false in default
    public boolean shooterAuto = false;
    public boolean autoTrans = false;

    public boolean autoForce = false;

    public double servoDiff = 0;

    public double servoTestpos = 0.2;

    public double testmode = 0;

    public boolean gatepos = false;




    // Constructor for intake motors
    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake");
        transferLeft = hardwareMap.get(Servo.class, "transferL");
        transferRight = hardwareMap.get(Servo.class, "transferR");


        // The intake does not need to necessarily move at steady
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // The transfer has to be steady for the case where there are already balls in the
        // transfer stage
        //transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        setServoPos(servoTestpos);
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
    public void setIntakePower(double power) {

        intake.setPower(power);

    }

//    public void setTransferPower(double power) {
//        transfer.setPower(power);
//    }

    // Enum which stores all the power needed for each state of the intake motors
    public enum IntakeTransferState {
        Suck_In(1,0.71),
        Split_Out(-0.8,0.2),
        Send_It_Up(1,0.2),
        Intake_Steady(0,0.2);
        private final double intakePower;
        private final double transServer;
        // Set update the transfer state
        IntakeTransferState(double InPower, double serverPos) {
            this.intakePower = InPower;
            this.transServer = serverPos;
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
            if(!gatepos) {
                setServoPos(intakeCurrentState.transServer);
            }
            else{
                setServoPos(intakeCurrentState.transServer+0.2);
            }
        }
        else{
            if(autoTrans){
                intakeCurrentState = IntakeTransferState.Send_It_Up;
                intake.setPower(intakeCurrentState.intakePower);
                setServoPos(intakeCurrentState.transServer);
            }
            else{
                intakeCurrentState = IntakeTransferState.Intake_Steady;
                intake.setPower(intakeCurrentState.intakePower);
                setServoPos(intakeCurrentState.transServer);
            }
        }

    }

    // Standardization of the two functions
    public void updateAutoshoot(boolean auto){
        shooterAuto = auto;
    }

    public void updateautotranse(boolean auto){
        autoTrans = auto;
    }

    @Override
    public void periodic() { // FTC 0.001s cycle
        if(!shooterAuto || autoForce) {
            // at shooterAuto or autoForce, the power of the DC motors are set separately
            // thus you will need to make sure that the robot is not in these two states
            intake.setPower(intakeCurrentState.intakePower);
            setServoPos(intakeCurrentState.transServer);
        }
        else{
            if(autoTrans){
                // autoTrans is the state of sending the ball from intake position to shooting
                // position
                intakeCurrentState = IntakeTransferState.Send_It_Up;

            }
            else{
                // if not, then the intake doesn't need to do anything
                intakeCurrentState = IntakeTransferState.Intake_Steady;
            }
            // update the power to the motors
            intake.setPower(intakeCurrentState.intakePower);
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
        setServoPos(intakeCurrentState.transServer);
        gatepos = false;
    }
}
