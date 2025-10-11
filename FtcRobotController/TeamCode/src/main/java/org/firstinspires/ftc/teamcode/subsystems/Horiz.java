package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Horiz extends SubsystemBase {

    private final DcMotor horizontalMotor;

    private final PIDController pidController;
    private boolean usePID = true;
    public static double Kp = 0.006, Ki = 0, Kd = 0;

    private double horizTargetPosition;
    private HorizState currentHorizState = HorizState.HORIZ_ZERO;
    public void horizReset() {
        horizontalMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        horizontalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizEnablePID();
    }
    public Horiz(HardwareMap hardwareMap) {
        horizontalMotor = hardwareMap.get(DcMotor.class, "horiz");
        pidController = new PIDController(Kp, Ki, Kd);
        horizReset();
    }

    public int getHorizMotorEncoder() {return horizontalMotor.getCurrentPosition();}
    public double getHorizMotorPower() {return horizontalMotor.getPower();}
    public void setHorizRawPower(double Power) {
        horizontalMotor.setPower(Power);
    }
    public void horizEnablePID() {
        usePID = true;
    }

    public void horizDisablePID() {
        usePID = false;
    }

    @Override
    public void periodic() {
        if (usePID) {
            horizontalMotor.setPower(pidController.calculate(horizontalMotor.getCurrentPosition(), currentHorizState.extensionValue));
        }
    }

    public void setHorizState(HorizState horizState) {
        currentHorizState = horizState;
    }
    public enum HorizState {
        HORIZ_CLOSE(100),
        HORIZ_MEDIUM(300),
        HORIZ_FAR(600),
        HORIZ_ZERO(-20);
        private final int extensionValue;


        HorizState(int extensionValue) {
            this.extensionValue = extensionValue;
        }
    }

}
