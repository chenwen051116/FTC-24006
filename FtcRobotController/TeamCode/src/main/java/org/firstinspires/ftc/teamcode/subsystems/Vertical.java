package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Vertical extends SubsystemBase {
    private final DcMotor vertLeft, vertRight;

    private final PIDController pidController;
    private boolean usePID = true;
    public static double Kp = 0.023, Ki = 0, Kd = 0;
    private Vertical.VertState currentVertState = VertState.VERT_ZERO;
    private double vertTargetPosition;

    public Vertical(HardwareMap hardwareMap) {
        vertLeft = hardwareMap.get(DcMotor.class, "vertLeft");
        vertRight = hardwareMap.get(DcMotor.class, "vertRight");
        pidController = new PIDController(Kp, Ki, Kd);
        vertLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vertLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        vertRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void periodic() {
        vertMotorsetPower(pidController.calculate(vertLeft.getCurrentPosition(), currentVertState.vertExtensionValue));
    }

    public void setVertState(VertState vertState) {
        currentVertState = vertState;
    }

    public enum VertState {
        SAMPLE_LOW(450),
        SAMPLE_HIGH(950),
        SPEC_BEFORE(180),
        SPEC_AFTER(470),
        VERT_ZERO(-5);
        private final int vertExtensionValue;


        VertState(int vertExtensionValue) {
            this.vertExtensionValue = vertExtensionValue;
        }
    }

    public int getVertLeftEncoder() {
        return vertLeft.getCurrentPosition();
    }

    public void setVertRawPower(double Power) {
        vertMotorsetPower(Power);
    }
    public void vertEnablePID() {
        usePID = true;
    }

    public void vertDisablePID() {
        usePID = false;
    }

    public boolean getVertPIDStatus() {
        return usePID;
    }
    public void vertMotorsetPower(double vertMotorPower) {
        vertLeft.setPower(vertMotorPower);
        vertRight.setPower(vertMotorPower);
    }

}
