package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.config.reflection.ReflectionConfig;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

@Config
public class Intake extends SubsystemBase {
    private final CachingDcMotorEx intakeL;
    private final CachingDcMotorEx intakeR;
    private final Servo drop;

    public IntakeState intakeCurrentState = IntakeState.INTAKE_STOP;

    // ---- auto-feed correlation with the shooter (ported from the world bot) ----
    // While the shooter is in Shooting mode (shooterAuto), the intake feeds at
    // full speed once the flywheel has reached its target RPM (autoTrans).
    // The Shooter's rpmreached latch keeps isAtTargetRPM() true after the first
    // time at RPM, so the feed stays on until shooting stops.
    public boolean shooterAuto = false;
    public boolean autoTrans = false;

    public Intake(HardwareMap hardwareMap) {
        drop = hardwareMap.get(Servo.class, "drop");
        intakeL = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "intakeLeft"));
        intakeR = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "intakeRight"));

        // Explicitly register our @Config statics with the dashboard
        // (the automatic DEX scan misses secondary dex files in large apps).
        FtcDashboard.getInstance().withConfigRoot(root ->
                root.putVariable("Intake", ReflectionConfig.createVariableFromClass(Intake.class)));
    }

    public enum IntakeState {
        INTAKING(-0.8),
        INTAKE_STOP(0),
        OUTTAKING(0.6);
        private final double intakeMotorPower;

        IntakeState(double intakeMotorPower) {
            this.intakeMotorPower = intakeMotorPower;
        }
    }

    /** True while the intake should auto-feed balls into the shooter. */
    public boolean autoFeeding() {
        return shooterAuto && autoTrans;
    }

    public void dropup(){
        drop.setPosition(0.18);
    }
    public void dropdown(){ drop.setPosition(0.595);
    }

    public void updateAutoshoot(boolean auto) {
        if (shooterAuto && !auto) {
            // shooting just ended -> stop the feed once
            autoTrans = false;
            setIntakeState(IntakeState.INTAKE_STOP);
        }
        shooterAuto = auto;
    }

    public void updateautotranse(boolean auto) {
        autoTrans = auto;
    }

    public void setIntakeState(IntakeState intakeState) {
        intakeCurrentState = intakeState;
        // manual control applies, except while auto-feeding
        // (OUTTAKING always works so the driver can clear jams)
        if (intakeState == IntakeState.OUTTAKING || !autoFeeding()) {
            intakeL.setPower(intakeState.intakeMotorPower);
            intakeR.setPower(intakeState.intakeMotorPower);
        }
    }

    @Override
    public void periodic() {
        // auto-feed at full intake speed while the shooter is ready
        if (autoFeeding() && intakeCurrentState != IntakeState.OUTTAKING) {
            intakeL.setPower(IntakeState.INTAKING.intakeMotorPower);
            intakeR.setPower(IntakeState.INTAKING.intakeMotorPower);
        }
    }

    /**
     * Clear state and stop motors for a clean TeleOp start.
     */
    public void resetTeleop() {
        shooterAuto = false;
        autoTrans = false;
        setIntakeState(IntakeState.INTAKE_STOP);
    }
}
