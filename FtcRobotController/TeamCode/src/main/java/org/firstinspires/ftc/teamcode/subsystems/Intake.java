package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.config.reflection.ReflectionConfig;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;

@Config
public class Intake extends SubsystemBase {
    private final CachingDcMotorEx intakeL;
    private final CachingDcMotorEx intakeR;
    private final Servo flowerL, flowerR;
    public IntakeState intakeCurrentState = IntakeState.INTAKE_STOP;

    // ---- auto-feed correlation with the shooter (ported from the world bot) ----
    // While the shooter is in Shooting mode (shooterAuto), the intake feeds at
    // full speed once the flywheel has reached its target RPM (autoTrans).
    // The Shooter's rpmreached latch keeps isAtTargetRPM() true after the first
    // time at RPM, so the feed stays on until shooting stops.
    public boolean flowerTesting = false;
    public boolean shooterAuto = false;
    public boolean autoTrans = false;
    public static double flowerPos = 0;
    public static double flowerOffset = 0;
    // Runtime flower positions per intake state (tune with "Servo Tune" or the
    // dashboard -- these are what matches actually use).
    public static double flowerPosIntaking = 0.8;
    public static double flowerPosIntakingFlower = 0.18;
    public static double flowerPosStop = 0.8;
    public static double flowerPosOuttaking = 0.8;

    public Intake(HardwareMap hardwareMap) {
        intakeL = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "intakeL"));
        intakeR = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "intakeR"));
        flowerL = new CachingServo(hardwareMap.get(Servo.class, "flowerL"));
        flowerR = new CachingServo(hardwareMap.get(Servo.class, "flowerR"));


        // Explicitly register our @Config statics with the dashboard
        // (the automatic DEX scan misses secondary dex files in large apps).
        FtcDashboard.getInstance().withConfigRoot(root ->
                root.putVariable("Intake", ReflectionConfig.createVariableFromClass(Intake.class)));
    }

    public enum IntakeState {
        INTAKING(0.8),
        INTAKING_FLOWER(0.8),
        INTAKE_STOP(0),
        OUTTAKING(-0.6);
        private final double intakeMotorPower;

        IntakeState(double intakeMotorPower) {
            this.intakeMotorPower = intakeMotorPower;
        }
    }

    /** Flower servo position for a given intake state (from the tunables). */
    private double flowerPoseFor(IntakeState s) {
        switch (s) {
            case INTAKING: return flowerPosIntaking;
            case INTAKING_FLOWER: return flowerPosIntakingFlower;
            case OUTTAKING: return flowerPosOuttaking;
            case INTAKE_STOP:
            default: return flowerPosStop;
        }
    }

    /** True while the intake should auto-feed balls into the shooter. */
    public boolean autoFeeding() {
        return shooterAuto && autoTrans;
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
        if (!flowerTesting){
            setFlowers(flowerPoseFor(intakeState));
        }
        // manual control applies, except while auto-feeding
        // (OUTTAKING always works so the driver can clear jams)
        if (intakeState == IntakeState.OUTTAKING || !autoFeeding()) {
            intakeL.setPower(intakeState.intakeMotorPower);
            intakeR.setPower(intakeState.intakeMotorPower);
        }
    }

    /** Command both flower servos to a position (right gets + flowerOffset). */
    public void setFlowers(double pos) {
        flowerL.setPosition(pos);
        flowerR.setPosition(pos + flowerOffset);
    }

    @Override
    public void periodic() {
        if (flowerTesting){
            flowerL.setPosition(flowerPos);
            flowerR.setPosition(flowerPos + flowerOffset);
        }
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
