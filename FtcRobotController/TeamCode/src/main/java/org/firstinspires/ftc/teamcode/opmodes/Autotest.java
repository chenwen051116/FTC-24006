package org.firstinspires.ftc.teamcode.opmodes;

import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.config.reflection.ReflectionConfig;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Twist2d;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Timer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

/**
 * Drivetrain with Road Runner style Pinpoint localization.
 *
 * The Pinpoint computer does the wheel/IMU fusion on-board; like Road Runner's
 * PinpointLocalizer, we configure the device (offsets, resolution, directions,
 * IMU recalibration), read its fused position each update, and track a
 * world-frame origin transform on top so setPose() works without writing to
 * the device mid-run.
 */
@Config
public class Drivetrain extends SubsystemBase {

    // ---- drive motors ----
    private final DcMotorEx frontLeft, frontRight, backLeft, backRight;

    // ---- pinpoint odometry ----
    public final GoBildaPinpointDriver pinpoint;

    // Pinpoint pod offsets in inches (from the Pedro Pinpoint AutoTuner)
    public static double pinpointXOffset =-3.9570 ;
    public static double pinpointYOffset =
            -6.6142;

    // world -> pinpoint frame transform (updated by setPose)
    private double originX = 0, originY = 0, originHeading = 0;

    // last raw pinpoint reading (pinpoint frame)
    private double pinX = 0, pinY = 0, pinH = 0;
    private double pinVX = 0, pinVY = 0, pinW = 0;

    // ---- aim point ----
    // The statics are the source of truth and are read LIVE on every access,
    // so updates (code or Dashboard) apply immediately -- no restart needed.
    // aimTrimX/Y are per-run driver adjustments (trim buttons), reset each run.
    public static double aimPointX = -49.43;
    public static double aimPointY = 44.46;

    // second aim point, used while the robot's y is above aimSwitchY
    public static double aimPoint2X = -51.70;
    public static double aimPoint2Y = 81.2377;
    public static double aimSwitchY = 62.0;

    public static double bluexOffset = -0;
    public static double blueyOffset = 0;

    public double aimTrimX = 0;
    public double aimTrimY = 0;

    /** True while the robot is on the far side (y > aimSwitchY) -> use aim point 2. */
    public boolean usingAimPoint2() {
        return pose().y() > aimSwitchY;
    }

    /** Live aim point X: static + driver trim (trim buttons). */
    public double aimX() {
        if (usingAimPoint2()) {
            return aimPoint2X + aimTrimX;
        }
        return aimPointX + aimTrimX;
    }

    /** Live aim point Y: static + driver trim (trim buttons). */
    public double aimY() {
        if (usingAimPoint2()) {
            return aimPoint2Y + aimTrimY;
        }
        return aimPointY + aimTrimY;
    }


    public void offsetchange(double dx, double dy){
        aimPointY+=dy;
        aimPoint2Y+=dy;
        aimPointX+=dx;
        aimPoint2X+=dx;
    }
    public Pose2d predictedPose = new Pose2d();
    public static double lookAheadTime = 0.03;

    public static double kPShooter = -0.2;

    public static double testspeedx = 0.2;
    public static double testspeedy = 0.2;
    public static double testspeedrx = 0.2;

    // Distance from robot center to the ball exit point along the robot heading
    public static double shooterCenterDis = 1.496;

    public boolean safeMode = false;

    // ---- aim assist ----
    // when true and the driver lets go of the turn stick, the drivetrain
    // PID-holds its heading pointed at the live aim point (aimX(), aimY())
    public boolean aimingMode = false;

    public static double aimKp = 2.0;             // turn power per radian of heading error
    public static double aimKd = 0.15;            // damping on angular velocity
    public static double aimMaxTurn = 1;        // turn power clamp while aiming
    public static double aimHeadingOffset = 0.0;  // 0 = shooter fires out the front, Math.PI = out the back

    public Timer looptimer;

    public double looptime = 0;

    public boolean ifMovingShooting = false;

    public static boolean TredFblue = false;

    public static Pose lastPose = new Pose(0, 0, 0);

    public Pose blueInitpose = new Pose(0.74, -0.036, 3.141);
    public Pose redInitpose = new Pose(0.74, 0.036, -3.141);

    public Drivetrain(HardwareMap hardwareMap, boolean isTele) {      //Constructor,新建对象时需要
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        // directions from the Mecanum AutoTuner output
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotorEx m : new DcMotorEx[]{frontLeft, frontRight, backLeft, backRight}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setOffsets(pinpointXOffset, pinpointYOffset, DistanceUnit.INCH);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD, // X
                GoBildaPinpointDriver.EncoderDirection.REVERSED); // Y
        try {
            pinpoint.recalibrateIMU();
            Thread.sleep(500); // let the IMU calibration settle
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        pinpoint.resetPosAndIMU();

        setPose(lastPose);
        looptimer = new Timer();

        // Explicitly register our @Config statics with the dashboard.
        // The dashboard's automatic DEX scan misses classes in secondary dex
        // files (this app is large enough for multidex), so without this the
        // Drivetrain category never appears / edits never apply.
        FtcDashboard.getInstance().withConfigRoot(root ->
                root.putVariable("Drivetrain", ReflectionConfig.createVariableFromClass(Drivetrain.class)));
    }

    // ------------------------------------------------------------------
    // Localization (Road Runner style)
    // ------------------------------------------------------------------

    /** Pull the latest fused pose/velocity off the Pinpoint. */
    public void updatePinpoint() {
        pinpoint.update();
        if (pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY) {
            pinX = pinpoint.getPosX(DistanceUnit.INCH);
            pinY = pinpoint.getPosY(DistanceUnit.INCH);
            pinH = pinpoint.getHeading(UnnormalizedAngleUnit.RADIANS);
            pinVX = pinpoint.getVelX(DistanceUnit.INCH);
            pinVY = pinpoint.getVelY(DistanceUnit.INCH);
            pinW = pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
        }
    }

    /** Current field pose = worldOrigin ∘ pinpointReading. */
    public Pose pose() {
        double c = cos(originHeading), s = sin(originHeading);
        return new Pose(
                originX + pinX * c - pinY * s,
                originY + pinX * s + pinY * c,
                originHeading + pinH);
    }

    /** Redefines the world frame so the current pose reads as the given pose. */
    public void setPose(Pose p) {
        // inverse of the current pinpoint reading
        double c = cos(pinH), s = sin(pinH);
        double invX = -(pinX * c + pinY * s);
        double invY = pinX * s - pinY * c;

        // origin = p ∘ reading⁻¹
        double cp = cos(p.heading()), sp = sin(p.heading());
        originX = p.x() + invX * cp - invY * sp;
        originY = p.y() + invX * sp + invY * cp;
        originHeading = p.heading() - pinH;
    }

    /** Field-frame forward/lateral velocity (in/s). */
    public double velX() {
        double c = cos(originHeading), s = sin(originHeading);
        return pinVX * c - pinVY * s;
    }

    public double velY() {
        double c = cos(originHeading), s = sin(originHeading);
        return pinVX * s + pinVY * c;
    }
    public double velAngle(){
        return (pose().heading() - lastPose.heading())/looptime;
    }

    public double velOmega() {
        return pinW;
    }

    // ------------------------------------------------------------------
    // Driving
    // ------------------------------------------------------------------

    public void teleDrive(double frontBackVelocity, double strafeVelocity, double turnVelocity) {
        double y = frontBackVelocity;
        double x = strafeVelocity;
        double rx = turnVelocity;

        if (abs(rx) < 0.1) {
            rx = 0;
            // aim assist takes over when the driver is not turning
            if (aimingMode) {
                rx = -aimTurnCommand(); // convert the CCW-positive command to the mixing sign
            }
        }

        // mecanum mixing -- identical wheel powers to what the Pedro follower produced
        double fl = y + x + rx;
        double fr = y - x - rx;
        double bl = y - x + rx;
        double br = y + x - rx;

        double maxMag = 1.0;
        maxMag = Math.max(maxMag, Math.abs(fl));
        maxMag = Math.max(maxMag, Math.abs(fr));
        maxMag = Math.max(maxMag, Math.abs(bl));
        maxMag = Math.max(maxMag, Math.abs(br));

        updatePinpoint();

        frontLeft.setPower(fl / maxMag);
        frontRight.setPower(fr / maxMag);
        backLeft.setPower(bl / maxMag);
        backRight.setPower(br / maxMag);
    }

    public void xposChange(double change) {
        aimTrimX += change;
    }

    // ------------------------------------------------------------------
    // Aim assist
    // ------------------------------------------------------------------

    /** Heading that points the shooter at the live aim point (aimX(), aimY()). */
    public double aimTargetHeading() {
        return Math.atan2(aimY() - pose().y(), aimX() - pose().x()) + aimHeadingOffset;
    }

    /** Signed heading error to the aim point, wrapped to [-pi, pi] radians. */
    public double aimError() {
        double e = aimTargetHeading() - pose().heading();
        return Math.atan2(Math.sin(e), Math.cos(e));
    }

    /** CCW-positive turn command that rotates the robot onto the aim heading. */
    public double aimTurnCommand() {
        double cmd = aimKp * aimError() - aimKd * velOmega();
        return Math.max(-aimMaxTurn, Math.min(aimMaxTurn, cmd));
    }

    public boolean aimed(){
        return Math.abs((Math.toDegrees(aimError())))<(5);
    }

    public void yposChange(double change) {
        aimTrimY += change;
    }

    public void localizerInit(double x, double y, double heading) {
        setPose(new Pose(x, y, heading));
    }

    /** Reset the current pose to (0, 0, 0). */
    public void zeroPose() {
        setPose(new Pose(0-1.41732283, 0, 0));
    }

    // height of the aim point above the ball exit (1.51 m)
    public static double aimHeightIn = 59.41;

    /** 3D distance to the aim point: sqrt(horizontal^2 + height^2). */
    public double getdis3D() {
        double d = getdis();
        return sqrt(d * d + aimHeightIn * aimHeightIn);
    }

    public void originInit() {
        if (TredFblue) {
            localizerInit(redInitpose.x(), redInitpose.y(), redInitpose.heading());
        } else {
            localizerInit(blueInitpose.x(), blueInitpose.y(), blueInitpose.heading());
        }
    }

    public double getdis() {
        ifMovingShooting = false;
        if (safeMode) {
            return 95;
        }
        double heading = pose().heading();
        double realx = pose().x();
        double realy = pose().y();
        if (heading > 0) {
            realx -= shooterCenterDis * cos(heading);
            realy -= shooterCenterDis * sin(heading);
        } else {
            heading = abs(heading);
            realx -= shooterCenterDis * cos(heading);
            realy += shooterCenterDis * sin(heading);
        }
        double x = realx - aimX();
        double y = realy - aimY();
        return sqrt(x * x + y * y);
    }

    public double getdis_TWO() {
        ifMovingShooting = true;
        if (safeMode) {
            return 95;
        }
        predictedPose = lookaheadPoseTime(new Pose2d(
                        pose().x(),
                        pose().y(),
                        new Rotation2d(pose().heading())),
                velX(),
                velY(),
                0,
                lookAheadTime
        );
        double heading = pose().heading();
        double realx = predictedPose.getX();
        double realy = predictedPose.getY();
        if (heading > 0) {
            realx -= shooterCenterDis * cos(heading);
            realy -= shooterCenterDis * sin(heading);
        } else {
            heading = abs(heading);
            realx -= shooterCenterDis * cos(heading);
            realy += shooterCenterDis * sin(heading);
        }
        double x = realx - aimX();
        double y = realy - aimY();
        return sqrt(x * x + y * y) + kPShooter * forwardvel();
    }

    public double getallspeed() {
        return sqrt(velX() * velX() + velY() * velY());
    }

    public double forwardvel() {
        double dx = pose().x() - aimX();
        double dy = pose().y() - aimY();

        double dist = Math.sqrt(dx * dx + dy * dy);

        return -(dx * velX() + dy * velY()) / dist;
    }

    public Pose2d lookaheadPoseTime(Pose2d current, double vx, double vy, double omega, double lookaheadTimeSec) {
        double dx = vx * lookaheadTimeSec;
        double dy = vy * lookaheadTimeSec;
        double dtheta = omega * lookaheadTimeSec;

        Twist2d twist = new Twist2d(dx, dy, dtheta);
        return current.exp(twist);
    }

    public void redinit() {
        TredFblue = true;
    }

    public void blueinit() {
        TredFblue = false;
    }

    public void looptimeupdate() {
        looptime = looptimer.milliseconds();
        looptimer.reset();
    }

    public void period() {
        looptimeupdate();
        lastPose = pose();
    }
}
