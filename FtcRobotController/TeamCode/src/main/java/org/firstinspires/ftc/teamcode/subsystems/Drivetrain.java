package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Math.abs;
import static java.lang.Math.sqrt;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.configurables.PanelsConfigurables;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
public class Drivetrain extends SubsystemBase {

    //declare motors.. 声明，赋值...
    //private final DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;


    private MecanumDrive drive;

    public Follower follower;

    public GoBildaPinpointDriver pin;

    public static double xpos = 126.67;
    public static double ypos = -129.01;

    public static double angle = 0;

    public Pose bluenearAimPos = new Pose(xpos,ypos,angle);


    public Pose rednearAimPos = new Pose(xpos,-ypos,angle);
    public Pose aimPos = bluenearAimPos;

    public Pose blueInitpose = new Pose(0.1224,-0.3717,3.141);
    public Pose redInitpose = new Pose(-0.02583,-0.09087,-3.141);

    public static boolean TredFblue = false;

    // dashboard tunable
    public static double turretTolDeg = 10.0;   // tolerance / deadband
    public static double turretMaxStepDeg = 0; // 0 = no rate limit, else limit per call

    private static boolean hasLastTurretCmd = false;
    private static double lastTurretCmdRad = 0.0;


    //servos

    public Drivetrain(HardwareMap hardwareMap) {      //Constructor,新建对象时需要
//        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeft");
//        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRight");
//        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeft");
//        backRightMotor = hardwareMap.get(DcMotor.class, "backRight");
       // drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        follower = Constants.createFollower(hardwareMap);
        PanelsConfigurables.INSTANCE.refreshClass(this);
        follower.startTeleopDrive();
        follower.update();
        follower.setStartingPose(new Pose(0,0,0));
//        pin = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
//        pin.setPosition(new Pose2D(DistanceUnit.MM,0,0, AngleUnit.RADIANS,0));
//        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void teleDrive(double frontBackVelocity, double strafeVelocity, double turnVelocity) {
        // What the follower *effectively* sees – include your sign flips here:
        double y  = frontBackVelocity;
        double x  = strafeVelocity;
        double rx = turnVelocity;

        if(abs(rx)<0.1){
            rx = 0;
        }
        // Recreate mecanum mixing
        double fl = y + x + rx;
        double bl = y - x + rx;
        double fr = y - x - rx;
        double br = y + x - rx;

        double maxMag = Math.max(
                Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br))
        );

        double scale = 1.0;
        if (maxMag > 1.0) {
            scale = 1.0 / maxMag;
        }

        double yScaled  = y  * scale;
        double xScaled  = x  * scale;
        double rxScaled = rx * scale;

        follower.update();
        // undo the sign changes we baked into x, rx
        follower.setTeleOpDrive(yScaled, -xScaled, -rxScaled, true);
    }


//    public void teleDrive (double frontBackVelocity, double strafeVelocity, double turnVelocity){
////        double y = frontBackVelocity;
////        double x = strafeVelocity;
////        double rx = turnVelocity;
//        follower.update();
//        follower.setTeleOpDrive(frontBackVelocity, -strafeVelocity, -turnVelocity, true);
////        // Denominator is the largest motor power (absolute value) or 1
////        // This ensures all the powers maintain the same ratio, but only when
////        // at least one is out of the range [-1, 1]
////        double denominator = Math.max(abs(y) + abs(x) + abs(rx), 1);
////        double frontLeftPower = (y + x + rx) / denominator;
////        double backLeftPower = (y - x + rx) / denominator;
////        double frontRightPower = (y - x - rx) / denominator;
////        double backRightPower = (y + x - rx) / denominator;
////
////        frontLeftMotor.setPower(frontLeftPower);
////        frontRightMotor.setPower(frontRightPower);
////        backLeftMotor.setPower(backLeftPower);
////        backRightMotor.setPower(backRightPower);
////        pin.update();
//    }
    // 在 Drivetrain getter
//    public double getFrontLeftPower() {
//        return frontLeftMotor.getPower();
//    }
//
//    public double getFrontRightPower() {
//        return frontRightMotor.getPower();
//    }
//
//    public double getBackLeftPower() {
//        return backLeftMotor.getPower();
//    }
//
//    public double getBackRightPower() {
//        return backRightMotor.getPower();
//    }

    public void localizerInit(double x, double y, double heading){
        follower.setPose(new Pose(x,y,heading));
    }


    public void originInit(){
        if(TredFblue){
            localizerInit(redInitpose.getX(),redInitpose.getY(),redInitpose.getHeading());
        }
        else{
            localizerInit(blueInitpose.getX(),blueInitpose.getY(),blueInitpose.getHeading());
        }
    }

    public double getdis(){
        double x = follower.getPose().getX()-xpos;
        double y = follower.getPose().getY()-ypos;
        return sqrt(x*x+y*y);
    }
    private static double wrapRad(double a) {
        // wrap to (-pi, +pi]
        while (a <= -Math.PI) a += 2.0 * Math.PI;
        while (a >  Math.PI) a -= 2.0 * Math.PI;
        return a;
    }

    private static double unwrapToNearest(double wrappedTarget, double reference) {
        // choose an equivalent angle (target + 2k*pi) closest to reference
        double k = Math.round((reference - wrappedTarget) / (2.0 * Math.PI));
        return wrappedTarget + k * (2.0 * Math.PI);
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    public double getturretangle() {
        // Robot pose
        double rx = follower.getPose().getX();
        double ry = follower.getPose().getY();
        double rh = follower.getPose().getHeading() + angle; // your extra offset

        // Vector FROM robot TO target (important!)
        double dx = xpos - rx;
        double dy = ypos - ry;

        // Global angle to target
        double worldAim = Math.atan2(dy, dx); // [-pi, pi], robust in all quadrants

        // Turret angle relative to robot front
        double relWrapped = wrapRad(worldAim - rh); // [-pi, pi]

        // First call init
        if (!hasLastTurretCmd) {
            hasLastTurretCmd = true;
            lastTurretCmdRad = relWrapped;
            return lastTurretCmdRad;
        }

        // Unwrap to the equivalent angle closest to last command (prevents 180 -> -180 spin)
        double relUnwrapped = unwrapToNearest(relWrapped, lastTurretCmdRad);

        // Deadband / tolerance to ignore tiny jitter near the wrap boundary
        double tolRad = Math.toRadians(turretTolDeg);
        double delta = relUnwrapped - lastTurretCmdRad;
        if (Math.abs(delta) < tolRad) {
            return lastTurretCmdRad;
        }

        // Optional rate limit (helps if pose heading is noisy)
        if (turretMaxStepDeg > 0) {
            double maxStep = Math.toRadians(turretMaxStepDeg);
            double step = clamp(delta, -maxStep, +maxStep);
            lastTurretCmdRad += step;
        } else {
            lastTurretCmdRad = relUnwrapped;
        }

        return lastTurretCmdRad;
    }


    public void redinit(){
        xpos = rednearAimPos.getX();
        ypos = rednearAimPos.getY();
    }

    public void blueinit(){
        xpos = bluenearAimPos.getX();
        ypos = bluenearAimPos.getY();
    }



}

