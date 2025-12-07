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

    public static double xpos = 15.46;
    public static double ypos = -25.67;

    public Pose bluenearAimPos = new Pose(15.46,-25.67,0);


    public Pose rednearAimPos = new Pose(15.46,25.67,0);
    public Pose aimPos = bluenearAimPos;

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


    public double getdis(){
        double x = follower.getPose().getX()-xpos;
        double y = follower.getPose().getY()-ypos;
        return sqrt(x*x+y*y);
    }
    public double getturretangle(){
//        double x = follower.getPose().getX()-aimPos.getX();
//        double y = follower.getPose().getY()-aimPos.getY();
        double x = follower.getPose().getX()-xpos;
        double y = follower.getPose().getY()-ypos;
        double h = follower.getPose().getHeading();
        if(y<0){
            return 1*h-Math.atan(abs(y)/abs(x));
        }
        else{
            return 1*h+Math.atan(abs(y)/abs(x));
        }
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

