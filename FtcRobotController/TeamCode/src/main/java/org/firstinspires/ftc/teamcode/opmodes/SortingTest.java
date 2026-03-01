package org.firstinspires.ftc.teamcode.opmodes; // make sure this aligns with class location

import static java.lang.Math.abs;
import static java.lang.Math.sqrt;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.commands.LimelightLockInCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MyLimelight;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Scheduler;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@Config
@Autonomous(name = "SortingTest")

public class SortingTest extends OpMode {

    //private drive.follower drive.follower;
    private Drivetrain drive;
    private Timer pathTimer, actionTimer, opmodeTimer, timer,gatetimer;
    //private final ElapsedTime timer  = new ElapsedTime();

    private int pathState = 0;
    private boolean firstshooting = false;

    public double turretoff = 0;
    private double gatePathPower = 1;
    private PathChain GateShoot,GatePath1,GatePath2, Shootpath1,Shootpath2, Shootpath3,Shootpath4,Shootpath5, lastOutPath;
    private PathChain prepGatherPath6,prepGatherPath1,finishGatherPath6,Shootpath6, prepGatherPath2, prepGatherPath3, prepGatherPath4;

    private PathChain finishGatherPath1,finishGatherPath2,finishGatherPath3,finishGatherPath4;
    public Intake intake;
    public Shooter shooter;
    public MyLimelight limelight;
    public Scheduler scheduler;

    public Turret turret;

    public static double stoptime = 2;
    public static double shoottime = 1.65;

    public static double waittime = 0.5;
    public static double checkcount = 3;

    public static double followingtime = 1.5;

    public double checkcounter = checkcount;




    public  PathChain simplePath(Pose a, Pose b){
        return drive.follower.pathBuilder()
                .addPath(new BezierLine(a, b))
                .setLinearHeadingInterpolation(a.getHeading(), b.getHeading())
                .build();
    }
    public void buildPaths() {

    }

    public boolean sortflag = false;
    public void toggleSortingMode(boolean sort){
        if(sort) {
            shooter.sortingMode = true;
            if(!sortflag) {
                shooter.sortingSpeed = 500;
                shooter.setShooterStatus(Shooter.ShooterStatus.Shooting);
                //shooter.sortedOut = !intake.frontHasBall();
            }

            if ((!intake.fronthasballRaw())||shooter.getTransDis()>18||(shooter.rpmreached&&shooter.getFlyWheelRPM()<440)){
                if(sortflag) {
                    //shooter.sortingSpeed = -1000;
                    shooter.setShooterStatus(Shooter.ShooterStatus.Stop);
                }
                sortflag = true;
            }
            shooter.periodic();
            intake.setIntakeState(Intake.IntakeTransferState.Suck_In_slow_Sorting);
            intake.periodic();
        }
        else{
            shooter.sortingMode = false;
            shooter.setShooterStatus(Shooter.ShooterStatus.Stop);
            //shooter.sortedOut = false;
            sortflag = false;
            shooter.periodic();
            intake.setIntakeState(Intake.IntakeTransferState.Intake_Steady);
            intake.periodic();
        }
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                sortflag = false;
                firstshooting = false;

                if((!intake.fronthasballRaw())){
                    intake.setIntakeState(Intake.IntakeTransferState.Suck_In_slow);
            }
                else {
                    setPathState(1);
                }
                break;
            case 1:
                if (!firstshooting) {
                    shooter.updateFocused(true);
                    toggleSortingMode(true);
                    timer.resetTimer();
                    firstshooting = true;
                }
                else{
                    if(timer.getElapsedTimeSeconds()<5){
                        toggleSortingMode(true);
                    }
                    else{
                        toggleSortingMode(false);
                        setPathState(2);
                    }

                }
                break;


        }
    }
    private void sleep(long ms){
        try{
            Thread.sleep(ms);
        } catch (InterruptedException e){
            Thread.currentThread().interrupt();
        }
    }
    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        // These loop the movements of the robot, these must be called continuously in order to work
        shooter.periodic();
        intake.periodic();
        autonomousPathUpdate();
        telemetry.addData("dis", shooter.getTransDis());
                telemetry.addData("rpmdifft", shooter.rpmdiff);
        telemetry.addData("rpm", shooter.getFlyWheelRPM());
        telemetry.addData("flag", sortflag);
        telemetry.addData("intake front", intake.frontHasBall());
        telemetry.addData("sortingMode",shooter.sortingMode);
        telemetry.update();
    }


    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        pathTimer = new Timer();
        timer = new Timer();
        gatetimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        drive = new Drivetrain(hardwareMap,false);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        limelight = new MyLimelight(hardwareMap);
        limelight.initPatternPipeline();
        limelight.startDetect();
        //intake.setIntakeState(Intake.IntakeTransferState.Intake_Steady);
        shooter.setShooterStatus(Shooter.ShooterStatus.Stop);
        turret = new Turret(hardwareMap,false);
        buildPaths();
        //drive.follower.setStartingPose(startPose);
        drive.blueinit();
//        telemetry.addData("turret target", turret.currentpos);
//        telemetry.addData("turret aim", turret.aimposition);
//        telemetry.addData("Shooter Target RPM", shooter.getTargetRPM());
//        telemetry.addData("Shooter Current RPM", shooter.getFlyWheelRPM());
//        telemetry.update();

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    public void setScheduler(Scheduler scheduler) {
        this.scheduler = scheduler;
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
        resetSubsystemsForTeleop();
        Drivetrain.lastPose = drive.follower.getPose();
        Drivetrain.TredFblue = false;
    }

    /**
     * Ensure all auto-only flags/powers are cleared so TeleOp does not fight leftover commands.
     */
    private void resetSubsystemsForTeleop() {
        if (turret != null) {
            turret.resetTeleop();
        }
        if (shooter != null) {
            shooter.resetTeleop();
        }
        if (intake != null) {
            intake.resetTeleop();
        }
    }
}