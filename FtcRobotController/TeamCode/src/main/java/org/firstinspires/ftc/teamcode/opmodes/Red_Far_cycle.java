package org.firstinspires.ftc.teamcode.opmodes; // make sure this aligns with class location

import static java.lang.Math.abs;
import static java.lang.Math.sqrt;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
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
@Autonomous(name = "Red_Far_cycle")

public class Red_Far_cycle extends OpMode {

    private Follower follower;
    private Drivetrain drive;
    private Timer pathTimer, actionTimer, opmodeTimer, timer,fulltimer;
    //private final ElapsedTime timer  = new ElapsedTime();

    private int pathState = 0;
    private final Pose startPose = new Pose(81.21237, 2.5016, 0); // Start Pose of our robot.
    private final Pose ShootPose1 = new Pose(81.21, 8.7571,0);
    private final Pose PrepGather1 = new Pose(91.9908, 28.6053+5, 0);
    private final Pose FinishGather1 = new Pose(114.9794, 28.6053, 0);

    private final Pose PrepGather2 = new Pose(91.9908, -52.0297-5, 0);

    private final Pose FinishGather2 = new Pose(114.9794, -52.0297, 0);
    private final Pose GatePassby = new Pose(112.6299, -59.2147, 0);
    private final Pose GatePose = new Pose(122.5,-59.2147, 0);
    private final Pose ShootPose2 = new Pose(84.1620, 75.80 ,0);

    private final Pose Shoot2passby = new Pose(95.7309,-59.2147,0);

    private final Pose PrepGather3 = new Pose(91.9908, -75.8070+8, 0);//accounted for overshoot

    private final Pose FinishGather3 = new Pose(114.9794, -75.8070, 0);

    private final Pose PrepGather4 = new Pose(119.5531, 2.5016, 0);//accounted for overshoot

    private final Pose FinishGather4 = new Pose(123.92, 2.5016, 0);

    private final Pose Park = new Pose(115.21, 8.7571,0);;

    private final Pose FinishGather5 = new Pose(121.42, -0.06455, 0);

    private boolean firstshooting = false;
    private PathChain GatePath, Shootpath1,Shootpath2, Shootpath3,Shootpath4,Shootpath5, lastOutPath;
    private PathChain prepGatherPath6,prepGatherPath1,finishGatherPath6,Shootpath6, prepGatherPath2, prepGatherPath3, prepGatherPath4;

    private PathChain finishGatherPath1,finishGatherPath2,finishGatherPath3,finishGatherPath4;
    public Intake intake;
    public Shooter shooter;

    public Scheduler scheduler;

    public Turret turret;

    public double turretoff = 0;

    public static double stoptime = 2.8;
    public static double shoottime = 3;
    public static double xpos = 129.67;
    public static double ypos = 134.01;

    public static double angle = 0;

    public static double waittime = 0.7;
    public static double intaketime = 0.7;
    public static double checkcount = 10;

    public static double followingtime = 2;

    public double checkcounter = checkcount;

    public double cyclecounter = 5;

    public boolean autoflag = false;


    public  PathChain simpleConstPath(Pose a, Pose b){
        return drive.follower.pathBuilder()
                .addPath(new BezierLine(a, b))
                .setConstantHeadingInterpolation(b.getHeading())
                .build();
    }
    public  PathChain simplePath(Pose a, Pose b){
        return drive.follower.pathBuilder()
                .addPath(new BezierLine(a, b))
                .setLinearHeadingInterpolation(a.getHeading(), b.getHeading())

                .build();
    }
    public void buildPaths() {

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        Shootpath1 = simplePath(startPose,ShootPose1);

        //prepGatherPath1 = simplePath(ShootPose1,PrepGather4);
        prepGatherPath1 = simplePath(startPose,PrepGather4);
        finishGatherPath1 = drive.follower.pathBuilder()
                .addPath(new BezierLine(PrepGather4, FinishGather4))
                .setLinearHeadingInterpolation(PrepGather4.getHeading(), FinishGather4.getHeading())
                .addPath(new BezierLine(FinishGather4,PrepGather4))
                .setLinearHeadingInterpolation(FinishGather4.getHeading(), PrepGather4.getHeading())

                .addPath(new BezierLine(PrepGather4, FinishGather4))
                .setLinearHeadingInterpolation(PrepGather4.getHeading(), FinishGather4.getHeading())

                .build();
        //finishGatherPath1 = simplePath(PrepGather4,FinishGather4);

//        prepGatherPath1 = drive.follower.pathBuilder()
//
//                .addPath(new BezierLine(ShootPose1, PrepGather4))
//                .setLinearHeadingInterpolation(ShootPose1.getHeading(), PrepGather4.getHeading())
//                .addPath(new BezierLine(PrepGather4, FinishGather4))
//                .setLinearHeadingInterpolation(PrepGather1.getHeading(), FinishGather1.getHeading())
//                .build();

        Shootpath2 = simplePath(FinishGather4,ShootPose1);
//
//        prepGatherPath2 = simplePath(ShootPose1,PrepGather2);
//
//        finishGatherPath2 = simplePath(PrepGather2, FinishGather2);
        prepGatherPath2 = drive.follower.pathBuilder()

                .addPath(new BezierLine(ShootPose1, PrepGather3))
                .setLinearHeadingInterpolation(ShootPose1.getHeading(), PrepGather3.getHeading())
                .addPath(new BezierLine(PrepGather3, FinishGather3))
                .setLinearHeadingInterpolation(PrepGather3.getHeading(), FinishGather3.getHeading())
                .build();

        GatePath = drive.follower.pathBuilder()

                .addPath(new BezierLine(FinishGather2, GatePassby))
                .setTValueConstraint(0.90)
                .setLinearHeadingInterpolation(FinishGather2.getHeading(), GatePassby.getHeading())
                .addPath(new BezierLine(GatePassby, GatePose))
                .setLinearHeadingInterpolation(GatePassby.getHeading(), GatePose.getHeading())
                .build();

        Shootpath3 = simplePath(FinishGather3,ShootPose2);

//        Shootpath3 =     drive.follower.pathBuilder()
//                .addPath(new BezierLine(GatePose, Shoot2passby))
//                .setLinearHeadingInterpolation(GatePose.getHeading(), Shoot2passby.getHeading())
//                .addPath(new BezierLine(Shoot2passby, ShootPose2))
//                .setLinearHeadingInterpolation(Shoot2passby.getHeading(), ShootPose2.getHeading())
//                .build();

//        prepGatherPath3 = simplePath(ShootPose2,PrepGather3);
//
//        finishGatherPath3 = simplePath(PrepGather3,FinishGather3);
        prepGatherPath3 = drive.follower.pathBuilder()

                .addPath(new BezierLine(ShootPose2, PrepGather2))
                .setLinearHeadingInterpolation(ShootPose2.getHeading(), PrepGather2.getHeading())
                .addPath(new BezierLine(PrepGather2, FinishGather2))
                .setLinearHeadingInterpolation(PrepGather3.getHeading(), FinishGather3.getHeading())
                .build();

        Shootpath4 = simplePath(FinishGather2,ShootPose2);

//        prepGatherPath4 = simplePath(ShootPose2,PrepGather4);
//
//        finishGatherPath4 = simplePath(PrepGather4,FinishGather4);
        prepGatherPath4 = drive.follower.pathBuilder()

                .addPath(new BezierLine(ShootPose2, PrepGather1))
                .setLinearHeadingInterpolation(ShootPose2.getHeading(), PrepGather1.getHeading())
                .setBrakingStrength(1.1)
                .addPath(new BezierLine(PrepGather1, FinishGather1))
                .setLinearHeadingInterpolation(PrepGather1.getHeading(), FinishGather1.getHeading())
                .build();

        Shootpath5 = simplePath(FinishGather1,ShootPose1);

        prepGatherPath6 = simplePath(ShootPose1,PrepGather4);
        finishGatherPath6 = simplePath(PrepGather4,FinishGather5);

        Shootpath6 = simplePath(FinishGather5,ShootPose1);

        lastOutPath = simplePath(ShootPose1,Park);
//
//        lastOutPath = drive.follower.pathBuilder()
//                .addPath(new BezierLine(ShootPose1, endPose))
//                .setLinearHeadingInterpolation(ShootPose1.getHeading(), endPose.getHeading())
//                .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                shooter.offset = 10;
                fulltimer.resetTimer();
                //shooter.autoLonger = false;
                shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                //drive.follower.followPath(Shootpath1,true);
                shooter.autoLonger = true;
                shooter.Autolong = 3160;
                //turret.autopos = 313;
                setPathState(1);

                break;
            case 1:
                if(!drive.follower.isBusy()) {
                    if (!firstshooting) {
                        shooter.updateFocused(true);

                        timer.resetTimer();
                        firstshooting = true;
                    }
                    else{
                        if(timer.getElapsedTimeSeconds()<(shoottime+0.5)){
                            shooter.setShooterStatus(Shooter.ShooterStatus.Shooting);
                        }
                        if(shooter.getTransDis()>18){
                            checkcounter -=1;
                        }
                        else{
                            checkcounter = checkcount;
                        }
                        if(checkcounter<0||timer.getElapsedTimeSeconds()> shoottime){
                            shooter.setShooterStatus(Shooter.ShooterStatus.Stop);
                            intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                            setPathState(2);
                        }

                    }
                    break;

                }
                break;
            //1st shooting________________________________________________
            case 2:
                if(!drive.follower.isBusy()) {
                    //turret.autopos = 319;
                    shooter.Autolong = 3135;
                    firstshooting = false;
                    shooter.setShooterStatus(Shooter.ShooterStatus.Stop);
                    intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                    shooter.periodic();
                    drive.follower.followPath(prepGatherPath1);
                    setPathState(3);
                }
                break;
            case 3:
                if(!drive.follower.isBusy()) {
                    drive.follower.followPath(finishGatherPath1);
                    setPathState(4);
                }
                break;
            case 4:
                if(!drive.follower.isBusy()){
                    shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                    //intake.setIntakeState(Intake.IntakeTransferState.Intake_Steady);
                    drive.follower.followPath(Shootpath2);
                    setPathState(5);
                }
                break;
            case 5:
                if(!drive.follower.isBusy()) {
                    if (!firstshooting) {
                        shooter.updateFocused(true);

                        timer.resetTimer();
                        firstshooting = true;
                    }
                    else{
                        if(timer.getElapsedTimeSeconds()>waittime&&timer.getElapsedTimeSeconds()<shoottime){
                            shooter.setShooterStatus(Shooter.ShooterStatus.Shooting);
                        }
                        if(shooter.getTransDis()>18){
                            checkcounter -=1;
                        }
                        else{
                            checkcounter = checkcount;
                        }
                        if(checkcounter<0||timer.getElapsedTimeSeconds()> shoottime){
                            shooter.setShooterStatus(Shooter.ShooterStatus.Stop);
                            intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                            setPathState(16);
                        }

                    }
                    break;

                }
                break;

            //2th shooting________________________________________________
            case 16:
                if(!drive.follower.isBusy()) {
                    //turret.autopos = 319;
                    firstshooting = false;
                    shooter.setShooterStatus(Shooter.ShooterStatus.Stop);
                    intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                    shooter.periodic();
                    drive.follower.followPath(prepGatherPath4);
                    setPathState(17);
                }
                break;
            case 17:
                if(!drive.follower.isBusy()) {
                    //drive.follower.followPath(finishGatherPath4);
                    setPathState(18);
                }
                break;
            case 18:
                if(!drive.follower.isBusy()){
                    intake.setIntakeState(Intake.IntakeTransferState.Intake_Steady);
                    shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                    drive.follower.followPath(Shootpath5);
                    setPathState(19);
                    shooter.autoLonger = true;
                }
                break;
            case 19:
                if(!drive.follower.isBusy()) {
                    if (!firstshooting) {
                        shooter.updateFocused(true);

                        timer.resetTimer();
                        firstshooting = true;
                    }
                    else{
                        if(timer.getElapsedTimeSeconds()>waittime&&timer.getElapsedTimeSeconds()<shoottime){
                            shooter.setShooterStatus(Shooter.ShooterStatus.Shooting);
                        }
                        if(shooter.getTransDis()>18){
                            checkcounter -=1;
                        }
                        else{
                            checkcounter = checkcount;
                        }
                        if(checkcounter<0||timer.getElapsedTimeSeconds()> shoottime){
                            shooter.setShooterStatus(Shooter.ShooterStatus.Stop);
                            intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                            setPathState(20);
                        }

                    }
                    break;

                }
                break;
            //5th shooting________________________________________________
            case 20:
                if(!drive.follower.isBusy()) {
                    //turret.autopos = 319;
                    shooter.Autolong = 3135;
                    firstshooting = false;
                    shooter.setShooterStatus(Shooter.ShooterStatus.Stop);
                    intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                    shooter.periodic();
                    drive.follower.followPath(prepGatherPath1);
                    setPathState(21);
                }
                break;
            case 21:
                if(!drive.follower.isBusy()) {
                    drive.follower.followPath(finishGatherPath1);
                    setPathState(22);
                }
                break;
            case 22:
                if(!drive.follower.isBusy()){
                    shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                    //intake.setIntakeState(Intake.IntakeTransferState.Intake_Steady);
                    drive.follower.followPath(Shootpath2);
                    setPathState(23);
                }
                break;
            case 23:
                if(!drive.follower.isBusy()) {
                    if (!firstshooting) {
                        shooter.updateFocused(true);

                        timer.resetTimer();
                        firstshooting = true;
                    }
                    else{
                        if(timer.getElapsedTimeSeconds()>waittime&&timer.getElapsedTimeSeconds()<shoottime){
                            shooter.setShooterStatus(Shooter.ShooterStatus.Shooting);
                        }
                        if(shooter.getTransDis()>18){
                            checkcounter -=1;
                        }
                        else{
                            checkcounter = checkcount;
                        }
                        if(checkcounter<0||timer.getElapsedTimeSeconds()> shoottime){
                            shooter.setShooterStatus(Shooter.ShooterStatus.Stop);
                            intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                            setPathState(30);
                        }

                    }
                    break;

                }
                break;


            case 30:
                if(!drive.follower.isBusy()) {
                    //turret.autopos = 319;
                    shooter.Autolong = 3135;
                    firstshooting = false;
                    shooter.setShooterStatus(Shooter.ShooterStatus.Stop);
                    intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                    shooter.periodic();
                    drive.follower.followPath(prepGatherPath1);
                    setPathState(31);
                }
                break;
            case 31:
                if(!drive.follower.isBusy()) {
                    drive.follower.followPath(finishGatherPath1);
                    setPathState(32);
                }
                break;
            case 32:
                if(!drive.follower.isBusy()){
                    shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                    //intake.setIntakeState(Intake.IntakeTransferState.Intake_Steady);
                    drive.follower.followPath(Shootpath2);
                    setPathState(33);
                }
                break;
            case 33:
                if(!drive.follower.isBusy()) {
                    if (!firstshooting) {
                        shooter.updateFocused(true);

                        timer.resetTimer();
                        firstshooting = true;
                    }
                    else{
                        if(timer.getElapsedTimeSeconds()>waittime&&timer.getElapsedTimeSeconds()<shoottime){
                            shooter.setShooterStatus(Shooter.ShooterStatus.Shooting);
                        }
                        if(shooter.getTransDis()>18){
                            checkcounter -=1;
                        }
                        else{
                            checkcounter = checkcount;
                        }
                        if(checkcounter<0||timer.getElapsedTimeSeconds()> shoottime){
                            shooter.setShooterStatus(Shooter.ShooterStatus.Stop);
                            intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                            setPathState(26);
                        }

                    }
                    break;

                }
                break;





            case 26:
                if(!drive.follower.isBusy()) {
                    //turret.autopos = 0;
                    shooter.setShooterStatus(Shooter.ShooterStatus.Stop);
                    intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                    shooter.periodic();
                    drive.follower.followPath(simplePath(drive.follower.getPose(),Park));
                    resetSubsystemsForTeleop();
                    setPathState(27);
                }
                break;
            case 27:
                intake.autoIntakeUp = false;
                Drivetrain.lastPose = drive.follower.getPose();
                Drivetrain.TredFblue = false;
                if(!drive.follower.isBusy()){
                    resetSubsystemsForTeleop();
                    Drivetrain.lastPose = drive.follower.getPose();
                    Drivetrain.TredFblue = false;
                    //setPathState(28);
                    break;

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


    /** This method is called once at the init of the OpMode. **/
    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        // These loop the movements of the robot, these must be called continuously in order to work
        drive.follower.update();
        shooter.periodic();
        turret.periodic();

        intake.periodic();
        // turret.automode = true;
//        if(shooter.autoLonger){
//            //turret.autopos = -195;
//        }
//        else{
//            //turret.autopos = -138;
//        }
        shooter.idleSpeed = 3200;
        shooter.forceShooting = true;
        if(shooter.shooterStatus == Shooter.ShooterStatus.Shooting){
            intake.updateAutoshoot(true);
            intake.isFarTeleMode = shooter.isAtFar();
//            if(shooter.reverIntake){
//                intake.updateAutoshoot(false);
//                intake.setIntakeState(Intake.IntakeTransferState.Split_Out);
//            }
            intake.updateautotranse(shooter.isAtTargetRPM());
//            shooter.updateDis(limelight.getDis());
//            shooter.updateFocused(limelight.isFocused());
            //shooter.updateFocused(true);
        }
        else{
            intake.updateAutoshoot(false);

        }
        if(shooter.shooterStatus != Shooter.ShooterStatus.Stop){

            shooter.ododis = drive.getdis_TWO();
            turret.aimangle = drive.getturretangle()+toRadians(turretoff);

            turret.updateAutoShoot(true);
            //turret.tx = limelight.getTx();

        }
        else{
            turret.updateAutoShoot(false);
        }
        autonomousPathUpdate();

//        // Feedback to Driver Hub for debugging
//        telemetry.addData("turret target", turret.currentpos);
//        telemetry.addData("turret aim", turret.aimposition);
//        telemetry.addData("Shooter Target RPM", shooter.getTargetRPM());
//        telemetry.addData("Shooter Current RPM", shooter.getFlyWheelRPM());
//        telemetry.addData("x", drive.follower.getPose().getX());
//        telemetry.addData("y", drive.follower.getPose().getY());
//        telemetry.addData("heading", drive.follower.getPose().getHeading());
//        telemetry.addData("timer", timer.getElapsedTimeSeconds());
//        telemetry.addData("shooter state", shooter.shooterStatus);
//        telemetry.addData("intake state", intake.intakeCurrentState);
//        telemetry.update();
    }


    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        pathTimer = new Timer();
        timer = new Timer();
        //gatetimer = new Timer();
        fulltimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        drive = new Drivetrain(hardwareMap,false);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        //intake.setIntakeState(Intake.IntakeTransferState.Intake_Steady);
        shooter.setShooterStatus(Shooter.ShooterStatus.Stop);
        turret = new Turret(hardwareMap,false);
        buildPaths();
        //drive.follower.setStartingPose(startPose);
        drive.follower.setPose(startPose);
        drive.redinit();
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
        Drivetrain.TredFblue = true;
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