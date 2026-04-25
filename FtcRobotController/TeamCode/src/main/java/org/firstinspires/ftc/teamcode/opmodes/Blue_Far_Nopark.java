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
@Autonomous(name = "Blue_Far_Nopark")

public class Blue_Far_Nopark extends OpMode {

    private Follower follower;
    private Drivetrain drive;
    private Timer pathTimer, actionTimer, opmodeTimer, timer,fulltimer;
    //private final ElapsedTime timer  = new ElapsedTime();

    private int pathState = 0;
    private final Pose startPose = new Pose(80.77, -0.65, 0); // Start Pose of our robot.
    private Pose ShootPose1 = new Pose(79.06, -10.98, -0.2);
    private final Pose PrepGather1 = new Pose(91.9908, -28.6053, 0);
    private final Pose FinishGather1 = new Pose(114.9794, -28.6053, 0);

    private final Pose PrepGather2 = new Pose(91.9908, -52.0297-5, 0);

    private final Pose FinishGather2 = new Pose(114.9794, -52.0297, 0);
    private final Pose GatePassby = new Pose(112.6299, -59.2147, 0);
    private final Pose GatePose = new Pose(122.5,-59.2147, 0);
    private final Pose ShootPose2 = new Pose(84.1620, -75.80 ,0);

    private final Pose Shoot2passby = new Pose(95.7309,-59.2147,0);

    private final Pose PrepGather3 = new Pose(91.9908, -75.8070+8, 0);//accounted for overshoot

    private final Pose FinishGather3 = new Pose(114.9794, -75.8070, 0);

    private final Pose PrepGather4 = new Pose(120.5531, -2.5016, 0);//accounted for overshoot

    private final Pose FinishGather4 = new Pose(122.52, -2.5016, 0);

    private final Pose Park = new Pose(115.21, -8.7571,0);;

    private final Pose FinishGather5 = new Pose(121.42, 0.06455, 0);

    private boolean firstshooting = false;
    private PathChain GatePath, Shootpath1,Shootpath2, Shootpath3,Shootpath4,Shootpath5, lastOutPath;
    private PathChain prepGatherPath6,prepGatherPath1,finishGatherPath6,Shootpath6, prepGatherPath2, prepGatherPath3, prepGatherPath4;

    private PathChain finishGatherPath1,finishGatherPath2,finishGatherPath3,finishGatherPath4;
    public Intake intake;
    public Shooter shooter;
    public MyLimelight limelight;
    public Scheduler scheduler;

    public Turret turret;

    public double turretoff = 0;

    public static double stoptime = 2.8;
    public static double shoottime = 1.2;
    public static double xpos = 129.67;
    public static double ypos = 134.01;

    public static double angle = 0;

    public static double waittime = 0.8;
    public static double intaketime = 0.7;
    public static double checkcount = 5;

    public static double followingtime = 1.5;

    public double checkcounter = checkcount;

    public double cyclecounter = 5;

    public boolean autoflag = false;


    public  PathChain simpleConstPath(Pose a, Pose b){
        return drive.follower.pathBuilder()
                .addPath(new BezierLine(a, b))
                .setConstantHeadingInterpolation(b.getHeading())
                .setTValueConstraint(0.99)
                .setBrakingStrength(0.7)
                .build();
    }
    public  PathChain simplePath(Pose a, Pose b){
        return drive.follower.pathBuilder()
                .addPath(new BezierLine(a, b))


                .setLinearHeadingInterpolation(a.getHeading(), b.getHeading())
                .setBrakingStrength(1)
                .setTValueConstraint(0.95)
                .build();
    }
    public void buildPaths() {

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        Shootpath1 = simplePath(startPose,ShootPose1);

        //prepGatherPath1 = simplePath(ShootPose1,PrepGather4);
        prepGatherPath1 = simplePath(startPose,PrepGather4);
        finishGatherPath1 = drive.follower.pathBuilder()
                //   .setTValueConstraint(0.98)

                .addPath(new BezierLine(PrepGather4, FinishGather4))
                .setLinearHeadingInterpolation(PrepGather4.getHeading(), FinishGather4.getHeading())
                .addPath(new BezierLine(FinishGather4,PrepGather4))
                .setLinearHeadingInterpolation(FinishGather4.getHeading(), PrepGather4.getHeading())

                .addPath(new BezierLine(PrepGather4, FinishGather4))
                .setLinearHeadingInterpolation(PrepGather4.getHeading(), FinishGather4.getHeading())
                .setBrakingStrength(1)
                .setTValueConstraint(0.95)
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
//
//        finishGatherPath4 = simplePath(PrepGather4,FinishGather4);
        prepGatherPath4 = drive.follower.pathBuilder()

                .addPath(new BezierLine(ShootPose1, PrepGather1))
                .setLinearHeadingInterpolation(ShootPose1.getHeading(), PrepGather1.getHeading())
                //.setBrakingStrength(1.1)
                .addPath(new BezierLine(PrepGather1, FinishGather1))
                .setLinearHeadingInterpolation(PrepGather1.getHeading(), FinishGather1.getHeading())
                .setBrakingStrength(1)
                .setTValueConstraint(0.95)
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
                //  shooter.offset = 10;
                fulltimer.resetTimer();
                //shooter.autoLonger = false;
                shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                //drive.follower.followPath(Shootpath1,true);
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
                        if(timer.getElapsedTimeSeconds()<(shoottime+2)){
                            shooter.setShooterStatus(Shooter.ShooterStatus.Shooting);
                        }
                        if((!intake.hasballCheck(1))&&(!intake.hasballCheck(2))&&(!intake.hasballCheck(3))){
                            checkcounter -=1;
                        }
                        else{
                            checkcounter = checkcount;
                        }
                        if(checkcounter<0||timer.getElapsedTimeSeconds()> shoottime+2){
                            shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
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
                    firstshooting = false;
                    shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
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
                        if((!intake.hasballCheck(1))&&(!intake.hasballCheck(2))&&(!intake.hasballCheck(3))){
                            checkcounter -=1;
                        }
                        else{
                            checkcounter = checkcount;
                        }
                        if(checkcounter<0||timer.getElapsedTimeSeconds()> shoottime){
                            shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
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
                    shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
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
                    //shooter.autoLonger = true;
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
                        if((!intake.hasballCheck(1))&&(!intake.hasballCheck(2))&&(!intake.hasballCheck(3))){
                            checkcounter -=1;
                        }
                        else{
                            checkcounter = checkcount;
                        }
                        if(checkcounter<0||timer.getElapsedTimeSeconds()> shoottime){
                            shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
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
//                    drive.follower.followPath(lastOutPath);
                    setPathState(21);
                }
                break;
            case 21:
                if(!drive.follower.isBusy()){
                    setPathState(22);
                }
                break;
            case 22:
                if(!drive.follower.isBusy()) {
                    intake.autoIntakeUp = true;
                    firstshooting = false;
                    shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                    intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                    shooter.periodic();
                    //drive.follower.followPath(prepGatherPath6);
                    //drive.follower.breakFollowing();
                    firstshooting = false;

                    setPathState(23);
                }
                break;
            case 23:
                if(!drive.follower.isBusy()) {
                    if (!firstshooting) {
                        timer.resetTimer();
                        firstshooting = true;
                        break;
                    }
                    else {

                        if (timer.getElapsedTimeSeconds() < followingtime&&checkcounter>0) {
                            if((intake.hasballCheck(1))&&(intake.hasballCheck(2))&&(intake.hasballCheck(3))){
                                checkcounter -=1;
                            }
                            else{
                                checkcounter = checkcount;
                            }
                            if(!drive.follower.isTeleopDrive()) {
                                drive.follower.breakFollowing();
                                drive.follower.startTeleopDrive();


                            }
                            if(drive.follower.getPose().getX()<120&&drive.follower.getPose().getY()>-55) {
//                                if(drive.follower.getPose().getY()>-15){
//                                    drive.teleDrive(0.4, 0, LimelightLockInCommand.Kp * limelight.getpatterTx()/(16+drive.follower.getPose().getY()));
//                                }
//                                else {
                                drive.teleDrive(0.4, 0, LimelightLockInCommand.Kp * limelight.getpatterTx());
//                                }
                            }
                            else{
                                drive.teleDrive(0, 0, 0);
                            }
                            autoflag = true;
                        }
                        else{
                            firstshooting = false;
                            autoflag = false;
//                            teleDrive(0, 0,
//                                    0);

                            drive.follower.breakFollowing();
                            checkcounter = checkcount;
                            intake.setIntakeState(Intake.IntakeTransferState.Suck_In_slow);
                            shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                            if(ShootPose1.getHeading() == -0.2){
                                ShootPose1 = new Pose(ShootPose1.getX(),ShootPose1.getY(),0);
                            }
                            else{
                                ShootPose1 = new Pose(ShootPose1.getX(),ShootPose1.getY(),-0.2);;
                            }
                            if(fulltimer.getElapsedTimeSeconds()<28.5) {
                                checkcounter = checkcount;
                                drive.follower.followPath(simpleConstPath(drive.follower.getPose(),ShootPose1));
                                setPathState(24);
                            }
                            else{
                                checkcounter = checkcount;
                                setPathState(26);
                            }
                            break;
                        }
                    }
                }
                break;
            case 24:
                if(!drive.follower.isBusy()) {
                    if (!firstshooting) {
                        shooter.updateFocused(true);

                        timer.resetTimer();
                        firstshooting = true;
                    }
                    else{
                        if(timer.getElapsedTimeSeconds()<(shoottime)){
                            shooter.setShooterStatus(Shooter.ShooterStatus.Shooting);
                        }
                        if((!intake.hasballCheck(1))&&(!intake.hasballCheck(2))&&(!intake.hasballCheck(3))){
                            checkcounter -=1;
                        }
                        else{
                            checkcounter = checkcount;
                        }
                        if(checkcounter<0||timer.getElapsedTimeSeconds()> shoottime){
                            shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                            intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                            firstshooting = false;
                            if(fulltimer.getElapsedTimeSeconds()<28.5) {
                                checkcounter = checkcount;
                                setPathState(22);
                            }
                            else{
                                checkcounter = checkcount;
                                setPathState(26);
                            }
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
        limelight.periodic();
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
        limelight = new MyLimelight(hardwareMap);
        limelight.initPatternPipeline();
        limelight.startDetect();
        //intake.setIntakeState(Intake.IntakeTransferState.Intake_Steady);
        shooter.setShooterStatus(Shooter.ShooterStatus.Stop);
        turret = new Turret(hardwareMap,false);
        buildPaths();
        //drive.follower.setStartingPose(startPose);
        drive.follower.setPose(startPose);
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