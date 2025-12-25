package org.firstinspires.ftc.teamcode.opmodes; // make sure this aligns with class location

import static java.lang.Math.abs;
import static java.lang.Math.sqrt;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MyLimelight;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Scheduler;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@Config
@Autonomous(name = "Red_Far_18ball_gate")

public class Red_Far_18ball_gate extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer, timer;
    //private final ElapsedTime timer  = new ElapsedTime();

    private int pathState = 0;
    private final Pose startPose = new Pose(81.21237, -0.2738, 0); // Start Pose of our robot.
    private final Pose ShootPose1 = new Pose(81.21, 8.7571,0);
    private final Pose PrepGather1 = new Pose(91.9908, 28.6053+5, 0);
    private final Pose FinishGather1 = new Pose(114.9794, 28.6053, 0);

    private final Pose PrepGather2 = new Pose(91.9908, 52.0297+5, 0);

    private final Pose FinishGather2 = new Pose(114.9794, 52.0297, 0);
    private final Pose GatePassby = new Pose(112.6299, 59.2147, 0);
    private final Pose GatePose = new Pose(122.5,59.2147, 0);
    private final Pose ShootPose2 = new Pose(84.1620, 75.80 ,0);

    private final Pose Shoot2passby = new Pose(95.7309,59.2147,0);

    private final Pose PrepGather3 = new Pose(91.9908, 75.8070-8, 0);//accounted for overshoot

    private final Pose FinishGather3 = new Pose(114.9794, 75.8070, 0);

    private final Pose PrepGather4 = new Pose(119.5531, -0.06455, 0);//accounted for overshoot

    private final Pose FinishGather4 = new Pose(123.42, -0.06455, 0);

    private final Pose Park = new Pose(121.21, 20.7571,0);;


    private boolean firstshooting = false;
    private PathChain GatePath, Shootpath1,Shootpath2, Shootpath3,Shootpath4,Shootpath5, lastOutPath;
    private PathChain prepGatherPath1, prepGatherPath2, prepGatherPath3, prepGatherPath4;

    private PathChain finishGatherPath1,finishGatherPath2,finishGatherPath3,finishGatherPath4;
    public Intake intake;
    public Shooter shooter;
    public MyLimelight limelight;
    public Scheduler scheduler;

    public Turret turret;

    public static double stoptime = 0.75;
    public static double shoottime = 1.65;
    public static double xpos = 126.67;
    public static double ypos = 129.01;

    public static double angle = 0;

    public static double waittime = 0.5;
    public static double intaketime = 2;


    public  PathChain simplePath(Pose a, Pose b){
        return follower.pathBuilder()
                .addPath(new BezierLine(a, b))
                .setLinearHeadingInterpolation(a.getHeading(), b.getHeading())
                .build();
    }
    public void buildPaths() {

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        Shootpath1 = simplePath(startPose,ShootPose1);

        //prepGatherPath1 = simplePath(ShootPose1,PrepGather4);
        prepGatherPath1 = simplePath(startPose,PrepGather4);
        finishGatherPath1 = simplePath(PrepGather4,FinishGather4);

//        prepGatherPath1 = follower.pathBuilder()
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
        prepGatherPath2 = follower.pathBuilder()

                .addPath(new BezierLine(ShootPose1, PrepGather3))
                .setLinearHeadingInterpolation(ShootPose1.getHeading(), PrepGather3.getHeading())
                .addPath(new BezierLine(PrepGather3, FinishGather3))
                .setLinearHeadingInterpolation(PrepGather3.getHeading(), FinishGather3.getHeading())
                .build();

        GatePath = follower.pathBuilder()

                .addPath(new BezierLine(FinishGather2, GatePassby))
                .setLinearHeadingInterpolation(FinishGather2.getHeading(), GatePassby.getHeading())
                .addPath(new BezierLine(GatePassby, GatePose))
                .setLinearHeadingInterpolation(GatePassby.getHeading(), GatePose.getHeading())
                .build();

         Shootpath3 = simplePath(FinishGather3,ShootPose2);

//        Shootpath3 =     follower.pathBuilder()
//                .addPath(new BezierLine(GatePose, Shoot2passby))
//                .setLinearHeadingInterpolation(GatePose.getHeading(), Shoot2passby.getHeading())
//                .addPath(new BezierLine(Shoot2passby, ShootPose2))
//                .setLinearHeadingInterpolation(Shoot2passby.getHeading(), ShootPose2.getHeading())
//                .build();

//        prepGatherPath3 = simplePath(ShootPose2,PrepGather3);
//
//        finishGatherPath3 = simplePath(PrepGather3,FinishGather3);
        prepGatherPath3 = follower.pathBuilder()

                .addPath(new BezierLine(ShootPose2, PrepGather2))
                .setLinearHeadingInterpolation(ShootPose2.getHeading(), PrepGather2.getHeading())
                .addPath(new BezierLine(PrepGather2, FinishGather2))
                .setLinearHeadingInterpolation(PrepGather3.getHeading(), FinishGather3.getHeading())
                .build();

        Shootpath4 = simplePath(FinishGather2,ShootPose2);

//        prepGatherPath4 = simplePath(ShootPose2,PrepGather4);
//
//        finishGatherPath4 = simplePath(PrepGather4,FinishGather4);
        prepGatherPath4 = follower.pathBuilder()

                .addPath(new BezierLine(ShootPose2, PrepGather1))
                .setLinearHeadingInterpolation(ShootPose2.getHeading(), PrepGather1.getHeading())
                .addPath(new BezierLine(PrepGather1, FinishGather1))
                .setLinearHeadingInterpolation(PrepGather1.getHeading(), FinishGather1.getHeading())
                .build();

        Shootpath5 = simplePath(FinishGather1,ShootPose1);

        lastOutPath = simplePath(ShootPose1,Park);
//
//        lastOutPath = follower.pathBuilder()
//                .addPath(new BezierLine(ShootPose1, endPose))
//                .setLinearHeadingInterpolation(ShootPose1.getHeading(), endPose.getHeading())
//                .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                //shooter.autoLonger = false;
                shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                //follower.followPath(Shootpath1,true);
                shooter.autoLonger = true;
                shooter.Autolong = 3125;
                turret.autopos = -190;
                setPathState(1);

                break;
            case 1:
                if(!follower.isBusy()) {
                    if (!firstshooting) {
                        shooter.updateFocused(true);

                        timer.resetTimer();
                        firstshooting = true;
                    }
                    else{
                        if(timer.getElapsedTimeSeconds()<(shoottime+0.5)){
                            shooter.setShooterStatus(Shooter.ShooterStatus.Shooting);
                        }
                        if(shooter.getTransDis()>18||timer.getElapsedTimeSeconds()> (shoottime+0.5)){
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
                if(!follower.isBusy()) {
                    turret.autopos = -195;
                    shooter.Autolong = 3100;
                    firstshooting = false;
                    shooter.setShooterStatus(Shooter.ShooterStatus.Stop);
                    intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                    shooter.periodic();
                    follower.followPath(prepGatherPath1);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    follower.followPath(finishGatherPath1);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()){
                    shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                    follower.followPath(Shootpath2);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    if (!firstshooting) {
                        shooter.updateFocused(true);

                        timer.resetTimer();
                        firstshooting = true;
                    }
                    else{
                        if(timer.getElapsedTimeSeconds()>waittime&&timer.getElapsedTimeSeconds()<shoottime){
                            shooter.setShooterStatus(Shooter.ShooterStatus.Shooting);
                        }
                        if(shooter.getTransDis()>18||timer.getElapsedTimeSeconds()> shoottime){
                            shooter.setShooterStatus(Shooter.ShooterStatus.Stop);
                            intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                            setPathState(6);
                        }

                    }
                    break;

                }
                break;
            //2nd shooting________________________________________________
            case 6:
                if(!follower.isBusy()) {
                    turret.autopos = -138;
                    shooter.setShooterStatus(Shooter.ShooterStatus.Stop);
                    intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                    shooter.periodic();
                    follower.followPath(prepGatherPath2);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    //follower.followPath(finishGatherPath2);

                    setPathState(10);
                }
                break;
            case 8:
                if(!follower.isBusy()){
                    intake.setIntakeState(Intake.IntakeTransferState.Intake_Steady);
                    follower.followPath(GatePath);
//                    intake.gatepos = true;
//                    intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                    firstshooting = false;
                    setPathState(9);
                }
                break;
            case 9:
                if(!follower.isBusy()){
                    if (!firstshooting) {
                        timer.resetTimer();
                        firstshooting = true;
                        break;
                    }
                    else{
                        if(timer.getElapsedTimeSeconds()> stoptime){
                            intake.gatepos = false;
                            shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                            follower.followPath(Shootpath4);
                            firstshooting = false;
                            setPathState(15);
                            //setPathState(23);

                            break;
                        }

                    }
                }
                break;
            case 10:
                if(!follower.isBusy()){
                    firstshooting = false;
                    shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                    follower.followPath(Shootpath3);
                    shooter.autoLonger = false;
                    setPathState(11);
                }
                break;



            case 11:
                if(!follower.isBusy()) {
                    if (!firstshooting) {
                        shooter.updateFocused(true);

                        timer.resetTimer();
                        firstshooting = true;
                    }
                    else{
                        if(timer.getElapsedTimeSeconds()>(waittime)&&timer.getElapsedTimeSeconds()<(shoottime+0.5)){
                            shooter.setShooterStatus(Shooter.ShooterStatus.Shooting);
                        }
                        if(shooter.getTransDis()>18||timer.getElapsedTimeSeconds()> (shoottime)){
                            shooter.setShooterStatus(Shooter.ShooterStatus.Stop);
                            intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                            setPathState(12);
                        }

                    }
                    break;

                }
                break;
            //3rd shooting________________________________________________
            case 12:
                if(!follower.isBusy()) {
                    firstshooting = false;
                    shooter.setShooterStatus(Shooter.ShooterStatus.Stop);
                    intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                    shooter.periodic();
                    follower.followPath(prepGatherPath3);
                    setPathState(8);
                }
                break;
            case 13:
                if(!follower.isBusy()) {
                    //follower.followPath(finishGatherPath3);
                    setPathState(14);
                }
                break;
            case 14:
                if(!follower.isBusy()){
                    shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                    follower.followPath(Shootpath4);
                    setPathState(15);
                }
                break;
            case 15:
                if(!follower.isBusy()) {
                    if (!firstshooting) {
                        shooter.updateFocused(true);

                        timer.resetTimer();
                        firstshooting = true;
                    }
                    else{
                        if(timer.getElapsedTimeSeconds()>waittime&&timer.getElapsedTimeSeconds()<shoottime){
                            shooter.setShooterStatus(Shooter.ShooterStatus.Shooting);
                        }
                        if(shooter.getTransDis()>18||timer.getElapsedTimeSeconds()> shoottime){
                            shooter.setShooterStatus(Shooter.ShooterStatus.Stop);
                            intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                            setPathState(16);
                        }

                    }
                    break;

                }
                break;
            //4th shooting________________________________________________
            case 16:
                if(!follower.isBusy()) {
                    turret.autopos = -195;
                    firstshooting = false;
                    shooter.setShooterStatus(Shooter.ShooterStatus.Stop);
                    intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                    shooter.periodic();
                    follower.followPath(prepGatherPath4);
                    setPathState(17);
                }
                break;
            case 17:
                if(!follower.isBusy()) {
                    //follower.followPath(finishGatherPath4);
                    setPathState(18);
                }
                break;
            case 18:
                if(!follower.isBusy()){
                    shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                    follower.followPath(Shootpath5);
                    setPathState(19);
                    shooter.autoLonger = true;
                }
                break;
            case 19:
                if(!follower.isBusy()) {
                    if (!firstshooting) {
                        shooter.updateFocused(true);

                        timer.resetTimer();
                        firstshooting = true;
                    }
                    else{
                        if(timer.getElapsedTimeSeconds()>waittime&&timer.getElapsedTimeSeconds()<shoottime){
                            shooter.setShooterStatus(Shooter.ShooterStatus.Shooting);
                        }
                        if(shooter.getTransDis()>18||timer.getElapsedTimeSeconds()> shoottime){
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
                if(!follower.isBusy()) {
//                    follower.followPath(lastOutPath);
                    setPathState(21);
                }
                break;
            case 21:
                if(!follower.isBusy()){
                    setPathState(22);
                }
                break;
            case 22:
                if(!follower.isBusy()) {
                    turret.autopos = -195;
                    shooter.Autolong = 3100;
                    firstshooting = false;
                    shooter.setShooterStatus(Shooter.ShooterStatus.Stop);
                    intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                    shooter.periodic();
                    follower.followPath(prepGatherPath1);
                    //follower.breakFollowing();
                    firstshooting = false;
                    setPathState(23);
                }
                break;
            case 23:

                if (!firstshooting) {
                    timer.resetTimer();
                    firstshooting = true;
                    break;
                }
                else{
                    if(timer.getElapsedTimeSeconds()> intaketime){
                        firstshooting = false;
                        follower.breakFollowing();
                        follower.followPath(finishGatherPath1);
                        setPathState(24);

                        break;
                    }

                }

                if(!follower.isBusy()) {
                    firstshooting = false;
                    follower.followPath(finishGatherPath1);
                    setPathState(24);
                }
                break;
            case 24:
                if(!follower.isBusy()){
                    shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                    follower.followPath(Shootpath2);
                    setPathState(25);
                }
                break;
            case 25:
                if(!follower.isBusy()) {
                    if (!firstshooting) {
                        shooter.updateFocused(true);

                        timer.resetTimer();
                        firstshooting = true;
                    }
                    else{
                        if(timer.getElapsedTimeSeconds()>waittime&&timer.getElapsedTimeSeconds()<shoottime){
                            shooter.setShooterStatus(Shooter.ShooterStatus.Shooting);
                        }
                        if(shooter.getTransDis()>18||timer.getElapsedTimeSeconds()> shoottime){
                            shooter.setShooterStatus(Shooter.ShooterStatus.Stop);
                            intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                            setPathState(26);
                        }

                    }
                    break;

                }
                break;
            //2nd shooting________________________________________________
            case 26:
                if(!follower.isBusy()) {
                    turret.autopos = 0;
                    shooter.setShooterStatus(Shooter.ShooterStatus.Stop);
                    intake.setIntakeState(Intake.IntakeTransferState.Intake_Steady);
                    shooter.periodic();
                    follower.followPath(lastOutPath);
                    resetSubsystemsForTeleop();
                    setPathState(27);
                }
                break;
            case 27:
                Drivetrain.lastPose = follower.getPose();
                Drivetrain.TredFblue = true;
                if(!follower.isBusy()){
                    resetSubsystemsForTeleop();
                    Drivetrain.lastPose = follower.getPose();
                    Drivetrain.TredFblue = true;
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

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        shooter.periodic();
        turret.periodic();
        limelight.periodic();
        intake.periodic();
        turret.automode = true;
//        if(shooter.autoLonger){
//            turret.autopos = -195;
//        }
//        else{
//            turret.autopos = -138;
//        }
        shooter.forceShooting = true;
        if(shooter.shooterStatus == Shooter.ShooterStatus.Shooting){
            intake.updateAutoshoot(true);
//            if(shooter.reverIntake){
//                intake.updateAutoshoot(false);
//                intake.setIntakeState(Intake.IntakeTransferState.Split_Out);
//            }
            intake.updateautotranse(shooter.isAtTargetRPM());
            shooter.updateDis(limelight.getDis());
            shooter.updateFocused(limelight.isFocused());
            //shooter.updateFocused(true);



        }
        else{
            intake.updateAutoshoot(false);

        }

        if(shooter.shooterStatus != Shooter.ShooterStatus.Stop){
            shooter.ododis = getdis();
            turret.updateAutoShoot(true);
            turret.tx = limelight.getTx();
            turret.aimangle = getturretangle();
        }
        else{
            turret.updateAutoShoot(false);
        }
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("realRPM", shooter.getFlyWheelRPM());
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("timer", timer.getElapsedTimeSeconds());
        telemetry.addData("shooter state", shooter.shooterStatus);
        telemetry.addData("intake state", intake.intakeCurrentState);
        telemetry.update();
    }

    private double getdis(){
        double x = follower.getPose().getX()-xpos;
        double y = follower.getPose().getY()-ypos;
        return sqrt(x*x+y*y);
    }
    private double getturretangle(){
//        double x = follower.getPose().getX()-aimPos.getX();
//        double y = follower.getPose().getY()-aimPos.getY();
        double x = follower.getPose().getX()-xpos;
        double y = follower.getPose().getY()-ypos;
        double h = follower.getPose().getHeading()+angle;
        //       if(!TredFblue) {
        if (y < 0) {
            return 1 * h - Math.atan(abs(y) / abs(x));
        } else {
            return 1 * h + Math.atan(abs(y) / abs(x));
        }

    }
    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        timer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        shooter.automode = true;
        limelight = new MyLimelight(hardwareMap);
        limelight.initRedPipeline();
        limelight.startDetect();
        intake.setIntakeState(Intake.IntakeTransferState.Intake_Steady);
        shooter.setShooterStatus(Shooter.ShooterStatus.Stop);
        turret = new Turret(hardwareMap);
        buildPaths();
        //follower.setStartingPose(startPose);
        follower.setPose(startPose);
        turret.automode = true;

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
        Drivetrain.lastPose = follower.getPose();
        Drivetrain.TredFblue = true;
        //setPathState(28);
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