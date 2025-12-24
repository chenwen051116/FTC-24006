package org.firstinspires.ftc.teamcode.opmodes; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
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

@Autonomous(name = "RED_Near_12ball_gate")
public class RED_Near_12ball extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer, timer;
    //private final ElapsedTime timer  = new ElapsedTime();

    private int pathState = 0;
    private final Pose startPose = new Pose(119.7571, 107.4503, 0); // Start Pose of our robot.
    private final Pose ShootPoseFar = new Pose(81.21, 8.7571,0);
    private final Pose PrepGather1 = new Pose(91.9908, 28.6053+5, 0);
    private final Pose FinishGather1 = new Pose(114.9794, 28.6053, 0);

    private final Pose PrepGather2 = new Pose(91.9908, 52.0297+5, 0);

    private final Pose FinishGather2 = new Pose(114.9794, 52.0297, 0);
    private final Pose GatePassby = new Pose(112.6299, 59.2147, 0);
    private final Pose GatePickupPose = new Pose(121.8784,52.0519, 0.6185);
    private final Pose ShootPoseNear = new Pose(84.1620, 75.80 ,0);

    private final Pose Shoot2passby = new Pose(95.7309,59.2147,0);

    private final Pose PrepGather3 = new Pose(91.9908, 75.8070, 0);//accounted for overshoot

    private final Pose FinishGather3 = new Pose(114.9794, 75.8070, 0);

    private final Pose PrepGather4 = new Pose(119.5531, -0.06455, 0);//accounted for overshoot

    private final Pose FinishGather4 = new Pose(123.42, -0.06455, 0);

    private final Pose Park = new Pose(121.21, 20.7571,0);

    private final Pose ShootPoseNear_2 = new Pose(79.7538, 97.3437,0);
    private boolean firstshooting = false;
    private PathChain Shootpath0,Shootpath1, Shootpath2, Shootpath3,Shootpathcycle, lastOutPath;
    private PathChain prepGatherPath1, prepGatherPath2, prepGatherPath3,prepGatherPathcycle;
    public Intake intake;
    public Shooter shooter;
    public MyLimelight limelight;
    public Scheduler scheduler;

    public Turret turret;

    public static double shoottime = 1.65;
    public static double xpos = 126.67;
    public static double ypos = 129.01;

    public static double angle = 0;

    public static double waittime = 0.5;
    public static double intaketime = 2;

    public static int cycle = 3;

    public void buildPaths() {

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        Shootpath0 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, ShootPoseNear))
                .setLinearHeadingInterpolation(startPose.getHeading(), ShootPoseNear.getHeading())

                .build();

        prepGatherPath1 = follower.pathBuilder()
                .addPath(new BezierLine(ShootPoseNear, PrepGather2))
                .setLinearHeadingInterpolation(ShootPoseNear.getHeading(), PrepGather2.getHeading())
                .addPath(new BezierLine(PrepGather2, FinishGather2))
                .setLinearHeadingInterpolation(PrepGather2.getHeading(), FinishGather2.getHeading())
                .addPath(new BezierLine(FinishGather2, ShootPoseNear))
                .setLinearHeadingInterpolation(FinishGather2.getHeading(), ShootPoseNear.getHeading())
                .build();
        Shootpath1 = follower.pathBuilder()

                .addPath(new BezierLine(FinishGather2, ShootPoseNear))
                .setLinearHeadingInterpolation(FinishGather2.getHeading(), ShootPoseNear.getHeading())
                .build();

        prepGatherPathcycle = follower.pathBuilder()
                .addPath(new BezierLine(ShootPoseNear, PrepGather1))
                .setLinearHeadingInterpolation(ShootPoseNear.getHeading(), PrepGather1.getHeading())
                .addPath(new BezierLine(PrepGather1, GatePickupPose))
                .setLinearHeadingInterpolation(PrepGather1.getHeading(), GatePickupPose.getHeading())
                .build();
        Shootpathcycle = follower.pathBuilder()
                .addPath(new BezierLine(GatePickupPose, PrepGather1))
                .setLinearHeadingInterpolation(GatePickupPose.getHeading(), PrepGather1.getHeading())
                .addPath(new BezierLine(PrepGather1, ShootPoseNear))
                .setLinearHeadingInterpolation(PrepGather1.getHeading(), ShootPoseNear.getHeading())
                .build();



        Shootpath2 = follower.pathBuilder()

                .addPath(new BezierLine(FinishGather3, ShootPoseNear))
                .setLinearHeadingInterpolation(FinishGather3.getHeading(), ShootPoseNear.getHeading())
                .build();

        prepGatherPath2 = follower.pathBuilder()
                .addPath(new BezierLine(ShootPoseNear, PrepGather3))
                .setLinearHeadingInterpolation(ShootPoseNear.getHeading(), PrepGather3.getHeading())
                .addPath(new BezierLine(PrepGather3, FinishGather3))
                .setLinearHeadingInterpolation(PrepGather3.getHeading(), FinishGather3.getHeading())
                .setBrakingStrength(1.5)
                .build();


        Shootpath3 = follower.pathBuilder()

                .addPath(new BezierLine(FinishGather1, ShootPoseNear_2))
                .setLinearHeadingInterpolation(FinishGather1.getHeading(), ShootPoseNear_2.getHeading())
                .build();

        prepGatherPath3 = follower.pathBuilder()
                .addPath(new BezierLine(ShootPoseNear, PrepGather1))
                .setLinearHeadingInterpolation(ShootPoseNear.getHeading(), PrepGather1.getHeading())
                .addPath(new BezierLine(PrepGather1, FinishGather1))
                .setLinearHeadingInterpolation(PrepGather1.getHeading(), FinishGather1.getHeading())
                .setBrakingStrength(1.5)
                .build();
//        lastOutPath = follower.pathBuilder()
//
//                .addPath(new BezierLine(ShootPose1, Park))
//                .setLinearHeadingInterpolation(ShootPose1.getHeading(), Park.getHeading())
//                .build();
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
                follower.followPath(Shootpath0,true);
                shooter.autoLonger = false;
                turret.autopos = -138;
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
                        if(timer.getElapsedTimeSeconds()<(shoottime)){
                            shooter.setShooterStatus(Shooter.ShooterStatus.Shooting);
                        }
                        if(shooter.getTransDis()>18||timer.getElapsedTimeSeconds()> (shoottime)){
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
                    turret.autopos = -138;
                    //shooter.Autolong = 3100;
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
                    //follower.followPath(finishGatherPath1s);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()){
                    shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                    follower.followPath(Shootpath1);
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
            //cycle shooting________________________________________________
            case 6:
                if(!follower.isBusy()) {
                    firstshooting = false;
                    turret.autopos = -138;
                    shooter.setShooterStatus(Shooter.ShooterStatus.Stop);
                    intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                    shooter.periodic();
                    follower.followPath(prepGatherPathcycle);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    //follower.followPath(finishGatherPath2);

                    setPathState(8);
                }
                break;
            case 8:
                if(!follower.isBusy()){
//                    intake.setIntakeState(Intake.IntakeTransferState.Intake_Steady);
//                    follower.followPath(GatePath);
////                    intake.gatepos = true;
////                    intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
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
                        if(timer.getElapsedTimeSeconds()> intaketime){
                            intake.gatepos = false;
                            shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                            follower.followPath(Shootpathcycle);
                            firstshooting = false;
                            setPathState(11);
                            //setPathState(23);

                            break;
                        }

                    }
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

            case 12:
                if(!follower.isBusy()) {
                    if(cycle>=0) {
                        cycle -= 1;
                        firstshooting = false;
                        setPathState(6);
                    }
                    else{
                        firstshooting = false;
                        setPathState(13);
                    }
                }
                break;
            case 13:
                if(!follower.isBusy()) {
                    follower.followPath(prepGatherPath2);
                    firstshooting = false;
                    setPathState(14);
                }
                break;
            case 14:
                if(!follower.isBusy()){
                    firstshooting = false;
                    shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                    follower.followPath(Shootpath2);
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
                    turret.autopos = -130;
                    firstshooting = false;
                    shooter.setShooterStatus(Shooter.ShooterStatus.Stop);
                    intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                    shooter.periodic();
                    follower.followPath(prepGatherPath3);
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
                    follower.followPath(Shootpath3);
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
            //shooter.ododis = getdis();
            turret.updateAutoShoot(true);
            turret.tx = limelight.getTx();
            //turret.aimangle = getturretangle();
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
    public void stop() {}
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