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
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Scheduler;

@Autonomous(name = "RED_Near_12ball_gate")
public class RED_Near_12ball extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer, timer;
    //private final ElapsedTime timer  = new ElapsedTime();

    private int pathState = 0;
    private final Pose startPose = new Pose(112.522,112.651,0.701); // Start Pose of our robot.
    private final Pose PrepGather1 = new Pose(88.880,70.346 ,0);
    private final Pose FinishGather1 = new Pose(111.989,70.346, 0);

    private final Pose PrepGather2 = new Pose(88.880,46.804, 0);

    private final Pose FinishGather2 = new Pose(111.989, 46.804, 0);
    private final Pose PrepGather3 = new Pose(88.880,22.586, 0);//accounted for overshoot
    private final Pose FinishGather3 = new Pose(111.989, 22.586, 0);
    private final Pose Park = new Pose(110.539 ,84.386, 1.060);

    private boolean firstshooting = false;
    private PathChain fGPath1,fGPath2,fGPath3 ,Shootpath0,Shootpath1, Shootpath2, Shootpath3,Shootpathcycle, lastOutPath;
    private PathChain prepGatherPath1, prepGatherPath2, prepGatherPath3,prepGatherPathcycle;
    public Intake intake;
    public Shooter shooter;
    public Scheduler scheduler;


    public static double shoottime = 1.5;
    public static double xpos = 126.67;
    public static double ypos = 129.01;

    public static double angle = 0;

    public static double waittime = 0.3;
    public static double intaketime = 1.5;

    public static int cycle = 2;

    public void buildPaths() {


        prepGatherPath1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, PrepGather1))
                .setLinearHeadingInterpolation(startPose.getHeading(), PrepGather1.getHeading())
                .build();

        fGPath1 = follower.pathBuilder()
                .addPath(new BezierLine(PrepGather1, FinishGather1))
                .setLinearHeadingInterpolation(PrepGather1.getHeading(), FinishGather1.getHeading())
                .build();
        Shootpath1 = follower.pathBuilder()

                .addPath(new BezierLine(FinishGather1, startPose))
                .setLinearHeadingInterpolation(FinishGather1.getHeading(), startPose.getHeading())
                .build();

        prepGatherPath2 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, PrepGather2))
                .setLinearHeadingInterpolation(startPose.getHeading(), PrepGather2.getHeading())
                .build();

        fGPath2 = follower.pathBuilder()
                .addPath(new BezierLine(PrepGather2, FinishGather2))
                .setLinearHeadingInterpolation(PrepGather2.getHeading(), FinishGather2.getHeading())
                .build();
        Shootpath2 = follower.pathBuilder()

                .addPath(new BezierLine(FinishGather2, startPose))
                .setLinearHeadingInterpolation(FinishGather2.getHeading(), startPose.getHeading())
                .build();


        prepGatherPath3 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, PrepGather3))
                .setLinearHeadingInterpolation(startPose.getHeading(), PrepGather3.getHeading())
                .build();

        fGPath3 = follower.pathBuilder()
                .addPath(new BezierLine(PrepGather3, FinishGather3))
                .setLinearHeadingInterpolation(PrepGather3.getHeading(), FinishGather3.getHeading())
                .build();
        Shootpath3 = follower.pathBuilder()

                .addPath(new BezierLine(FinishGather3, startPose))
                .setLinearHeadingInterpolation(FinishGather3.getHeading(), startPose.getHeading())
                .build();
        lastOutPath = follower.pathBuilder()

                .addPath(new BezierLine(startPose, Park))
                .setLinearHeadingInterpolation(startPose.getHeading(), Park.getHeading())
                .build();
//
//        lastOutPath = follower.pathBuilder()
//                .addPath(new BezierLine(ShootPose1, endPose))
//                .setLinearHeadingInterpolation(ShootPose1.getHeading(), endPose.getHeading())
//                .build();

    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                firstshooting = false;
                setPathState(1);

                break;
            case 1:
                if(!follower.isBusy()) {
//                    if (!firstshooting) {
//
//                        timer.resetTimer();
//                        shooter.setFireButton(true);
//                        firstshooting = true;
//                    }
//                    else{
//
//                        if(timer.getElapsedTimeSeconds()>waittime){
//                            intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                            setPathState(2);
//                        }
//                        else if(timer.getElapsedTimeSeconds()>shoottime){
//                            shooter.setFireButton(false);
//                        }
//                        else{
//                            shooter.setFireButton(true);
//                        }
//
//                    }
//                    break;
//
                }
                break;
            //1st shooting________________________________________________
            case 2:
                if(!follower.isBusy()) {
                    firstshooting = false;
                    intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                    shooter.periodic();
                    follower.followPath(prepGatherPath1);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    follower.followPath(fGPath1);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()){
                    //shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                    follower.followPath(Shootpath1);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()) {
//                    if (!firstshooting) {
//
//                        timer.resetTimer();
//                        shooter.setFireButton(true);
//                        firstshooting = true;
//                    }
//                    else{
//
//                        if(timer.getElapsedTimeSeconds()>waittime){
//                            intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                           setPathState(12);
//                        }
//                        else if(timer.getElapsedTimeSeconds()>shoottime){
//                            shooter.setFireButton(false);
//                        }
//                        else{
//                            shooter.setFireButton(true);
//                        }
//
//                    }
//                    break;

                }
                break;

            case 12:
                if(!follower.isBusy()) {
                    follower.followPath(prepGatherPath2);
                    firstshooting = false;
                    setPathState(13);
                }
                break;

            case 13:
                if(!follower.isBusy()) {
                    follower.followPath(fGPath2);
                    firstshooting = false;
                    setPathState(14);
                }
                break;
            case 14:
                if(!follower.isBusy()){
                    firstshooting = false;
                    //shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                    follower.followPath(Shootpath2);
                    setPathState(15);
                }
                break;
            case 15:
                if(!follower.isBusy()) {
//                    if (!firstshooting) {
//
//                        timer.resetTimer();
//                        shooter.setFireButton(true);
//                        firstshooting = true;
//                    }
//                    else{
//
//                        if(timer.getElapsedTimeSeconds()>waittime){
//                            intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                            setPathState(16);
//                        }
//                        else if(timer.getElapsedTimeSeconds()>shoottime){
//                            shooter.setFireButton(false);
//                        }
//                        else{
//                            shooter.setFireButton(true);
//                        }
//
//                    }
//                    break;

                }
                break;
            //4th shooting________________________________________________
            case 16:
                if(!follower.isBusy()) {
                    firstshooting = false;
                    intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                    shooter.periodic();
                    follower.followPath(prepGatherPath3);
                    setPathState(17);
                }
                break;
            case 17:
                if(!follower.isBusy()) {
                    follower.followPath(fGPath3);
                    setPathState(18);
                }
                break;
            case 18:
                if(!follower.isBusy()){
                    follower.followPath(Shootpath3);
                    setPathState(19);
                }
                break;
            case 19:
                if(!follower.isBusy()) {
//                    if (!firstshooting) {
//
//                        timer.resetTimer();
//                        shooter.setFireButton(true);
//                        firstshooting = true;
//                    }
//                    else{
//
//                        if(timer.getElapsedTimeSeconds()>waittime){
//                            intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                            setPathState(20);
//                        }
//                        else if(timer.getElapsedTimeSeconds()>shoottime){
//                            shooter.setFireButton(false);
//                        }
//                        else{
//                            shooter.setFireButton(true);
//                        }
//
//                    }
//                    break;

                }
                break;
            //5th shooting________________________________________________
            case 20:
                if(!follower.isBusy()) {
                    follower.followPath(lastOutPath);
                    setPathState(27);
                }
                break;

            case 27:
                if(!follower.isBusy()){
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
        intake.periodic();
//        if(shooter.autoLonger){
//            turret.autopos = -195;
//        }
//        else{
//            turret.autopos = -138;
//        }

        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("timer", timer.getElapsedTimeSeconds());
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

        intake.setIntakeState(Intake.IntakeTransferState.Intake_Steady);


        buildPaths();
        //follower.setStartingPose(startPose);
        follower.setPose(startPose);


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

    }
}