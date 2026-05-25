package org.firstinspires.ftc.teamcode.opmodes; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MyLimelight;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Scheduler;

@Autonomous(name = "Red_Far_12ball")
public class RED_Far_12ball extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer, timer;
    //private final ElapsedTime timer  = new ElapsedTime();

    private int pathState =0;
    private final Pose startPose = new Pose(0, 0, 0); // Start Pose of our robot.
    private final Pose ShootPose1 = new Pose(7.187, 0.9717, -0.354764);

    private final Pose ShootPose2 = ShootPose1;
    private final Pose PrepGather1 = new Pose(7.187, -5.904, -1.590508);

    private final Pose FinishGather1 = new Pose(2.98, -43.7588, -1.6110508);

    private final Pose PrepGather2 = new Pose(28.631, 1.02, -1.5541);

    private final Pose FinishGather2 = new Pose(28.631, -42.2588, -1.580508);

    private final Pose PrepGather3 = PrepGather1;

    private final Pose FinishGather3 = FinishGather1;
    private boolean cycleflag = false;
    private final Pose endPose = new Pose(4.64556,-44.66559,-1.5623);

    private boolean firstshooting = false;
    private PathChain Shootpath1, Shootpath2, Shootpath3,Shootpath4, lastOutPath;
    private PathChain prepGatherPath1, prepGatherPath2, prepGatherPath3;

    public Intake intake;
    public Shooter shooter;
    public MyLimelight limelight;
    public Scheduler scheduler;
    public double waittime = 1;
    public double shoottime = 2;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        Shootpath1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, ShootPose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), ShootPose1.getHeading())
                .build();

        prepGatherPath1 = follower.pathBuilder()
                .addPath(new BezierLine(ShootPose1, PrepGather1))
                .setLinearHeadingInterpolation(ShootPose1.getHeading(), PrepGather1.getHeading())
                .addPath(new BezierLine(PrepGather1, FinishGather1))
                .setLinearHeadingInterpolation(PrepGather1.getHeading(), FinishGather1.getHeading())
                .build();



        Shootpath2 = follower.pathBuilder()
                .addPath(new BezierLine(FinishGather1, ShootPose1))
                .setLinearHeadingInterpolation(FinishGather1.getHeading(), ShootPose1.getHeading())
                .build();

        prepGatherPath2 = follower.pathBuilder()
                .addPath(new BezierLine(ShootPose1, PrepGather2))
                .setLinearHeadingInterpolation(ShootPose1.getHeading(), PrepGather2.getHeading())
                .addPath(new BezierLine(PrepGather2, FinishGather2))
                .setLinearHeadingInterpolation(PrepGather2.getHeading(), FinishGather2.getHeading())
                .build();

        Shootpath3 = follower.pathBuilder()
                .addPath(new BezierLine(FinishGather2, PrepGather2))
                .setLinearHeadingInterpolation(FinishGather2.getHeading(), PrepGather2.getHeading())
                .addPath(new BezierLine(PrepGather2, ShootPose2))
                .setLinearHeadingInterpolation(PrepGather2.getHeading(), ShootPose2.getHeading())
                .build();

        prepGatherPath3 = follower.pathBuilder()
                .addPath(new BezierLine(ShootPose2, PrepGather3))
                .setLinearHeadingInterpolation(ShootPose2.getHeading(), PrepGather3.getHeading())
                .addPath(new BezierLine(PrepGather3, FinishGather3))
                .setLinearHeadingInterpolation(PrepGather3.getHeading(), FinishGather3.getHeading())
                .build();

        Shootpath4 = follower.pathBuilder()
                .addPath(new BezierLine(FinishGather3, PrepGather3))
                .setLinearHeadingInterpolation(FinishGather3.getHeading(), PrepGather3.getHeading())
                .addPath(new BezierLine(PrepGather3, ShootPose2))
                .setLinearHeadingInterpolation(PrepGather3.getHeading(), ShootPose2.getHeading())
                .build();

        lastOutPath = follower.pathBuilder()
                .addPath(new BezierLine(ShootPose2, endPose))
                .setLinearHeadingInterpolation(ShootPose2.getHeading(), endPose.getHeading())
                .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                shooter.autoLonger = true;
                shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                follower.followPath(Shootpath1);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    if (!firstshooting) {
                        shooter.updateFocused(true);
                        //shooter.setShooterStatus(Shooter.ShooterStatus.Shooting);
                        timer.resetTimer();

                        firstshooting = true;
                    }
                    else{
                        if(timer.getElapsedTimeSeconds()>waittime){
                            shooter.setShooterStatus(Shooter.ShooterStatus.Shooting);
                        }
                        if(timer.getElapsedTimeSeconds()> shoottime){
                            shooter.setShooterStatus(Shooter.ShooterStatus.Stop);
                            intake.setSwingBarPos(0.4);
                            intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                            setPathState(2);
                        }


                    }
                    break;

                }
            case 2:
                if(!follower.isBusy()) {

                    shooter.setShooterStatus(Shooter.ShooterStatus.Stop);
                    intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                    shooter.periodic();
                    follower.followPath(prepGatherPath1);
                    setPathState(3);
                }
                break;
            case 3:

                if(!follower.isBusy()) {
                    setPathState(4);
                    shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                    intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                    shooter.periodic();
                }
                break;
            case 4:

                follower.followPath(Shootpath2);
                firstshooting = false;
                setPathState(5);

                break;
            case 5:
                if(!follower.isBusy()) {
                    if (!firstshooting) {
                        shooter.updateFocused(true);
                        //shooter.setShooterStatus(Shooter.ShooterStatus.Shooting);
                        timer.resetTimer();

                        firstshooting = true;
                    }
                    else{
                        if(timer.getElapsedTimeSeconds()>waittime){
                            shooter.setShooterStatus(Shooter.ShooterStatus.Shooting);
                        }
                        if(timer.getElapsedTimeSeconds()> shoottime){
                            shooter.setShooterStatus(Shooter.ShooterStatus.Stop);
                            intake.setSwingBarPos(0.4);
                            intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                            setPathState(6);
                        }


                    }
                    break;
                }
            case 6:
                if(!follower.isBusy()) {

                    shooter.setShooterStatus(Shooter.ShooterStatus.Stop);
                    intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                    shooter.periodic();
                    follower.followPath(prepGatherPath2);
                    setPathState(7);
                }
                break;
            case 7:

                if(!follower.isBusy()) {
                    setPathState(8);
                    shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                    intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                    shooter.periodic();
                }
                break;
            case 8:
                shooter.autoLonger = true;
                follower.followPath(Shootpath3);
                firstshooting = false;
                setPathState(9);

                break;
            case 9:
                if(!follower.isBusy()) {
                    if (!firstshooting) {
                        shooter.updateFocused(true);
                        //shooter.setShooterStatus(Shooter.ShooterStatus.Shooting);
                        timer.resetTimer();

                        firstshooting = true;
                    }
                    else{
                        if(timer.getElapsedTimeSeconds()>waittime){
                            shooter.setShooterStatus(Shooter.ShooterStatus.Shooting);
                        }
                        if(timer.getElapsedTimeSeconds()> shoottime){
                            shooter.setShooterStatus(Shooter.ShooterStatus.Stop);
                            intake.setSwingBarPos(0.4);
                            intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                            setPathState(10);
                        }


                    }
                    break;
                }
            case 10:
                if(!follower.isBusy()) {

                    shooter.setShooterStatus(Shooter.ShooterStatus.Stop);
                    intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                    shooter.periodic();
                    follower.followPath(prepGatherPath3);
                    setPathState(11);
                }
                break;
            case 11:
                if(follower.getPose().getY()<-34){
                    intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                    intake.periodic();
                }

                if(!follower.isBusy()) {
                    setPathState(12);
                    shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                    intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                    shooter.periodic();
                }
                break;
            case 12:

                follower.followPath(Shootpath4);
                firstshooting = false;
                setPathState(13);

                break;
            case 13:
                if(!follower.isBusy()) {
                    if (!firstshooting) {
                        shooter.updateFocused(true);
                        //shooter.setShooterStatus(Shooter.ShooterStatus.Shooting);
                        timer.resetTimer();

                        firstshooting = true;
                    }
                    else{
                        if(timer.getElapsedTimeSeconds()>waittime){
                            shooter.setShooterStatus(Shooter.ShooterStatus.Shooting);
                        }
                        if(timer.getElapsedTimeSeconds()> shoottime){
                            shooter.setShooterStatus(Shooter.ShooterStatus.Stop);
                            intake.setSwingBarPos(0.4);
                            intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                            if(!cycleflag){
                                cycleflag = true;
                                setPathState(10);
                            }else {
                                setPathState(14);
                            }
                        }


                    }
                    break;
                }
            case 14:
                if(!follower.isBusy()) {
                    follower.followPath(lastOutPath);
                    setPathState(15);
                }
                break;
            case 15:
                if(!follower.isBusy()) {
                    setPathState(16);
                }

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
        intake.periodic();
        shooter.periodic();
        if(shooter.shooterStatus == Shooter.ShooterStatus.Shooting){
            intake.updateAutoshoot(true);
            intake.updateautotranse(shooter.isAtTargetRPM());
            shooter.updateDis(limelight.getDis());
            shooter.updateFocused(true);
        }
        else{
            intake.updateAutoshoot(false);
            shooter.updateFocused(false);
        }
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
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
        limelight.initBluePipeline();
        limelight.startDetect();
        intake.setIntakeState(Intake.IntakeTransferState.Intake_Steady);
        shooter.setShooterStatus(Shooter.ShooterStatus.Stop);
        intake.setSwingBarPos(0.4);
        buildPaths();
        follower.setStartingPose(startPose);

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
}