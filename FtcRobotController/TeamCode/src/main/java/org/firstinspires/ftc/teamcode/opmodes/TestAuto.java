package org.firstinspires.ftc.teamcode.opmodes; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import static android.os.SystemClock.sleep;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MyLimelight;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Scheduler;
import org.firstinspires.ftc.teamcode.commands.LimelightLockInCommand;
@Autonomous(name = "TestAuto")
public class TestAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer, timer;
    //private final ElapsedTime timer  = new ElapsedTime();

    private int pathState =0;
    private final Pose startPose = new Pose(0, 0, 0); // Start Pose of our robot.
    private final Pose ShootPose1 = new Pose(9.4033, 1.4877, -0.436764);
    private final Pose PrepGather1 = new Pose(24.7910, -17.19286, -1.590508);

    private final Pose FinishGather1 = new Pose(24.7910, -35.2588, -1.590508);
    private boolean firstshooting = false;
    private PathChain Shootpath1, Shootpath2;
    private PathChain prepGatherPath1;

    public Intake intake;
    public Shooter shooter;
    public MyLimelight limelight;
    public Scheduler scheduler;

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

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                follower.followPath(Shootpath1);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    if (!firstshooting) {
                        shooter.updateFocused(true);
                        shooter.setShooterStatus(Shooter.ShooterStatus.Shooting);
                        timer.resetTimer();

                        firstshooting = true;
                    }
                    else{
                        if(timer.getElapsedTimeSeconds()> 4.2){
                            shooter.setShooterStatus(Shooter.ShooterStatus.Stop);
                            intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                            setPathState(2);
                        }
                        if(timer.getElapsedTimeSeconds()>2.3&&timer.getElapsedTimeSeconds()<2.8){
                            intake.autoforce = true;
                            intake.setIntakeState(Intake.IntakeTransferState.Intake_Steady);
                            intake.periodic();
                        }
                        if(timer.getElapsedTimeSeconds()>2.8&&timer.getElapsedTimeSeconds()<3.0){
                            intake.autoforce = true;
                            intake.setIntakeState(Intake.IntakeTransferState.Split_Out);
                            intake.periodic();
                        }
                        if(timer.getElapsedTimeSeconds()>3.0&&timer.getElapsedTimeSeconds()<3.4){
                            intake.autoforce = true;
                            intake.setIntakeState(Intake.IntakeTransferState.Send_It_Up);
                            intake.periodic();
                        }
                        if(timer.getElapsedTimeSeconds()>3.4&&timer.getElapsedTimeSeconds()<3.5){
                            intake.autoforce = false;
                            intake.periodic();
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
                        shooter.setShooterStatus(Shooter.ShooterStatus.Shooting);
                        timer.resetTimer();

                        firstshooting = true;
                    }
                    else{
                        if(timer.getElapsedTimeSeconds()> 4.2){
                            shooter.setShooterStatus(Shooter.ShooterStatus.Stop);
                            intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                            setPathState(2);
                        }
                        if(timer.getElapsedTimeSeconds()>2.3&&timer.getElapsedTimeSeconds()<2.8){
                            intake.autoforce = true;
                            intake.setIntakeState(Intake.IntakeTransferState.Intake_Steady);
                            intake.periodic();
                        }
                        if(timer.getElapsedTimeSeconds()>2.8&&timer.getElapsedTimeSeconds()<3.0){
                            intake.autoforce = true;
                            intake.setIntakeState(Intake.IntakeTransferState.Split_Out);
                            intake.periodic();
                        }
                        if(timer.getElapsedTimeSeconds()>3.0&&timer.getElapsedTimeSeconds()<3.4){
                            intake.autoforce = true;
                            intake.setIntakeState(Intake.IntakeTransferState.Send_It_Up);
                            intake.periodic();
                        }
                        if(timer.getElapsedTimeSeconds()>3.4&&timer.getElapsedTimeSeconds()<3.5){
                            intake.autoforce = false;
                            intake.periodic();
                        }

                    }
                    break;
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