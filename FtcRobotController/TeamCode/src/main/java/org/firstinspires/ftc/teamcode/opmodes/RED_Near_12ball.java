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

@Autonomous(name = "RED_Far_12ball")
public class RED_Near_12ball extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer, timer;
    //private final ElapsedTime timer  = new ElapsedTime();

    private int pathState =0;
    private final Pose startPose = new Pose(0, 0, 0); // Start Pose of our robot.
    private final Pose ShootPose1 = new Pose(-52.702944, -5.8971, -0.71322);
    private final Pose GatePose = new Pose(-61.10353, -44.8150, 0.02);
    private final Pose PrepGather1 = new Pose(-50.4755, -21.3882, -1.583372);

    private final Pose FinishGather1 = new Pose(-50.4755, -37.8899, -1.583372);

    private final Pose PrepGather2 = new Pose(-74.5229, -21.3882, -1.583372);

    private final Pose FinishGather2 = new Pose(-74.5229, -37.8899, -1.583372);

    private final Pose PrepGather3 = new Pose(-98.51537, -21.3882, -1.583372);

    private final Pose FinishGather3 = new Pose(-98.51537, -37.8899, -1.583372);


    private boolean firstshooting = false;
    private PathChain Shootpath1, Shootpath2, Shootpath3,Shootpath4, lastOutPath;
    private PathChain prepGatherPath1, prepGatherPath2, prepGatherPath3;

    public Intake intake;
    public Shooter shooter;
    public MyLimelight limelight;
    public Scheduler scheduler;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                shooter.autoLonger = true;
                shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                follower.followPath(Shootpath1);
                setPathState(1);
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
        limelight.initRedPipeline();
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
