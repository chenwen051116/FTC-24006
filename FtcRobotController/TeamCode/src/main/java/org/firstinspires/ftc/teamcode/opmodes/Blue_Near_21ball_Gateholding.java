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
import com.qualcomm.hardware.lynx.LynxModule;
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

import java.util.List;

@Config
@Autonomous(name = "Blue_Near_21ball_Gateholding")

public class Blue_Near_21ball_Gateholding extends OpMode {

    //private drive.follower drive.follower;
    private Drivetrain drive;
    private Timer pathTimer, actionTimer, opmodeTimer, timer,gatetimer;
    //private final ElapsedTime timer  = new ElapsedTime();

    private int pathState = 0;
    public List<LynxModule> allHubs;

    private Timer looptimer;


    private final Pose GatePrep2 = new Pose(120.2648,-62.6550,1.57);
    private final Pose GatePush2 = new Pose(120.2648,-62.6550,1.57);
    private final Pose startPose = new Pose(118.85432686392717, -105.98108006274607, 0); //
    private final Pose PrepGather1 = new Pose(91.9908, -28.6053, 0);
    private final Pose FinishGather1 = new Pose(114.9794, -28.6053, 0);

    private final Pose PrepGather2 = new Pose(112.19, -65.32, 1.57);

    private final Pose FinishGather2 = new Pose(112.19, -48.62, 1.57);
    private final Pose GatePassby = new Pose(118.1884092796506, -51.44941675381398, -0.7060);//;//real pass by
    private final Pose GatePassby2 = new Pose(118.6561, -55.5252, -0.30050);//hit gate
    private final Pose GatePose = new Pose(124.5884092796506, -50.84941675381398, -0.6560);//
    //private final Pose GatePose = new Pose(120.6561, -55.0, -0.30050);//pickup
    private final Pose ShootPose = new Pose(79.1620, -70.80 ,0);
    private final Pose ShootPose_Near = new Pose(96.2372, -97.6927 ,1.57);

    private final Pose PrepGather3 = new Pose(109.61, -90.64, 1.57);//accounted for overshoot

    private final Pose FinishGather3 = new Pose(109.61, -74.64, 1.57);

    private final Pose Park = new Pose(76.3860, -93.5130, 0.7830);;

    private boolean firstshooting = false;

    public double turretoff = 0;
    private double gatePathPower = 1;
    private PathChain GateShoot,GatePath1,GatePath2, Shootpath1,Shootpath2, Shootpath3,Shootpath4,Shootpath5, lastOutPath;
    private PathChain GatePushPath,prepGatherPath6,prepGatherPath1,finishGatherPath6,Shootpath6, prepGatherPath2, prepGatherPath3, prepGatherPath4;

    private PathChain finishGatherPath1,finishGatherPath2,finishGatherPath3,finishGatherPath4;
    public Intake intake;
    public Shooter shooter;
    public MyLimelight limelight;
    public Scheduler scheduler;

    public Turret turret;

    public static double stoptime = 1;
    public static double shoottime = 0.8;

    public static double waittime = 0;
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

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        Shootpath1 = drive.follower.pathBuilder()
                .addPath(new BezierCurve(startPose, ShootPose_Near))
                .setConstantHeadingInterpolation(PrepGather2.getHeading())
                .build();
//
//                .addPath(new BezierLine(PrepGather4, FinishGather4))
//                .setLinearHeadingInterpolation(PrepGather4.getHeading(), FinishGather4.getHeading())
//                .setTValueConstraint(0.9)
//                .addPath(new BezierLine(FinishGather4, PrepGather4))
//                .setLinearHeadingInterpolation(FinishGather4.getHeading(), PrepGather4.getHeading())
//                .addPath(new BezierLine(PrepGather4, FinishGather4))
//                .setLinearHeadingInterpolation(PrepGather4.getHeading(), FinishGather4.getHeading())
//
//                .build();
        prepGatherPath1 = drive.follower.pathBuilder()

                .addPath(new BezierCurve(ShootPose_Near, PrepGather2))
                .setLinearHeadingInterpolation(ShootPose_Near.getHeading(), PrepGather2.getHeading())
                .addPath(new BezierCurve(PrepGather2, FinishGather2))
                .setLinearHeadingInterpolation(PrepGather2.getHeading(), FinishGather2.getHeading())

                .build();
        GatePushPath = drive.follower.pathBuilder()
                .addPath(new BezierCurve(FinishGather2, GatePrep2))
                .setLinearHeadingInterpolation(FinishGather2.getHeading(), GatePrep2.getHeading())
//                .addPath(new BezierCurve(GatePrep2, GatePush2))
//                .setLinearHeadingInterpolation(GatePrep2.getHeading(), GatePush2.getHeading())
                //  .addPath(new BezierCurve(GatePush2, FinishGather2))
                //  .setLinearHeadingInterpolation(GatePush2.getHeading(), FinishGather2.getHeading())
                .build();
        Shootpath2 = simplePath(GatePrep2,ShootPose);
//
//        prepGatherPath2 = simplePath(ShootPose1,PrepGather2);
//
//        finishGatherPath2 = simplePath(PrepGather2, FinishGather2);
        prepGatherPath2 = drive.follower.pathBuilder()

                .addPath(new BezierCurve(ShootPose_Near, PrepGather3))
                .setLinearHeadingInterpolation(ShootPose_Near.getHeading(), PrepGather3.getHeading())
                .addPath(new BezierCurve(PrepGather3, FinishGather3))
                .setLinearHeadingInterpolation(PrepGather3.getHeading(), FinishGather3.getHeading())


                .build();

//        Shootpath3 = simplePath(FinishGather3,ShootPose);

//        GatePath1 = drive.follower.pathBuilder()
////                .setTValueConstraint(0.95)
//                .addPath(new BezierLine(ShootPose, GatePassby))
//                .setLinearHeadingInterpolation(ShootPose.getHeading(), GatePassby.getHeading())
//                .addPath(new BezierLine(GatePassby, GatePassby2))
//                .setLinearHeadingInterpolation(GatePassby.getHeading(), GatePassby2.getHeading())

        GatePath1 = drive.follower.pathBuilder()
//                .setTValueConstraint(0.95)
                .addPath(new BezierLine(ShootPose, GatePassby))
                .setLinearHeadingInterpolation(ShootPose.getHeading(), GatePassby.getHeading())

                .build();
        GatePath2 = drive.follower.pathBuilder()
//                .setTValueConstraint(0.997)
                .addPath(new BezierLine(GatePassby, GatePose))
                .setLinearHeadingInterpolation(GatePassby.getHeading(), GatePose.getHeading())
                .setTValueConstraint(0.8)
                .build();

//        GatePath2 = drive.follower.pathBuilder()
////                .setTValueConstraint(0.997)
//                .addPath(new BezierLine(GatePassby2, GatePose))
//                .setLinearHeadingInterpolation(GatePassby2.getHeading(), GatePose.getHeading())
//                .build();

        GateShoot =
                drive.follower.pathBuilder()
                        .addPath(new BezierLine(GatePose, ShootPose))
                        .setLinearHeadingInterpolation(GatePose.getHeading(), ShootPose.getHeading())
                        .setTValueConstraint(0.997)
                        .build();
//                       .addPath(new BezierLine(GatePose, GatePassby))
//                       // .setTValueConstraint(0.90)
//                       .setLinearHeadingInterpolation(GatePose.getHeading(), GatePassby.getHeading())
//                       .addPath(new BezierLine(GatePassby, ShootPose))
//                       .setLinearHeadingInterpolation(GatePassby.getHeading(), ShootPose.getHeading())
//                       .build();
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

                .addPath(new BezierCurve(ShootPose, PrepGather1))
                .setLinearHeadingInterpolation(ShootPose.getHeading(), PrepGather1.getHeading())
                .addPath(new BezierLine(PrepGather1, FinishGather1))
                .setLinearHeadingInterpolation(PrepGather1.getHeading(), FinishGather1.getHeading())
                .setTValueConstraint(0.85)
                .setBrakingStrength(0.8)
                .build();

        Shootpath3 = drive.follower.pathBuilder()
                .addPath(new BezierLine(FinishGather3,ShootPose_Near))
//                .setLinearHeadingInterpolation(FinishGather1.getHeading(),Park.getHeading())
                .setConstantHeadingInterpolation(ShootPose_Near.getHeading())
                .setBrakingStrength(0.8)
                .build();

        Shootpath4 = drive.follower.pathBuilder()
                .addPath(new BezierLine(GatePose,Park))
//                .setLinearHeadingInterpolation(FinishGather1.getHeading(),Park.getHeading())
                .setConstantHeadingInterpolation(Park.getHeading())
                .setBrakingStrength(0.8)
                .build();

//        prepGatherPath4 = simplePath(ShootPose2,PrepGather4);
//
//        finishGatherPath4 = simplePath(PrepGather4,FinishGather4);

        lastOutPath = simplePath(ShootPose,Park);
//
//        lastOutPath = drive.follower.pathBuilder()
//                .addPath(new BezierLine(ShootPose1, endPose))
//                .setLinearHeadingInterpolation(ShootPose1.getHeading(), endPose.getHeading())
//                .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                shooter.offset = 50;
                shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                drive.follower.followPath(Shootpath1,1,true);
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
                        if(timer.getElapsedTimeSeconds()<0){
                            shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                        }
                        else if(timer.getElapsedTimeSeconds()<(shoottime+5.5)){
                            shooter.setShooterStatus(Shooter.ShooterStatus.Shooting);
                        }
                        if((!intake.hasballCheck(3))&&(!intake.hasballCheck(2))&&(!intake.hasballCheck(1))){
                            checkcounter -=1;
                        }
                        else{
                            checkcounter = checkcount;
                        }
                        if(checkcounter<0||timer.getElapsedTimeSeconds()> shoottime+1.5){
                            shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                            intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                            // turret.autopos = 0;

                            setPathState(12);
                        }

                    }
                    break;
                }
                //1st shooting________________________________________________
            case 2:
                if(!drive.follower.isBusy()) {
                    //  turret.autopos = 0;
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
                    shooter.offset = 0;
                    // turret.isManeulCentering = true;
                    // turret.centeringDir = false;
                    //intake.setIntakeState(Intake.IntakeTransferState.Intake_Steady);
                    drive.follower.followPath(GatePushPath,1,false);

                    setPathState(4);
                }
                break;
            case 4:
                if(!drive.follower.isBusy()){
                    //  turret.isManeulCentering = false;
                    // turret.centeringDir = false;
                    shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                    intake.setIntakeState(Intake.IntakeTransferState.Intake_Steady);
                    drive.follower.followPath(Shootpath2,1,true);
                    firstshooting = false;
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
                        if((!intake.hasballCheck(3))&&(!intake.hasballCheck(2))&&(!intake.hasballCheck(1))){
                            checkcounter -=1;
                        }
                        else{
                            checkcounter = checkcount;
                        }
                        if(checkcounter<0||timer.getElapsedTimeSeconds()> shoottime){
                            shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                            intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                            firstshooting = false;
                            setPathState(6);
                        }

                    }
                    break;

                }
                break;
            //2nd shooting________________________________________________
            case 6:

                if(!drive.follower.isBusy()) {
//                    shooter.offset = 0;
                    intake.autoIntakeUp = true;
                    shooter.setShooterStatus(Shooter.ShooterStatus.Stop);
                    intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                    shooter.periodic();
                    drive.follower.followPath(GatePath1,1,true);
                    setPathState(7);
                }
                break;
            case 7:
                if(!drive.follower.isBusy()) {
                    shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                    //intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                    shooter.periodic();
                    drive.follower.followPath(GatePath2,gatePathPower,true);
                    firstshooting = false;
                    gatetimer.resetTimer();
                    setPathState(9);
                }
                break;
            case 9:
                if(gatetimer.getElapsedTimeSeconds()> 0.25&&gatetimer.getElapsedTimeSeconds()< stoptime){
                    intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                }
                if(!drive.follower.isBusy()){
                    if (!firstshooting) {
                        timer.resetTimer();
                        firstshooting = true;
                        break;
                    }
                    else{

                        if((timer.getElapsedTimeSeconds()> stoptime || (intake.hasballCheck(1)&&intake.hasballCheck(2)&&intake.hasballCheck(3)))&&(timer.getElapsedTimeSeconds()>0.8)){

                            shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                            intake.setIntakeState(Intake.IntakeTransferState.Suck_In_slow);
                            drive.follower.followPath(GateShoot);
                            firstshooting = false;
                            setPathState(10);
                            //setPathState(23);

                            break;
                        }

                    }
                }
                break;
            case 10:
                if(!drive.follower.isBusy()){
                    firstshooting = false;
                    setPathState(11);
                }
                break;



            case 11:
                if(!drive.follower.isBusy()) {
                    intake.autoIntakeUp = false;
                    if (!firstshooting) {
                        shooter.updateFocused(true);

                        timer.resetTimer();
                        firstshooting = true;
                    }
                    else{
                        if(timer.getElapsedTimeSeconds()>(waittime)&&timer.getElapsedTimeSeconds()<(shoottime+0.5)){
                            shooter.setShooterStatus(Shooter.ShooterStatus.Shooting);
                        }
                        if((!intake.hasballCheck(3))&&(!intake.hasballCheck(2))&&(!intake.hasballCheck(1))){
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
            // 3rd shooting________________________________________________
            case 12:
                if(!drive.follower.isBusy()) {
                    firstshooting = false;
                    shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                    intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                    shooter.periodic();
                    drive.follower.followPath(prepGatherPath2,1,true);
                    setPathState(13);
                }
                break;
            case 13:
                if(!drive.follower.isBusy()) {
                    //drive.follower.followPath(finishGatherPath3);
                    setPathState(14);
                }
                break;
            case 14:
                if(!drive.follower.isBusy()){
                    intake.setIntakeState(Intake.IntakeTransferState.Intake_Steady);
                    shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                    drive.follower.followPath(Shootpath3);
                    setPathState(15);
                }
                break;
            case 15:
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
                        if((!intake.hasballCheck(3))&&(!intake.hasballCheck(2))&&(!intake.hasballCheck(1))){
                            checkcounter -=1;
                        }
                        else{
                            checkcounter = checkcount;
                        }
                        if(checkcounter<0||timer.getElapsedTimeSeconds()> shoottime){
                            shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                            intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                            setPathState(2);
                        }

                    }
                    break;

                }
                break;
            //4th shooting________________________________________________
            case 16:
                if(!drive.follower.isBusy()) {
                    shooter.setShooterStatus(Shooter.ShooterStatus.Stop);
                    intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                    shooter.periodic();
                    drive.follower.followPath(GatePath1,1,true);
                    setPathState(47);
                }
                break;
            case 47:
                if(!drive.follower.isBusy()) {
                    shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                    //intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                    shooter.periodic();
                    drive.follower.followPath(GatePath2,gatePathPower,true);
                    gatetimer.resetTimer();
                    firstshooting = false;
                    setPathState(17);
                }
                break;
            case 17:
                if(gatetimer.getElapsedTimeSeconds()> 0.25&&gatetimer.getElapsedTimeSeconds()< stoptime){
                    intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                }
                if(!drive.follower.isBusy()){
                    if (!firstshooting) {
                        timer.resetTimer();
                        firstshooting = true;
                        break;
                    }
                    else{

                        if((timer.getElapsedTimeSeconds()> stoptime || (intake.hasballCheck(1)&&intake.hasballCheck(2)&&intake.hasballCheck(3)))&&(timer.getElapsedTimeSeconds()>0.8)){

                            shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                            intake.setIntakeState(Intake.IntakeTransferState.Suck_In_slow);
                            drive.follower.followPath(GateShoot);
                            firstshooting = false;
                            setPathState(18);
                            //setPathState(23);

                            break;
                        }

                    }
                }
                break;
            case 18:
                if(!drive.follower.isBusy()){
                    firstshooting = false;
                    setPathState(19);
                }
                break;



            case 19:
                if(!drive.follower.isBusy()) {
                    if (!firstshooting) {
                        //shooter.updateFocused(true);

                        timer.resetTimer();
                        firstshooting = true;
                    }
                    else{
                        if(timer.getElapsedTimeSeconds()>(waittime)&&timer.getElapsedTimeSeconds()<(shoottime+0.5)){
                            shooter.setShooterStatus(Shooter.ShooterStatus.Shooting);
                        }
                        if((!intake.hasballCheck(3))&&(!intake.hasballCheck(2))&&(!intake.hasballCheck(1))){
                            checkcounter -=1;
                        }
                        else{
                            checkcounter = checkcount;
                        }
                        if(checkcounter<0||timer.getElapsedTimeSeconds()> shoottime){
                            shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                            intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                            setPathState(24);
                        }

                    }
                    break;

                }
                break;

            //2th Gate
            case 20:
                if(!drive.follower.isBusy()) {
                    shooter.setShooterStatus(Shooter.ShooterStatus.Stop);
                    intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                    shooter.periodic();
                    drive.follower.followPath(GatePath1,1,true);
                    setPathState(40);
                }
                break;
            case 40:
                if(!drive.follower.isBusy()) {
                    shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                    // intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                    shooter.periodic();
                    drive.follower.followPath(GatePath2,gatePathPower,true);
                    firstshooting = false;
                    gatetimer.resetTimer();
                    setPathState(21);
                }
                break;
            case 21:
                if(gatetimer.getElapsedTimeSeconds()> 0.25&&gatetimer.getElapsedTimeSeconds()< stoptime){
                    intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                }
                if(!drive.follower.isBusy()){
                    if (!firstshooting) {
                        timer.resetTimer();
                        firstshooting = true;
                        break;
                    }
                    else{

                        if((timer.getElapsedTimeSeconds()> stoptime || (intake.hasballCheck(1)&&intake.hasballCheck(2)&&intake.hasballCheck(3)))&&(timer.getElapsedTimeSeconds()>0.8)){

                            shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                            intake.setIntakeState(Intake.IntakeTransferState.Suck_In_slow);
                            drive.follower.followPath(GateShoot);
                            firstshooting = false;
                            setPathState(22);
                            //setPathState(23);

                            break;
                        }

                    }
                }
                break;
            case 22:
                if(!drive.follower.isBusy()){
                    firstshooting = false;
                    setPathState(23);
                }
                break;



            case 23:
                if(!drive.follower.isBusy()) {
                    if (!firstshooting) {
//                        shooter.updateFocused(true);

                        timer.resetTimer();
                        firstshooting = true;
                    }
                    else{
                        if(timer.getElapsedTimeSeconds()>(waittime)&&timer.getElapsedTimeSeconds()<(shoottime+0.5)){
                            shooter.setShooterStatus(Shooter.ShooterStatus.Shooting);
                        }
                        if((!intake.hasballCheck(3))&&(!intake.hasballCheck(2))&&(!intake.hasballCheck(1))){
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




            //2th Gate
            case 24:
                if(!drive.follower.isBusy()) {
                    shooter.setShooterStatus(Shooter.ShooterStatus.Stop);
                    intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                    shooter.periodic();
                    drive.follower.followPath(GatePath1,1,true);
                    setPathState(25);
                }
                break;
            case 25:
                if(!drive.follower.isBusy()) {
                    shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                    // intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                    shooter.periodic();
                    drive.follower.followPath(GatePath2,gatePathPower,true);
                    firstshooting = false;
                    gatetimer.resetTimer();
                    setPathState(26);
                }
                break;
            case 26:
                if(gatetimer.getElapsedTimeSeconds()> 0.25&&gatetimer.getElapsedTimeSeconds()< stoptime){
                    intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                }
                if(!drive.follower.isBusy()){
                    if (!firstshooting) {
                        timer.resetTimer();
                        firstshooting = true;
                        break;
                    }
                    else{

                        if((timer.getElapsedTimeSeconds()> stoptime || (intake.hasballCheck(1)&&intake.hasballCheck(2)&&intake.hasballCheck(3)))&&(timer.getElapsedTimeSeconds()>0.8)){

                            shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                            intake.setIntakeState(Intake.IntakeTransferState.Suck_In_slow);
                            drive.follower.followPath(Shootpath4);
                            firstshooting = false;
                            setPathState(27);
                            //setPathState(23);

                            break;
                        }

                    }
                }
                break;
            case 27:
                if(!drive.follower.isBusy()){
                    firstshooting = false;
                    setPathState(28);
                }
                break;



            case 28:
                if(!drive.follower.isBusy()) {
                    if (!firstshooting) {
//                        shooter.updateFocused(true);

                        timer.resetTimer();
                        firstshooting = true;
                    }
                    else{
                        if(timer.getElapsedTimeSeconds()>(waittime)&&timer.getElapsedTimeSeconds()<(shoottime+0.5)){
                            shooter.setShooterStatus(Shooter.ShooterStatus.Shooting);
                        }
                        if((!intake.hasballCheck(3))&&(!intake.hasballCheck(2))&&(!intake.hasballCheck(1))){
                            checkcounter -=1;
                        }
                        else{
                            checkcounter = checkcount;
                        }
                        if(checkcounter<0||timer.getElapsedTimeSeconds()> shoottime){
                            shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                            intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                            setPathState(29);
                        }

                    }
                    break;

                }
                break;






//
//            case 28:
//                if(!drive.follower.isBusy()) {
//                    shooter.setShooterStatus(Shooter.ShooterStatus.Stop);
//                    intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
//                    shooter.periodic();
//                    drive.follower.followPath(lastOutPath);
//                    resetSubsystemsForTeleop();
//                    setPathState(29);
//                }
//                break;
            case 29:
                intake.autoIntakeUp = false;
                Drivetrain.lastPose = drive.follower.getPose();
                Drivetrain.TredFblue = false;
                if(!drive.follower.isBusy()){
                    shooter.offset = 0;
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
//            shooter.updateDis(limelight.getDis());
//            shooter.updateFocused(limelight.isFocused());
            //shooter.updateFocused(true);
        }
        else{
            intake.updateAutoshoot(false);

        }
        if(shooter.shooterStatus != Shooter.ShooterStatus.Stop){

            shooter.ododis = drive.getdis_TWO();
            turret.aimangle = drive.getturretangle_TWO()+toRadians(turretoff);

            turret.updateAutoShoot(true);
            //turret.tx = limelight.getTx();

        }
        else{
            turret.updateAutoShoot(false);
        }
        autonomousPathUpdate();
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
//        // Feedback to Driver Hub for debugging

        telemetry.addData("looptime", looptimer.getElapsedTime());
        looptimer.resetTimer();
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
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        looptimer = new Timer();
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