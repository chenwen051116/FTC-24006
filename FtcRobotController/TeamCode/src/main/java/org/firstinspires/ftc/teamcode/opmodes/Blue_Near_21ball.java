package org.firstinspires.ftc.teamcode.opmodes; // make sure this aligns with class location

import static java.lang.Math.abs;
import static java.lang.Math.sqrt;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
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
@Autonomous(name = "Blue_Near_21ball_gate")

public class Blue_Near_21ball extends OpMode {

    //private drive.follower drive.follower;
    private Drivetrain drive;
    private Timer pathTimer, actionTimer, opmodeTimer, timer,gatetimer;
    //private final ElapsedTime timer  = new ElapsedTime();

    private int pathState = 0;
    private final Pose startPose = new Pose(116.6447, -109.9232, 0); // Start Pose of our robot.
    private final Pose PrepGather1 = new Pose(91.9908, -28.6053-5, 0);
    private final Pose FinishGather1 = new Pose(114.9794, -28.6053, 0);

    private final Pose PrepGather2 = new Pose(91.9908, -52.0297-5, 0);

    private final Pose FinishGather2 = new Pose(114.9794, -52.0297, 0);
    private final Pose GatePassby = new Pose(104.9794, -58.7386, 0);//real pass by
    private final Pose GatePassby2 = new Pose(118.7843, -60.7386, 0);//hit gate
    private final Pose GatePose = new Pose(121.9260, -49.1962, -0.7081);//pickup
    private final Pose ShootPose = new Pose(79.1620, -70.80 ,0);

    private final Pose PrepGather3 = new Pose(91.9908, -75.8070, 0);//accounted for overshoot

    private final Pose FinishGather3 = new Pose(114.9794, -75.8070, 0);

    private final Pose Park = new Pose(72.3860, -88.5130, 0.7830);;

    private boolean firstshooting = false;
    private double gatePathPower = 0.6;
    private PathChain GateShoot,GatePath1,GatePath2, Shootpath1,Shootpath2, Shootpath3,Shootpath4,Shootpath5, lastOutPath;
    private PathChain prepGatherPath6,prepGatherPath1,finishGatherPath6,Shootpath6, prepGatherPath2, prepGatherPath3, prepGatherPath4;

    private PathChain finishGatherPath1,finishGatherPath2,finishGatherPath3,finishGatherPath4;
    public Intake intake;
    public Shooter shooter;
    public MyLimelight limelight;
    public Scheduler scheduler;

    public Turret turret;

    public static double stoptime = 1;
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

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        Shootpath1 = simplePath(startPose,ShootPose);

        //prepGatherPath1 = simplePath(ShootPose1,PrepGather4);
//        prepGatherPath1 = simplePath(startPose,PrepGather2);
////        finishGatherPath1 = simplePath(PrepGather4,FinishGather4);
//        finishGatherPath1 = drive.follower.pathBuilder()
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

                .addPath(new BezierLine(ShootPose, PrepGather2))
                .setLinearHeadingInterpolation(ShootPose.getHeading(), PrepGather2.getHeading())
                .addPath(new BezierLine(PrepGather2, FinishGather2))
                .setLinearHeadingInterpolation(PrepGather2.getHeading(), FinishGather2.getHeading())
                .build();

        Shootpath2 = simplePath(FinishGather2,ShootPose);
//
//        prepGatherPath2 = simplePath(ShootPose1,PrepGather2);
//
//        finishGatherPath2 = simplePath(PrepGather2, FinishGather2);
        prepGatherPath2 = drive.follower.pathBuilder()

                .addPath(new BezierLine(ShootPose, PrepGather3))
                .setLinearHeadingInterpolation(ShootPose.getHeading(), PrepGather3.getHeading())
                .addPath(new BezierLine(PrepGather3, FinishGather3))
                .setLinearHeadingInterpolation(PrepGather3.getHeading(), FinishGather3.getHeading())
                .build();

        Shootpath3 = simplePath(FinishGather3,ShootPose);

        GatePath1 = drive.follower.pathBuilder()
//                .setTValueConstraint(0.95)
                .addPath(new BezierLine(ShootPose, GatePassby))
                .setLinearHeadingInterpolation(ShootPose.getHeading(), GatePassby.getHeading())
                .addPath(new BezierLine(GatePassby, GatePassby2))
                .setLinearHeadingInterpolation(GatePassby.getHeading(), GatePassby2.getHeading())

                .build();
        GatePath2 = drive.follower.pathBuilder()
//                .setTValueConstraint(0.997)
                .addPath(new BezierLine(GatePassby2, GatePose))
                .setLinearHeadingInterpolation(GatePassby2.getHeading(), GatePose.getHeading())
                .build();

       GateShoot =
               drive.follower.pathBuilder()
                       .addPath(new BezierLine(GatePose, ShootPose))
                       .setLinearHeadingInterpolation(GatePose.getHeading(), ShootPose.getHeading())
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

                .addPath(new BezierLine(ShootPose, PrepGather1))
                .setLinearHeadingInterpolation(ShootPose.getHeading(), PrepGather1.getHeading())
                .addPath(new BezierLine(PrepGather1, FinishGather1))
                .setLinearHeadingInterpolation(PrepGather1.getHeading(), FinishGather1.getHeading())
                .setTValueConstraint(0.90)
                .build();

        Shootpath4 = drive.follower.pathBuilder()
                .addPath(new BezierLine(FinishGather1,Park))
                .setLinearHeadingInterpolation(FinishGather1.getHeading(),Park.getHeading())
                .setBrakingStrength(1.3)
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
                shooter.offset = 15;
                //shooter.autoLonger = false;
                //shooter.setShooterStatus(Shooter.ShooterStatus.);
                drive.follower.followPath(Shootpath1,0.8,true);
                setPathState(1);

                break;
            case 1:
                if(!drive.follower.isBusy()) {
                }
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
                        if(shooter.getTransDis()>18){
                            checkcounter -=1;
                        }
                        else{
                            checkcounter = checkcount;
                        }
                        if(checkcounter<0||timer.getElapsedTimeSeconds()> shoottime+1.5){
                            shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                            intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                            // turret.autopos = 0;

                            setPathState(2);
                        }

                    }
                    break;
            //1st shooting________________________________________________
            case 2:
                if(!drive.follower.isBusy()) {
                    shooter.offset = 0;
                    shooter.offset = 0;
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
                    // turret.isManeulCentering = true;
                    // turret.centeringDir = false;
                    //drive.follower.followPath(finishGatherPath1,1,false);

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
                        if(shooter.getTransDis()>18){
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
                    shooter.setShooterStatus(Shooter.ShooterStatus.Stop);
                    intake.setIntakeState(Intake.IntakeTransferState.Intake_Steady);
                    shooter.periodic();
                    drive.follower.followPath(GatePath1,1,false);
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
                if(!drive.follower.isBusy()){
                if (!firstshooting) {
                    timer.resetTimer();
                    firstshooting = true;
                    break;
                }
                else{
                    if(gatetimer.getElapsedTimeSeconds()> 0.2&&timer.getElapsedTimeSeconds()< stoptime){
                        intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                    }
                    if(timer.getElapsedTimeSeconds()> stoptime){
                        intake.gatepos = false;
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
                    if (!firstshooting) {
                        shooter.updateFocused(true);

                        timer.resetTimer();
                        firstshooting = true;
                    }
                    else{
                        if(timer.getElapsedTimeSeconds()>(waittime)&&timer.getElapsedTimeSeconds()<(shoottime+0.5)){
                            shooter.setShooterStatus(Shooter.ShooterStatus.Shooting);
                        }
                        if(shooter.getTransDis()>18){
                            checkcounter -=1;
                        }
                        else{
                            checkcounter = checkcount;
                        }
                        if(checkcounter<0||timer.getElapsedTimeSeconds()> shoottime){
                            shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                            intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                            setPathState(12);
                        }

                    }
                    break;

                }
                break;
            //3rd shooting________________________________________________
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
                        if(shooter.getTransDis()>18){
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
            //4th shooting________________________________________________
            case 16:
                if(!drive.follower.isBusy()) {
                    shooter.setShooterStatus(Shooter.ShooterStatus.Stop);
                    intake.setIntakeState(Intake.IntakeTransferState.Intake_Steady);
                    shooter.periodic();
                    drive.follower.followPath(GatePath1,1,false);
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
                if(!drive.follower.isBusy()){
                    if (!firstshooting) {
                        timer.resetTimer();
                        firstshooting = true;
                        break;
                    }
                    else{
                        if(gatetimer.getElapsedTimeSeconds()> 0.2&&timer.getElapsedTimeSeconds()< stoptime){
                            intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                        }
                        if(timer.getElapsedTimeSeconds()> stoptime){
                            intake.gatepos = false;
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
                        shooter.updateFocused(true);

                        timer.resetTimer();
                        firstshooting = true;
                    }
                    else{
                        if(timer.getElapsedTimeSeconds()>(waittime)&&timer.getElapsedTimeSeconds()<(shoottime+0.5)){
                            shooter.setShooterStatus(Shooter.ShooterStatus.Shooting);
                        }
                        if(shooter.getTransDis()>18){
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

            //4th shooting________________________________________________
            case 20:
                if(!drive.follower.isBusy()) {
                    shooter.setShooterStatus(Shooter.ShooterStatus.Stop);
                    intake.setIntakeState(Intake.IntakeTransferState.Intake_Steady);
                    shooter.periodic();
                    drive.follower.followPath(GatePath1,1,false);
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
                if(!drive.follower.isBusy()){
                    if (!firstshooting) {
                        timer.resetTimer();
                        firstshooting = true;
                        break;
                    }
                    else{
                        if(gatetimer.getElapsedTimeSeconds()> 0.2&&timer.getElapsedTimeSeconds()< stoptime){
                            intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                        }
                        if(timer.getElapsedTimeSeconds()> stoptime){
                            intake.gatepos = false;
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
                        if(shooter.getTransDis()>18){
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




            case 24:
                if(!drive.follower.isBusy()) {
                    //  turret.autopos = 0;
                    firstshooting = false;
                    shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                    intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                    shooter.periodic();
                    drive.follower.followPath(prepGatherPath3);
                    setPathState(25);
                }
                break;
            case 25:
                if(!drive.follower.isBusy()) {
                    // turret.isManeulCentering = true;
                    // turret.centeringDir = false;
                    //drive.follower.followPath(finishGatherPath1,1,false);
                    setPathState(26);
                }
                break;
            case 26:
                if(!drive.follower.isBusy()){
                    //  turret.isManeulCentering = false;
                    // turret.centeringDir = false;
                    shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                    intake.setIntakeState(Intake.IntakeTransferState.Intake_Steady);
                    drive.follower.followPath(Shootpath4,1,true);
                    firstshooting = false;
                    setPathState(27);
                }
                break;
            case 27:
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
                            shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                            intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                            firstshooting = false;
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
                Drivetrain.lastPose = drive.follower.getPose();
                Drivetrain.TredFblue = false;
                if(!drive.follower.isBusy()){
                    shooter.offset = -25;
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
            turret.aimangle = drive.getturretangle()+toRadians(3);

            turret.updateAutoShoot(true);
            //turret.tx = limelight.getTx();

        }
        else{
            turret.updateAutoShoot(false);
        }
        autonomousPathUpdate();

//        // Feedback to Driver Hub for debugging
//        telemetry.addData("realRPM", shooter.getFlyWheelRPM());
//        telemetry.addData("path state", pathState);
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
        pathTimer = new Timer();
        timer = new Timer();
        gatetimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        drive = new Drivetrain(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        limelight = new MyLimelight(hardwareMap);
        limelight.initPatternPipeline();
        limelight.startDetect();
        //intake.setIntakeState(Intake.IntakeTransferState.Intake_Steady);
        shooter.setShooterStatus(Shooter.ShooterStatus.Stop);
        turret = new Turret(hardwareMap);
        buildPaths();
        //drive.follower.setStartingPose(startPose);
        drive.follower.setPose(startPose);
        drive.blueinit();

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