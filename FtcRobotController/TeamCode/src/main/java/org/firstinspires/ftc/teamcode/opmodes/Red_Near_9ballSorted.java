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
@Autonomous(name = "Red_Near_9ballsort_gate")

public class Red_Near_9ballSorted extends OpMode {

    //private drive.follower drive.follower;
    private Drivetrain drive;
    private Timer pathTimer, actionTimer, opmodeTimer, timer,gatetimer;
    //private final ElapsedTime timer  = new ElapsedTime();

    private int pathState = 0;

    private final Pose sortpassPose = new Pose(106.4926,114.8010,0.6409);
    private final Pose sortPose = new Pose(104.4926,112.8010,0.6409);
    private final Pose patternPose = new Pose(76.8341,73.2160,1.7731);
    private final Pose startPose = new Pose(112.6333, 115.9616, -0.99945); // Start Pose of our robot.
    private final Pose PrepGather1 = new Pose(91.9908, +28.6053+5, 0);
    private final Pose FinishGather1 = new Pose(114.9794, 28.6053, 0);

    private final Pose PrepGather2 = new Pose(81.9908, 52.0297+5, 0);

    private final Pose FinishGather2 = new Pose(118.9794, 52.0297, 0);
    private final Pose GatePassby = new Pose(104.9794, 57.7386, 0);//real pass by
    private final Pose GatePassby2 = new Pose(118.6561, 55.5252, 0.30050);//hit gate
    private final Pose GatePose = new Pose(120.6561, 55.0, 0.30050);//pickup
    private final Pose ShootPose = new Pose(79.1620, 70.80 ,0);

    private final Pose PrepGather3 = new Pose(81.9908, 75.8070, 0);//accounted for overshoot

    private final Pose FinishGather3 = new Pose(119.9794, 75.8070, 0);

    private final Pose Park = new Pose(75.3860, 95.5130, -0.7830);

    private boolean firstshooting = false;

    private int[] indexnum = {0,1};
    private int[][] rowPickup = {{1,2,3},{2,3,1},{3,1,2}};

    public double turretoff = -3;
    private double gatePathPower = 1;
    private PathChain sortShoot,GateShoot,GatePath1,GatePath2, Shootpath1,Shootpath2, Shootpath3,Shootpath4,Shootpath5, lastOutPath;
    private PathChain prepGatherPath6,prepGatherPath1,finishGatherPath6,Shootpath6, prepGatherPath2, prepGatherPath3, prepGatherPath4;

    private PathChain finishGatherPath1,finishGatherPath2,finishGatherPath3,finishGatherPath4;
    public Intake intake;
    public Shooter shooter;
    public MyLimelight limelight;
    public Scheduler scheduler;

    public Turret turret;

    public static double stoptime = 2;
    public static double shoottime = 2.5;

    public static double waittime = 0.5;
    public static double checkcount = 6;

    public static double followingtime = 1.5;

    public double checkcounter = checkcount;

    public int pattern = 0;



    public  PathChain simplePath(Pose a, Pose b){
        return drive.follower.pathBuilder()
                .addPath(new BezierLine(a, b))
                .setLinearHeadingInterpolation(a.getHeading(), b.getHeading())
                .build();
    }

    public  PathChain simpleconstPath(Pose a, Pose b){
        return drive.follower.pathBuilder()
                .addPath(new BezierLine(a, b))
                .setConstantHeadingInterpolation(b.getHeading())
                .build();
    }

    public  PathChain simpleindexPath(int a){

        if(a == 1){
            return drive.follower.pathBuilder()
                    .addPath(new BezierCurve(FinishGather1, PrepGather1))
                    .setLinearHeadingInterpolation(FinishGather1.getHeading(), PrepGather1.getHeading())
                    .addPath(new BezierCurve(PrepGather1, ShootPose))
                    .setLinearHeadingInterpolation(PrepGather1.getHeading(), ShootPose.getHeading())
                    .addPath(new BezierCurve(ShootPose, sortPose))
                    .setConstantHeadingInterpolation(sortPose.getHeading())
                    .build();
        }
        else if(a == 2){
            return drive.follower.pathBuilder()
                    .addPath(new BezierCurve(FinishGather2, PrepGather2))
                    .setLinearHeadingInterpolation(FinishGather2.getHeading(), PrepGather2.getHeading())
                    .addPath(new BezierCurve(PrepGather2, ShootPose))
                    .setLinearHeadingInterpolation(PrepGather2.getHeading(), ShootPose.getHeading())
                    .addPath(new BezierCurve(ShootPose, sortPose))
                    .setLinearHeadingInterpolation(ShootPose.getHeading(), sortPose.getHeading())
                    .build();
        }
        else{
            return drive.follower.pathBuilder()
                    .addPath(new BezierCurve(FinishGather3, sortPose))
                    .setLinearHeadingInterpolation(FinishGather3.getHeading(), sortPose.getHeading())
                    .build();
        }
    }
    public void buildPaths() {

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        Shootpath1 = drive.follower.pathBuilder()
                .addPath(new BezierCurve(startPose, patternPose))
                .setConstantHeadingInterpolation(patternPose.getHeading())
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

                .addPath(new BezierCurve(ShootPose, PrepGather1))
                .setLinearHeadingInterpolation(ShootPose.getHeading(), PrepGather1.getHeading())
                .addPath(new BezierCurve(PrepGather1, FinishGather1))
                .setLinearHeadingInterpolation(PrepGather1.getHeading(), FinishGather1.getHeading())
                .build();

        sortShoot = drive.follower.pathBuilder()

                .addPath(new BezierCurve(sortPose, sortpassPose))
                .setLinearHeadingInterpolation(sortPose.getHeading(), sortpassPose.getHeading())
                .addPath(new BezierCurve(sortpassPose, ShootPose))
                .setLinearHeadingInterpolation(sortpassPose.getHeading(), ShootPose.getHeading())
                .build();

        Shootpath2 = drive.follower.pathBuilder()

                .addPath(new BezierCurve(FinishGather1, PrepGather1))
                .setLinearHeadingInterpolation(FinishGather1.getHeading(), PrepGather1.getHeading())
                .addPath(new BezierCurve(PrepGather1, ShootPose))
                .setLinearHeadingInterpolation(PrepGather1.getHeading(), ShootPose.getHeading())
                .build();
//
//        prepGatherPath2 = simplePath(ShootPose1,PrepGather2);
//
//        finishGatherPath2 = simplePath(PrepGather2, FinishGather2);
        prepGatherPath2 = drive.follower.pathBuilder()

                .addPath(new BezierCurve(ShootPose, PrepGather2))
                .setLinearHeadingInterpolation(ShootPose.getHeading(), PrepGather2.getHeading())
                .addPath(new BezierCurve(PrepGather2, FinishGather2))
                .setLinearHeadingInterpolation(PrepGather2.getHeading(), FinishGather2.getHeading())
                .build();

        Shootpath3 = simplePath(FinishGather2,ShootPose);

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

                .addPath(new BezierCurve(ShootPose, PrepGather3))
                .setLinearHeadingInterpolation(ShootPose.getHeading(), PrepGather3.getHeading())
                .addPath(new BezierLine(PrepGather3, FinishGather3))
                .setLinearHeadingInterpolation(PrepGather3.getHeading(), FinishGather3.getHeading())
                .setTValueConstraint(0.85)
                .setBrakingStrength(1)
                .build();

        Shootpath4 = drive.follower.pathBuilder()
                .addPath(new BezierLine(FinishGather3,ShootPose))
//                .setLinearHeadingInterpolation(FinishGather1.getHeading(),Park.getHeading())
                .setConstantHeadingInterpolation(ShootPose.getHeading())
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
                shooter.offset = -10;
                shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                //shooter.autoLonger = false;
                //shooter.setShooterStatus(Shooter.ShooterStatus.);
                drive.follower.followPath(Shootpath1,1,true);
                limelight.initRealPattern();
                setPathState(1);

                break;
            case 1:
                if(!drive.follower.isBusy()) {
                    shooter.forceShooting = true;
                    if (!firstshooting) {

                        shooter.updateFocused(true);

                        timer.resetTimer();
                        firstshooting = true;
                    } else {
                        if (timer.getElapsedTimeSeconds() < waittime) {
                            shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                        } else if (timer.getElapsedTimeSeconds() < (shoottime + 5.5)) {
                            shooter.setShooterStatus(Shooter.ShooterStatus.Shooting);
                        }
                        if (shooter.getTransDis() > 18) {
                            checkcounter -= 1;
                        } else {
                            checkcounter = checkcount;
                        }
                        if (checkcounter < 0 || timer.getElapsedTimeSeconds() > shoottime + 1.5) {
                            shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                            intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                            // turret.autopos = 0;


                            setPathState(2);
                        }

                    }
                }
                break;

            case 2:
                if(!drive.follower.isBusy()) {
                    shooter.offset = -10;
                    intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                    if(limelight.patternnum()>0){
                        pattern = limelight.patternnum()-1;
                        setPathState(3);
                    }
                }
                break;
            case 3:
                if(!drive.follower.isBusy()) {
                    if (rowPickup[pattern][0] == 1) {
                       //
                        // turretoff = 0;
                        // shooter.offset = -45;
                        //  turret.autopos = 0;
                        firstshooting = false;
                        shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                        intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                        shooter.periodic();
                        drive.follower.followPath(prepGatherPath1);
                        setPathState(4);
                    } else if (rowPickup[pattern][0] == 2) {
                    //    //turretoff = 0;
                        // shooter.offset = -45;
                        //  turret.autopos = 0;
                        firstshooting = false;
                        shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                        intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                        shooter.periodic();
                        drive.follower.followPath(prepGatherPath2);
                        setPathState(4);
                    } else if (rowPickup[pattern][0] == 3) {
                        //turretoff = 0;
                        // shooter.offset = -45;
                        //  turret.autopos = 0;
                        firstshooting = false;
                        shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                        intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                        shooter.periodic();
                        drive.follower.followPath(prepGatherPath3);
                        setPathState(4);
                    }
                }
                break;

            case 4:
                if(!drive.follower.isBusy()) {
                    if (rowPickup[pattern][0] == 1) {
                        shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                        intake.setIntakeState(Intake.IntakeTransferState.Intake_Steady);
                        drive.follower.followPath(Shootpath2, 1, true);
                        firstshooting = false;
                        setPathState(5);
                    } else if (rowPickup[pattern][0] == 2) {
                        shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                        intake.setIntakeState(Intake.IntakeTransferState.Intake_Steady);
                        drive.follower.followPath(Shootpath3, 1, true);
                        firstshooting = false;
                        setPathState(5);
                    } else if (rowPickup[pattern][0] == 3) {
                        shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                        intake.setIntakeState(Intake.IntakeTransferState.Intake_Steady);
                        drive.follower.followPath(Shootpath4, 1, true);
                        firstshooting = false;
                        setPathState(5);
                    }
                }
                break;


            case 5:
                if(!drive.follower.isBusy()) {
                    if (!firstshooting) {
                        shooter.forceShooting = false;
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
                        if(timer.getElapsedTimeSeconds()<2){
                            intake.transferSpeed = 0.8;
                        }
                        else{
                            intake.transferSpeed = 1;
                        }

                    }
                    break;

                }
                break;
            case 6:
                if(!drive.follower.isBusy()) {
                    if (rowPickup[pattern][1] == 1) {
                        //turretoff = 0;
                        // shooter.offset = -45;
                        //  turret.autopos = 0;
                        firstshooting = false;
                        shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                        intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                        shooter.periodic();
                        drive.follower.followPath(prepGatherPath1);
                        setPathState(7);
                    } else if (rowPickup[pattern][1] == 2) {
                        //turretoff = 0;
                        // shooter.offset = -45;
                        //  turret.autopos = 0;
                        firstshooting = false;
                        shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                        intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                        shooter.periodic();
                        drive.follower.followPath(prepGatherPath2);
                        setPathState(7);
                    } else if (rowPickup[pattern][1] == 3) {
                        //turretoff = 0;
                        // shooter.offset = -45;
                        //  turret.autopos = 0;
                        firstshooting = false;
                        shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                        intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                        shooter.periodic();
                        drive.follower.followPath(prepGatherPath3);
                        setPathState(7);
                    }
                }
                firstshooting = false;
                break;

            case 7:
                if(!drive.follower.isBusy()) {
                    shooter.idleSpeed = 400;
                    shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                    intake.setIntakeState(Intake.IntakeTransferState.Intake_Steady);
                    turret.isIndexing = true;
                    drive.follower.followPath(simpleindexPath(rowPickup[pattern][1]), 1, true);
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
                        if(timer.getElapsedTimeSeconds()> 0.5){
                            shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                            intake.setIntakeState(Intake.IntakeTransferState.Intake_Steady);
                            firstshooting = false;
                            setPathState(8);
                        }

                    }
                    break;

                }
                break;

            case 8:
                sortedflag = false;
                sortflag = false;
                if (!firstshooting) {
//                        shooter.updateFocused(true);

                    timer.resetTimer();
                    firstshooting = true;
                }

                if((!intake.fronthasballRaw())&&timer.getElapsedTimeSeconds()<1){
                    intake.setIntakeState(Intake.IntakeTransferState.Suck_In_slow);
                }
                else {
                    firstshooting = false;
                    setPathState(9);
                }
                break;
            case 9:
                if (!firstshooting) {
                    shooter.updateFocused(true);
                    toggleSortingMode(true);
                    timer.resetTimer();
                    firstshooting = true;
                }
                else{
                    if(timer.getElapsedTimeSeconds()<1.5){
                        toggleSortingMode(true);
                    }
                    else{
                        toggleSortingMode(false);
                        setPathState(10);
                    }

                }
                break;

            case 10:
                if(!drive.follower.isBusy()) {
                    shooter.idleSpeed = 2600;
                    turret.isIndexing = false;
                    shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                    intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                    drive.follower.followPath(sortShoot, 1, true);
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
                        if(timer.getElapsedTimeSeconds()>waittime&&timer.getElapsedTimeSeconds()<shoottime){
                            shooter.setShooterStatus(Shooter.ShooterStatus.Shooting);
                        }
                        if(shooter.getTransDis()>18){
                            checkcounter -=1;
                        }
                        else{
                            checkcounter = checkcount;
                        }
                        if(timer.getElapsedTimeSeconds()> shoottime){
                            shooter.setShooterStatus(Shooter.ShooterStatus.Stop);
                            intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                            firstshooting = false;
                            //drive.follower.followPath(simplePath(drive.follower.getPose(),Park));
                            setPathState(12);
                        }

                        if(timer.getElapsedTimeSeconds()<2){
                            intake.transferSpeed = 0.9;
                        }
                        else{
                            intake.transferSpeed = 1;
                        }

                    }
                    break;

                }
                break;

            case 12:
                if(!drive.follower.isBusy()) {
                    if (rowPickup[pattern][2] == 1) {
                        //turretoff = 0;
                        // shooter.offset = -45;
                        //  turret.autopos = 0;
                        firstshooting = false;
                        shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                        intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                        shooter.periodic();
                        drive.follower.followPath(prepGatherPath1);
                        setPathState(13);
                    } else if (rowPickup[pattern][2] == 2) {
                        //turretoff = 0;
                        // shooter.offset = -45;
                        //  turret.autopos = 0;
                        firstshooting = false;
                        shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                        intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                        shooter.periodic();
                        drive.follower.followPath(prepGatherPath2);
                        setPathState(13);
                    } else if (rowPickup[pattern][2] == 3) {
                        //turretoff = 0;
                        // shooter.offset = -45;
                        //  turret.autopos = 0;
                        firstshooting = false;
                        shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                        intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                        shooter.periodic();
                        drive.follower.followPath(prepGatherPath3);
                        setPathState(13);
                    }
                }
                break;

            case 13:
                if(!drive.follower.isBusy()) {
                    shooter.offset = -20; 
                    shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                    intake.setIntakeState(Intake.IntakeTransferState.Intake_Steady);
                    drive.follower.followPath(simpleconstPath(drive.follower.getPose(),Park), 1, true);
                    firstshooting = false;

                    setPathState(14);
                }
                break;
            case 14:
                if(!drive.follower.isBusy()) {
                    shooter.forceShooting = true;
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
                        if(timer.getElapsedTimeSeconds()> shoottime){
                            shooter.setShooterStatus(Shooter.ShooterStatus.Stop);
                            intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
                            firstshooting = false;
                            //drive.follower.followPath(simplePath(drive.follower.getPose(),Park));
                            setPathState(120);
                        }

                        if(timer.getElapsedTimeSeconds()<2){
                            intake.transferSpeed = 0.9;
                        }
                        else{
                            intake.transferSpeed = 1;
                        }

                    }
                    break;

                }
                break;

            case 120:
                intake.autoIntakeUp = false;
                Drivetrain.lastPose = drive.follower.getPose();
                Drivetrain.TredFblue = true;
                if(!drive.follower.isBusy()){
                    shooter.offset = 0;
                    resetSubsystemsForTeleop();
                    Drivetrain.lastPose = drive.follower.getPose();
                    Drivetrain.TredFblue = true;
                    //setPathState(28);
                    break;

                }
                break;

        }

    }

    public void toggleSortingMode(boolean sort){
        // if(turret.posDiff()<300&&turret.aimposition>20000) {
        if (sort) {
            shooter.sortingMode = true;
            if (!sortflag) {
                shooter.sortingSpeed = 400;
                shooter.setShooterStatus(Shooter.ShooterStatus.Shooting);
                //shooter.sortedOut = !intake.frontHasBall();
            }

            if ((!intake.frontHasBall()) || (!intake.midHasBall()) || shooter.getTransDis() > 9 || (shooter.rpmreached && shooter.getFlyWheelRPM() < 440)) {
                if (sortflag) {
                    //shooter.sortingSpeed = -1000;
                    shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                }
                sortflag = true;
            }
            shooter.periodic();
            if (sortflag && !intake.fronthasballRaw() && !sortedflag) {
                sortedflag = true;
                intake.setIntakeState(Intake.IntakeTransferState.Split_Out);
            } else {
                if(turret.isfocuedTu()) {
                    intake.setIntakeState(Intake.IntakeTransferState.Suck_In_slow_Sorting);
                }
                else{
                    intake.setIntakeState(Intake.IntakeTransferState.Intake_Steady);
                }
            }
            intake.periodic();
        } else {
            shooter.sortingMode = false;
            shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
            //shooter.sortedOut = false;
            sortflag = false;
            shooter.periodic();
            intake.setIntakeState(Intake.IntakeTransferState.Intake_Steady);
            intake.periodic();
        }
        //}
    }
    public boolean sortflag = false;
    public boolean sortedflag = false;
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
        shooter.isfocused = turret.isfocuedTu();
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

        if(pathState!=8&&pathState!=9) {
            if (shooter.shooterStatus == Shooter.ShooterStatus.Shooting) {
                intake.updateAutoshoot(true);
//            if(shooter.reverIntake){
//                intake.updateAutoshoot(false);
//                intake.setIntakeState(Intake.IntakeTransferState.Split_Out);
//            }
                intake.updateautotranse(shooter.isAtTargetRPM());
//            shooter.updateDis(limelight.getDis());
//            shooter.updateFocused(limelight.isFocused());
                //shooter.updateFocused(true);
            } else {
                intake.updateAutoshoot(false);

            }
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
        telemetry.addData("patternnum", limelight.patternnum());
        telemetry.addData("timer", timer.getElapsedTimeSeconds());
        telemetry.addData("state", pathState);
        telemetry.addData("pattern", pattern);
        telemetry.update();
    }


    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
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
        limelight.initRealPattern();
        limelight.startDetect();
        //intake.setIntakeState(Intake.IntakeTransferState.Intake_Steady);
        shooter.setShooterStatus(Shooter.ShooterStatus.Stop);
        turret = new Turret(hardwareMap,false);
        buildPaths();
        //drive.follower.setStartingPose(startPose);
        shooter.forceShooting = false;
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