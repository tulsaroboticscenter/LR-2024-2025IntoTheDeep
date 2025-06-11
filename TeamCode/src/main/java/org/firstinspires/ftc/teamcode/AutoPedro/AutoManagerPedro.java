package org.firstinspires.ftc.teamcode.AutoPedro;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Enums.AnimationType;
import org.firstinspires.ftc.teamcode.Enums.AutoLocation;
import org.firstinspires.ftc.teamcode.Enums.GrabAngle;
import org.firstinspires.ftc.teamcode.Enums.TeleopMode;
import org.firstinspires.ftc.teamcode.Enums.WristAngle;
import org.firstinspires.ftc.teamcode.Hardware.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Hardware.Params;
import org.firstinspires.ftc.teamcode.bedroBathing.follower.Follower;
import org.firstinspires.ftc.teamcode.bedroBathing.localization.Pose;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.DoubleHeadingTangentPath;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.Point;

import java.util.concurrent.TimeUnit;

@Config
public class AutoManagerPedro {
    public PathBuilder toBucketPath;
    //    public PathBuilder backupPath;
    public PathBuilder turnPath;
    public PathBuilder bucketScorePathFromIntake1;
    public PathBuilder bucketScorePathFromIntake2;
    public PathBuilder bucketScorePathFromIntake3;
    public PathBuilder bucketScorePathFromIntake4;
    public PathBuilder bucketScorePathS1;
    public PathBuilder bucketScoreFromSubPath;
    public PathBuilder park;
    public PathBuilder intakeYellow1;
//    public PathBuilder intakeYellow1;
    public PathBuilder intakeYellow2;
    public PathBuilder intakeYellow3;
    public PathBuilder intakeYellow4;
    public PathBuilder specScore1;
    public PathBuilder specScore2;
    public PathBuilder specScore3;
    public PathBuilder specScore4;
    public PathBuilder specScore5;
    public PathBuilder specScore6;

    public PathBuilder backupFromSub;
    public PathBuilder depositToHP;

    public PathBuilder intakeSpec1;
    public PathBuilder pushSamples;

    public PathBuilder newPushSample1P1;
    public PathBuilder newPushSample1P2;
    public PathBuilder newPushSample2P1;
    public PathBuilder newPushSample2P2;
    public PathBuilder newPushSample3P1;
    public PathBuilder newPushSample3P2;

    public PathBuilder intakeSpec2;
    public PathBuilder intakeSpec3;
    public PathBuilder intakeSpec4;
    public PathBuilder specBackup;
    private Follower follower;
    private HWProfile robot;
    private ArmSubsystem arm;
    private TeleopMode currentMode = TeleopMode.IDLE;
    private IntakeSubsystem intake;
    private Point startPose = new Point(0, 0, Point.CARTESIAN);
    private LinearOpMode opMode;
    private boolean pathingDisabled = false;
    // zero is two tiles from bucket to the leftmost of the tile
    public Pose start_4_0_V1 = new Pose(.399, -.48, Math.toRadians(-90));
    public Pose start_3_1_V1 = new Pose(0, -23.6, Math.toRadians(0));
    public Pose start_Speci_V1 = new Pose(4.3, -49.9, Math.toRadians(180));
    public Pose start_Speci_V2 = new Pose(0, -47.5, Math.toRadians(0));

    public Pose sampleGrabOffsetSpecimen = new Pose(13, -21.5);

    private Point leftScoreLocation = new Point(19, 109, Point.CARTESIAN);
    private Runnable updateAction;
    private MultipleTelemetry telemetry;
    private Params params;
    private boolean specimenGrabFromSub = false;
    private double slowdownT = 1;
    private double slowdownSpeed = 1;
    public static double xP = 0.005;
    public static double xD = 0.0009;
    public static double homeTolerance = 5;
    private PIDController xController = new PIDController(xP, 0, xD);
    private VoltageSensor vSensor;
    private boolean useDistanceReloc = false;
    private Pose oldPose = new Pose();

    public AutoManagerPedro(LinearOpMode opMode, Follower _follower, Runnable updateAction, ArmSubsystem arm, IntakeSubsystem intake, HWProfile robot) {
        follower = _follower;
        this.opMode = opMode;
        this.updateAction = updateAction;
        this.arm = arm;
        this.intake = intake;
        this.telemetry = new MultipleTelemetry(opMode.telemetry, FtcDashboard.getInstance().getTelemetry());
        this.robot = robot;
        this.params = new Params();
    }

    public void setSpecimenGrabFromSub(boolean set) {
        this.specimenGrabFromSub = set;
    }

    private void run(PathChain path, boolean correct) {
        if (!pathingDisabled) follower.followPath(path, correct);

        while (follower.isBusy()) {
            try {
                update();

//                double currentT = follower.getCurrentTValue();

//                if(currentT >= slowdownT) {
//                    follower.setMaxPower(slowdownSpeed);
//                }

                if (opMode.isStopRequested() || !opMode.opModeIsActive()) {
                    follower.breakFollowing();
                    break;
                }
            } catch (Exception e) {
                follower.breakFollowing();
                break;
            }
        }
    }


    public void runPath(PathChain path) {
        run(path, true);
    }

    public void runPath(PathBuilder builder) {
        run(builder.build(), true);
    }

    public void runPath(PathChain path, boolean correct) {
        run(path, correct);
    }

    public void runPath(PathBuilder builder, boolean correct) {
        run(builder.build(), correct);
    }

    public void update() {
        if(useDistanceReloc) relocalizeX(params.DISTANCE_ONE_SPEC_OFFSET);

        Params.AUTO_END_HEADING = Math.toDegrees(follower.getPose().getHeading());

        follower.update();
        updateAction.run();

//        telemetry.addData("get closest T: ", follower.getCurrentTValue());
//        telemetry.addData("get path type: ", follower.getCurrentPath().pathType());
////            telemetry.update();
        try {
//            follower.telemetryDebug(telemetry);
        } catch (Exception e) {

        }
    }

    public boolean isBusy() {
        return follower.isBusy();
    }

    public void disablePathing(boolean disabled) {
        pathingDisabled = disabled;
    }

    public void holdCurrentPoint() {
        BezierPoint currentPoint = new BezierPoint(follower.getCurrentPath().getLastControlPoint());
        follower.holdPoint(currentPoint, follower.getCurrentPath().getPathEndHeadingConstraint());
    }

    public void setSpeed(double speed) {
        follower.setMaxPower(speed);
    }

    public void holdPoint(Pose pose) {
        follower.holdPoint(pose);
    }

    public void buildPaths(AutoLocation autoLocation) {
        if (autoLocation == AutoLocation.PEDRO_LEFT_SAMPLE_V1 || autoLocation == AutoLocation.PEDRO_LEFT_3_1_V1) {
            toBucketPath = new PathBuilder()
                    .addPath(new Path(
                            new BezierLine(
                                    new Point(0, 0, Point.CARTESIAN),
                                    new Point(15, 9, Point.CARTESIAN)
                            )
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(140));

//            toBucketPath.getPath(0).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(140));
            double startHeading = 0;

            if (autoLocation == AutoLocation.PEDRO_LEFT_3_1_V1) {
                startPose = new Point(start_3_1_V1.getX(), start_3_1_V1.getY(), Point.CARTESIAN);
                startHeading = start_3_1_V1.getHeading();

                specScore1 = new PathBuilder()
                        .addPath(new Path(
                                new BezierCurve(
                                        startPose,
                                        new Point(32.5 /*2.75*/, -45/*19.5*/, Point.CARTESIAN),
                                        new Point(32.5 /*2.75*/, -40/*19.5*/, Point.CARTESIAN) //33
                                )
                        ))
                        .addTemporalCallback(0, () -> {
                            currentMode = TeleopMode.SPECIMEN_SCORE;
                            arm.setTeleopMode(currentMode);
                            arm.setAnimationType(AnimationType.NONE);
                            arm.update(opMode.opModeIsActive());

                            arm.setArmPositionSpecimen(100);
                            arm.setSlidesPositionSpecimen(10.5);
                        })
                        .setZeroPowerAccelerationMultiplier(.5)
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .setPathEndTimeoutConstraint(100)
                        .setPathEndTValueConstraint(.9);

                specBackup = new PathBuilder()
                        .addPath(new Path(
                                new BezierCurve(
                                        specScore1.build().getPath(specScore1.build().size() - 1).getLastControlPoint(),
                                        new Point(0, 3, Point.CARTESIAN),
                                        new Point(16, 6, Point.CARTESIAN)
                                )
                        ))
                        .addParametricCallback(.35, () -> {
                            currentMode = TeleopMode.INTAKE;
                            arm.setTeleopMode(currentMode);
                            arm.update(opMode.opModeIsActive());
                        })
                        .addParametricCallback(.99, () -> {
//                            relocalizeX();
                        })
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .setPathEndTimeoutConstraint(100)
                        .setPathEndTValueConstraint(.85);
            } else if (autoLocation == AutoLocation.PEDRO_LEFT_SAMPLE_V1) {
                startPose = new Point(start_4_0_V1.getX(), start_4_0_V1.getY(), Point.CARTESIAN);
                startHeading = start_4_0_V1.getHeading();
            }

            Point bucketFinalPoint = new Point(10 /*2.75*/, 17/*19.5*/, Point.CARTESIAN);
            double scoreHeading = -45;
            double bucketScoreZPM = 7;

            bucketScorePathS1 = new PathBuilder()
                    .addPath(new Path(
                            new BezierLine(
                                    startPose,
                                    new Point(7 /*2.75*/, 13/*19.5*/, Point.CARTESIAN)
                            )
                    ))
                    .setZeroPowerAccelerationMultiplier(bucketScoreZPM)
                    .setLinearHeadingInterpolation(startHeading, Math.toRadians(scoreHeading))
                    .setPathEndTValueConstraint(.8);

//            bucketScorePathFromIntake1.getPath(0).setConstantHeadingInterpolation(Math.toRadians(140));
//            bucketScorePathFromIntake1.getPath(0).setPathEndTValueConstraint(.9);

//            backupPath = new PathBuilder()
//                    .addPath(new Path(
//                            new BezierLine(
//                                    bucketScorePathFromIntake1.build().getPath(0).getLastControlPoint(),
//                                    new Point(9, 11, Point.CARTESIAN)
//                            )
//                    ))
//                    .setConstantHeadingInterpolation(Math.toRadians(140));

//            backupPath.getPath(0).setConstantHeadingInterpolation(Math.toRadians(140));
//            backupPath.getPath(0).setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(0));


//            turnPath = new PathBuilder()
//                    .addPath(new Path(
//                            new BezierPoint(
//                                    backupPath.getPath(backupPath.size() - 1).getLastControlPoint()
//                            )
//                    )).build();
//
//            turnPath.getPath(0).setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(0));

            Point intakeYellow1Start = bucketFinalPoint;
            double intakeYellow1StartHeading = scoreHeading;
            double xOffset = 0;

            if (autoLocation == AutoLocation.PEDRO_LEFT_3_1_V1) {
                intakeYellow1Start = specBackup.build().getPath(0).getLastControlPoint();
                intakeYellow1StartHeading = 0;
                xOffset = 0;
            }

            double intakeZPM = 3;
            double intakeX = 22;

            intakeYellow1 = new PathBuilder()
                    .addPath(new Path(
                            new BezierLine(
                                    intakeYellow1Start,
                                    new Point(intakeX, 11.5, Point.CARTESIAN) //24.25 10
                            )
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(intakeYellow1StartHeading), Math.toRadians(0), .5)
                    .addParametricCallback(.6, () -> {
                        currentMode = TeleopMode.INTAKE;
                        arm.setTeleopMode(currentMode);
                        arm.intakeDownMode();
                        if(autoLocation == AutoLocation.PEDRO_LEFT_SAMPLE_V1) {
                            arm.setAnimationType(AnimationType.NORMAL);
                        }
                        arm.update(opMode.opModeIsActive());
                    })
                    .addParametricCallback(.9, () -> {
                        setSpeed(1);
                    })
                    .setZeroPowerAccelerationMultiplier(intakeZPM)
                    .setPathEndTValueConstraint(.95)
                    .setPathEndTimeoutConstraint(100);

//            intakeYellow1.getPath(0).setConstantHeadingInterpolation(Math.toRadians(0));
//            intakeYellow1.getPath(0).setPathEndTValueConstraint(.99);
//            intakeYellow1.getPath(0).setPathEndTimeoutConstraint(500);

            intakeYellow2 = new PathBuilder()
                    .addPath(new Path(
                            new BezierLine(
                                    bucketFinalPoint,
                                    new Point(intakeX, 22, Point.CARTESIAN)
                            )
                    ))
                    .addParametricCallback(.6, () -> {
                        currentMode = TeleopMode.INTAKE;
                        arm.setTeleopMode(currentMode);
                        arm.intakeDownMode();
                        if(autoLocation == AutoLocation.PEDRO_LEFT_SAMPLE_V1) {
                            arm.setAnimationType(AnimationType.NORMAL);
                        }
                        arm.update(opMode.opModeIsActive());
                    })
                    .setLinearHeadingInterpolation(Math.toRadians(scoreHeading), Math.toRadians(0))
//                    .addPath(new Path(
//                            new BezierLine(
//                                    new Point(17, 20, Point.CARTESIAN),
//                                    new Point(17.1, 20, Point.CARTESIAN)
//                            )
//                    ))
                    .addParametricCallback(.95, () -> {
                        setSpeed(1);
                    })
//                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .setZeroPowerAccelerationMultiplier(intakeZPM)
                    .setPathEndTValueConstraint(.97)
                    .setPathEndTimeoutConstraint(100);

//            intakeYellow2.getPath(0).setConstantHeadingInterpolation(Math.toRadians(0));
//            intakeYellow2.getPath(0).setPathEndTValueConstraint(.99);
//            intakeYellow2.getPath(0).setPathEndTimeoutConstraint(500);

            intakeYellow3 = new PathBuilder()
                    .addPath(new Path(
                            new BezierLine(
                                    bucketFinalPoint,
                                    new Point(24, 22, Point.CARTESIAN)
                            )
                    ))
                    .addParametricCallback(.6, () -> {
                        currentMode = TeleopMode.INTAKE;
                        arm.setTeleopMode(currentMode);
                        arm.intakeDownMode();
                        if(autoLocation == AutoLocation.PEDRO_LEFT_SAMPLE_V1) {
                            arm.setAnimationType(AnimationType.NORMAL);
                        }
                        arm.update(opMode.opModeIsActive());
                    })
                    .setLinearHeadingInterpolation(Math.toRadians(scoreHeading), Math.toRadians(40))
//                    .setLinearHeadingInterpolation(Math.toRadians(0), .875)
                    .setZeroPowerAccelerationMultiplier(intakeZPM)
                    .setPathEndTValueConstraint(.9)
                    .setPathEndTimeoutConstraint(0);

            intakeYellow4 = new PathBuilder()
                    .addPath(new Path(
                            new BezierCurve(
                                    bucketFinalPoint,
                                    new Point(5, -30, Point.CARTESIAN),
                                    new Point(0, -60, Point.CARTESIAN)
                            )
                    ))
                    .addParametricCallback(.1, () -> {
                        currentMode = TeleopMode.INTAKE;
                        arm.setTeleopMode(currentMode);
                        arm.intakeDownMode();
                        if(autoLocation == AutoLocation.PEDRO_LEFT_SAMPLE_V1) {
                            arm.setAnimationType(AnimationType.NORMAL);
                        }
                        arm.update(opMode.opModeIsActive());
                    })
                    .setLinearHeadingInterpolation(Math.toRadians(scoreHeading), Math.toRadians(-90))
//                    .setLinearHeadingInterpolation(Math.toRadians(0), .875)
                    .setZeroPowerAccelerationMultiplier(intakeZPM)
                    .setZeroPowerAccelerationMultiplier(1)
                    .setPathEndTValueConstraint(.9)
                    .setPathEndTimeoutConstraint(0);


            bucketScorePathFromIntake1 = new PathBuilder()
                    .addPath(new Path(
                            new BezierLine(
                                    intakeYellow1.build().getPath(intakeYellow1.build().size() - 1).getLastControlPoint(),
//                                    new Point(15, 0, Point.CARTESIAN),
                                    bucketFinalPoint
                            )
                    ))
                    .addParametricCallback(.95, () -> {
                        setSpeed(1);
                    })
                    .setZeroPowerAccelerationMultiplier(bucketScoreZPM)
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(scoreHeading), .75)
//                    .setConstantHeadingInterpolation(Math.toRadians(130))
                    .setPathEndTValueConstraint(.87);

            bucketScorePathFromIntake2 = new PathBuilder()
                    .addPath(new Path(
                            new BezierLine(
                                    intakeYellow2.build().getPath(intakeYellow2.build().size() - 1).getLastControlPoint(),
//                                    new Point(15, 5, Point.CARTESIAN),
                                    bucketFinalPoint

                            )
                    ))
                    .addParametricCallback(.95, () -> {
                        setSpeed(1);
                    })
                    .setZeroPowerAccelerationMultiplier(bucketScoreZPM)
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(scoreHeading), .75)
//                    .setConstantHeadingInterpolation(Math.toRadians(130))
                    .setPathEndTValueConstraint(.87);

            bucketScorePathFromIntake3 = new PathBuilder()
                    .addPath(new Path(
                            new BezierLine(
                                    intakeYellow3.build().getPath(intakeYellow3.build().size() - 1).getLastControlPoint(),
//                                    new Point(15, 5, Point.CARTESIAN),
                                    bucketFinalPoint
                            )
                    ))
                    .addParametricCallback(.95, () -> {
                        setSpeed(1);
                    })
                    .setZeroPowerAccelerationMultiplier(bucketScoreZPM)
                    .setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(scoreHeading), .75)
//                    .setConstantHeadingInterpolation(Math.toRadians(130))
                    .setPathEndTValueConstraint(.87);

            bucketScorePathFromIntake4 = new PathBuilder()
                    .addPath(new Path(
                            new BezierCurve(
                                    intakeYellow4.build().getPath(intakeYellow3.build().size() - 1).getLastControlPoint(),
//                                    new Point(15, 5, Point.CARTESIAN),
                                    new Point(10, -10, Point.CARTESIAN),
                                    bucketFinalPoint
                            )
                    ))
                    .addParametricCallback(.95, () -> {
                        setSpeed(1);
                    })
                    .setZeroPowerAccelerationMultiplier(bucketScoreZPM)
                    .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(scoreHeading), .75)
//                    .setConstantHeadingInterpolation(Math.toRadians(130))
                    .setPathEndTValueConstraint(.87);

//            intakeYellow3.getPath(0).setLinearHeadingInterpolation(Math.toRadians(0), .875);
//            intakeYellow3.getPath(0).setPathEndTValueConstraint(.99);
//            intakeYellow3.getPath(0).setsetAniPathEndTimeoutConstraint(500);

            bucketScoreFromSubPath = new PathBuilder()
                    .addPath(new Path(
                            new BezierCurve(
                                    new Point(42.5, -13, Point.CARTESIAN),
                                    new Point(45, 15, Point.CARTESIAN),
                                    bucketFinalPoint
                            )
                    ))
                    .addTemporalCallback(.1, () -> {
                        arm.setTeleopMode(TeleopMode.BUCKET_SCORE);
//                        arm.setAnimationType(AnimationType.FAST);
                        arm.setBucket(2);
//                        arm.setAnimationType(AnimationType.NORMAL);
                        arm.update(opMode.opModeIsActive());
//                        arm.setAnimationType(AnimationType.NORMAL);
                    })
                    .setZeroPowerAccelerationMultiplier(bucketScoreZPM)
                    .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(scoreHeading), .75)
//                    .setConstantHeadingInterpolation(Math.toRadians(130))
                    .setPathEndTValueConstraint(.87);

            park = new PathBuilder()
                    .addPath(new Path(
                                    new BezierLine(
                                            bucketFinalPoint,
                                            new Point(54, 1, Point.CARTESIAN)
                                    )
                            )
                    )
                    .addParametricCallback(.4, () -> {
                        currentMode = TeleopMode.TOUCH_POLE_AUTO;
                        arm.setTeleopMode(currentMode);
                        arm.update(opMode.opModeIsActive());
                        arm.setParkArmUp(true);
                        safeSleep(1500);
//                        arm.setParkArmUp(false);
                        arm.setArmPower(0);
                        safeSleep(10000);
                    })
                    .setLinearHeadingInterpolation(Math.toRadians(scoreHeading), Math.toRadians(-90))
                    .addPath(new Path(
                            new BezierLine(
                                    new Point(54, 1, Point.CARTESIAN),
                                    new Point(54, -14, Point.CARTESIAN)
                            )
                    )).setConstantHeadingInterpolation(Math.toRadians(-90))
                    .setPathEndTValueConstraint(.9)
                    .setPathEndTimeoutConstraint(100);

//            park.getPath(0).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-90));
//            park.getPath(1).setConstantHeadingInterpolation(Math.toRadians(-90));

//            park.getPath(0).setPathEndTValueConstraint(.9);
//            park.getPath(1).setPathEndTValueConstraint(.9);
//            park.getPath(0).setPathEndTimeoutConstraint(10);
//            park.getPath(1).setPathEndTimeoutConstraint(10);
        }
        else if (autoLocation == AutoLocation.PEDRO_LEFT_SPECI_V1) {
            startPose = new Point(start_Speci_V1.getX(), start_Speci_V1.getY(), Point.CARTESIAN);

            double looseGrabP1 = .2;
            double looseGrabP2 = 1;
            double scoreX = 31.5;
            double initalScoreY = -40;
            double specSize = 1.5;

            specScore1 = new PathBuilder()
                    .addPath(new Path(
                            new BezierCurve(
                                    startPose,
                                    new Point(scoreX /*2.75*/, initalScoreY/*19.5*/, Point.CARTESIAN),
                                    new Point(scoreX /*2.75*/, initalScoreY/*19.5*/, Point.CARTESIAN)
                            )
                    ))
                    .addTemporalCallback(0, () -> {
                        currentMode = TeleopMode.SPECIMEN_SCORE;
                        arm.setTeleopMode(currentMode);
                        arm.setAnimationType(AnimationType.NONE);
                        arm.update(opMode.opModeIsActive());

//                        arm.setArmPositionSpecimen(100);
//                        arm.setSlidesPositionSpecimen(9);
//                        arm.update(opMode.opModeIsActive());
                    })
                    .addParametricCallback(looseGrabP1, () -> {
                        intake.looseGrab();
                    })
                    .addParametricCallback(looseGrabP2, () -> {
                        intake.intake();
                    })
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .setPathEndTimeoutConstraint(0)
                    .setPathEndVelocityConstraint(100)
                    .setZeroPowerAccelerationMultiplier(1)
                    .setPathEndTValueConstraint(.9);

            int sample1Y = -90;
            int sample2Y = -100;
            int sample3Y = -105;
            int sampleOffset = 10;
            int stopX = 22;
            int stopS1X = 22;
            int afterSampleX = 50;
            int midZPM = 10;
            double pushSamplesHeading = 180;
            int endZPM = 6;

            pushSamples = new PathBuilder()
                    .addPath(new Path(
                            new BezierCurve(
                                    specScore1.build().getPath(specScore1.build().size() - 1).getLastControlPoint(),
                                    new Point(0, -40, Point.CARTESIAN),
                                    new Point(10, -78, Point.CARTESIAN),
                                    new Point(afterSampleX - 5, sample1Y + sampleOffset, Point.CARTESIAN)
                            )
                    ))
                    .addTemporalCallback(.25, () -> {
                        currentMode = TeleopMode.INTAKE;
                        arm.setTeleopMode(currentMode);
                        arm.setAnimationType(AnimationType.NONE);
                        arm.intakeSpecimen = true;
                        arm.update(opMode.opModeIsActive());
                        intake.setWristAngle(WristAngle.DOWN);
                    })
                    .setZeroPowerAccelerationMultiplier(midZPM)
//                    .setTangentHeadingInterpolation()
                    .setLinearHeadingInterpolation(Math.toRadians(pushSamplesHeading), Math.toRadians(pushSamplesHeading), .1)
//                    .addPath(new Path(
//                            new BezierLine(
//                                    new Point(stopX, sample1Y + sampleOffset, Point.CARTESIAN),
//                                    new Point(35, sample1Y + sampleOffset, Point.CARTESIAN)
//                            )
//                    ))
//                    .setZeroPowerAccelerationMultiplier(midZPM)
//                    .setConstantHeadingInterpolation(Math.toRadians(pushSamplesHeading))
                    .addPath(new Path(
                            new BezierCurve(
                                    new Point(afterSampleX, sample1Y - sampleOffset, Point.CARTESIAN),
                                    new Point(afterSampleX, sample1Y, Point.CARTESIAN)
                            )
                    ))
                    .setZeroPowerAccelerationMultiplier(midZPM)
//                    .setPathEndVelocityConstraint(80)
//                    .setTangentHeadingInterpolation()
//                    .setLinearHeadingInterpolation(Math.toRadians(pushSamplesHeading), Math.toRadians(pushSamplesHeading), .25)
                    .setConstantHeadingInterpolation(Math.toRadians(pushSamplesHeading))
                    .addPath(new Path(
                            new BezierLine(
                                    new Point(afterSampleX, sample1Y, Point.CARTESIAN),
                                    new Point(stopS1X, sample1Y, Point.CARTESIAN)
                            )
                    ))
                    .setZeroPowerAccelerationMultiplier(2)
                    .setConstantHeadingInterpolation(Math.toRadians(pushSamplesHeading))
                    .addPath(new Path(
                            new BezierLine(
                                    new Point(stopS1X, sample1Y, Point.CARTESIAN),
                                    new Point(afterSampleX - 15, sample1Y, Point.CARTESIAN)
                            )
                    ))
                    .setPathEndVelocityConstraint(80)
                    .setConstantHeadingInterpolation(Math.toRadians(pushSamplesHeading))
                    .addPath(new Path(new BezierLine(
                            new Point(afterSampleX, sample2Y, Point.CARTESIAN),
                            new Point(stopX, sample2Y, Point.CARTESIAN)
                    )))
                    .setPathEndVelocityConstraint(80)
                    .setConstantHeadingInterpolation(Math.toRadians(pushSamplesHeading))
                    .addPath(new Path(new BezierCurve(
                            new Point(stopX, sample2Y, Point.CARTESIAN),
                            new Point(afterSampleX - 10, -100, Point.CARTESIAN),
                            new Point(afterSampleX - 5, sample3Y, Point.CARTESIAN)
                    )))
                    .setPathEndVelocityConstraint(80)
                    .setConstantHeadingInterpolation(Math.toRadians(pushSamplesHeading))
                    .addPath(new Path(new BezierLine(
                            new Point(afterSampleX - 5, sample3Y, Point.CARTESIAN),
                            new Point(15, sample3Y, Point.CARTESIAN)
                    )))
                    .addTemporalCallback(.25, () -> {
                        arm.setTeleopMode(TeleopMode.INTAKE);
                        arm.intakeSpecimen = true;
                        arm.update(opMode.opModeIsActive());

                        arm.intakeDownMode();
                        arm.update(opMode.opModeIsActive());
                    })
                    .setZeroPowerAccelerationMultiplier(1)
                    .setConstantHeadingInterpolation(Math.toRadians(pushSamplesHeading))
                    .setPathEndTValueConstraint(.9);

            double grabY = -75;
            int intakeTimeout = 0;
            double intakeEndVelo = 80;
            double intakeZPM = 1;
            double intakeHeading = 180;
            Point intakeEndPoint = new Point(15, grabY, Point.CARTESIAN);

            intakeSpec1 = new PathBuilder()
                    .addPath(new Path(
                            new BezierLine(
                                    pushSamples.build().getPath(pushSamples.build().size() - 1).getLastControlPoint(),
                                    new Point(15, -103, Point.CARTESIAN)
//                                    new Point(grabX /*2.75*/, grabY/*19.5*/, Point.CARTESIAN)
                            )
                    ))
                    .setPathEndVelocityConstraint(intakeEndVelo)
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(intakeHeading))
                    .setZeroPowerAccelerationMultiplier(intakeZPM)
                    .setPathEndTimeoutConstraint(intakeTimeout);

            specScore2 = new PathBuilder()
                    .addPath(new Path(
                            new BezierCurve(
                                    intakeSpec1.build().getPath(intakeSpec1.build().size() - 1).getLastControlPoint(),
                                    new Point(5 /*2.75*/, -60/*19.5*/, Point.CARTESIAN),
                                    new Point(scoreX /*2.75*/, initalScoreY-(specSize*1)/*19.5*/, Point.CARTESIAN)
                            )
                    ))
                    .addParametricCallback(looseGrabP1, () -> {
                        intake.looseGrab();
                    })
                    .addParametricCallback(looseGrabP2, () -> {
                        intake.intake();
                    })
                    .setLinearHeadingInterpolation(Math.toRadians(intakeHeading), Math.toRadians(180))
                    .setPathEndTimeoutConstraint(100)
                    .setPathEndTValueConstraint(.9);

            intakeSpec2 = new PathBuilder()
                    .addPath(new Path(
                            new BezierLine(
                                    specScore2.build().getPath(specScore2.build().size() - 1).getLastControlPoint(),
                                    intakeEndPoint
                            )
                    ))
                    .setPathEndVelocityConstraint(intakeEndVelo)
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(intakeHeading))
                    .setZeroPowerAccelerationMultiplier(intakeZPM)
                    .setPathEndTimeoutConstraint(intakeTimeout);
//                    .addPath(new Path(
//                            new BezierLine(
//                                    intakeEndPoint,
//                            )
//                    ))
//                    .setConstantHeadingInterpolation(Math.toRadians(180))
//                    .setPathEndTimeoutConstraint(0)
//                    .setZeroPowerAccelerationMultiplier(3)
//                    .setPathEndTValueConstraint(.9);

            specScore3 = new PathBuilder()
                    .addPath(new Path(
                            new BezierCurve(
                                    intakeSpec2.build().getPath(intakeSpec2.build().size() - 1).getLastControlPoint(),
                                    new Point(5 /*2.75*/, -50/*19.5*/, Point.CARTESIAN),
                                    new Point(scoreX /*2.75*/, initalScoreY-(specSize*2)/*19.5*/, Point.CARTESIAN)
                            )
                    ))
                    .addParametricCallback(looseGrabP1, () -> {
                        intake.looseGrab();
                    })
                    .addParametricCallback(looseGrabP2, () -> {
                        intake.intake();
                    })
                    .setLinearHeadingInterpolation(Math.toRadians(intakeHeading), Math.toRadians(180))
                    .setPathEndTimeoutConstraint(100)
                    .setPathEndTValueConstraint(.9);

            intakeSpec3 = new PathBuilder()
                    .addPath(new Path(
                            new BezierLine(
                                    specScore3.build().getPath(specScore3.build().size() - 1).getLastControlPoint(),
                                    new Point(scoreX /*2.75*/, -40/*19.5*/, Point.CARTESIAN)
                            )
                    ))
                    .addParametricCallback(.89, () -> {
                        intake.outtake();
                        intake.setWristAngle(WristAngle.SPECIMEN_INTAKE);

                        intake.setShortRange(true);
                        currentMode = TeleopMode.INTAKE;
                        arm.setTeleopMode(currentMode);

                        arm.update(opMode.opModeIsActive());
                        intake.update(opMode.opModeIsActive());
                    })
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .setPathEndTimeoutConstraint(0)
                    .addPath(new Path(
                            new BezierLine(
                                    new Point(scoreX /*2.75*/, -40/*19.5*/, Point.CARTESIAN),
                                    intakeEndPoint
                            )
                    ))
                    .addParametricCallback(0, () -> {
                        intake.setGrabAngle(GrabAngle.VERTICAL_GRAB);
                    })
                    .setPathEndVelocityConstraint(intakeEndVelo)
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(intakeHeading))
                    .setZeroPowerAccelerationMultiplier(intakeZPM)
                    .setPathEndTimeoutConstraint(intakeTimeout);

            specScore4 = new PathBuilder()
                    .addPath(new Path(
                            new BezierCurve(
                                    intakeSpec2.build().getPath(intakeSpec2.build().size() - 1).getLastControlPoint(),
                                    new Point(5 /*2.75*/, -60/*19.5*/, Point.CARTESIAN),
                                    new Point(scoreX /*2.75*/, initalScoreY-(specSize*3)/*19.5*/, Point.CARTESIAN)
                            )
                    ))
                    .addParametricCallback(looseGrabP1, () -> {
                        intake.looseGrab();
                    })
                    .addParametricCallback(looseGrabP2, () -> {
                        intake.intake();
                    })
                    .setLinearHeadingInterpolation(Math.toRadians(intakeHeading), Math.toRadians(180))
                    .setPathEndTimeoutConstraint(0)
                    .setPathEndTValueConstraint(.9);

            intakeSpec4 = new PathBuilder()
                    .addPath(new Path(
                            new BezierLine(
                                    specScore4.build().getPath(specScore4.build().size() - 1).getLastControlPoint(),
                                    intakeEndPoint
                            )
                    ))
                    .setPathEndVelocityConstraint(intakeEndVelo)
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(intakeHeading))
                    .setZeroPowerAccelerationMultiplier(intakeZPM)
                    .setPathEndTimeoutConstraint(intakeTimeout);
//                    .addPath(new Path(
//                            new BezierLine(
//                                    intakeEndPoint,
//                            )
//                    ))
//                    .setConstantHeadingInterpolation(Math.toRadians(180))
//                    .setPathEndTimeoutConstraint(0)
//                    .setZeroPowerAccelerationMultiplier(3)
//                    .setPathEndTValueConstraint(.9);

            specScore5 = new PathBuilder()
                    .addPath(new Path(
                            new BezierCurve(
                                    intakeSpec2.build().getPath(intakeSpec2.build().size() - 1).getLastControlPoint(),
                                    new Point(5 /*2.75*/, -50/*19.5*/, Point.CARTESIAN),
                                    new Point(scoreX /*2.75*/, initalScoreY-(specSize*4)/*19.5*/, Point.CARTESIAN)
                            )
                    ))
                    .addParametricCallback(looseGrabP1, () -> {
                        intake.looseGrab();
                    })
                    .addParametricCallback(looseGrabP2, () -> {
                        intake.intake();
                    })
                    .setLinearHeadingInterpolation(Math.toRadians(intakeHeading), Math.toRadians(180))
                    .setPathEndTimeoutConstraint(100)
                    .setPathEndTValueConstraint(.9);


            park = new PathBuilder()
                    .addPath(new Path(
                            new BezierCurve(
                                    specScore3.build().getPath(specScore3.build().size() - 1).getLastControlPoint(),
                                    new Point(5, -40, Point.CARTESIAN),
                                    new Point(0, -90, Point.CARTESIAN)
                            )
                    ))
                    .addParametricCallback(0, () -> {
                        currentMode = TeleopMode.IDLE;
                        arm.setTeleopMode(currentMode);
                        arm.setAnimationType(AnimationType.NONE);
                        arm.update(opMode.opModeIsActive());
                    })
                    .setConstantHeadingInterpolation(Math.toRadians(180))
//                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0), .1)
                    .setPathEndTimeoutConstraint(0)
                    .setPathEndTValueConstraint(.9);
        }
        else if (autoLocation == AutoLocation.PEDRO_LEFT_3_1_V2) {
            toBucketPath = new PathBuilder()
                    .addPath(new Path(
                            new BezierLine(
                                    new Point(0, 0, Point.CARTESIAN),
                                    new Point(15, 9, Point.CARTESIAN)
                            )
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(140));

//            toBucketPath.getPath(0).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(140));
            double startHeading = start_3_1_V1.getHeading();

            startPose = new Point(start_3_1_V1.getX(), start_3_1_V1.getY(), Point.CARTESIAN);

            specScore1 = new PathBuilder()
                    .addPath(new Path(
                            new BezierCurve(
                                    startPose,
                                    new Point(33 /*2.75*/, -45/*19.5*/, Point.CARTESIAN),
                                    new Point(33 /*2.75*/, -40/*19.5*/, Point.CARTESIAN)
                            )
                    ))
                    .addTemporalCallback(0, () -> {
                        currentMode = TeleopMode.SPECIMEN_SCORE;
                        arm.setTeleopMode(currentMode);
                        arm.update(opMode.opModeIsActive());

                        arm.setArmPositionSpecimen(100);
                        arm.setSlidesPositionSpecimen(8);
                    })
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .setPathEndTimeoutConstraint(250)
                    .setPathEndTValueConstraint(.9);

            specBackup = new PathBuilder()
                    .addPath(new Path(
                            new BezierCurve(
                                    specScore1.build().getPath(specScore1.build().size() - 1).getLastControlPoint(),
                                    new Point(0, 3, Point.CARTESIAN),
                                    new Point(15, 10, Point.CARTESIAN)
                            )
                    ))
                    .addParametricCallback(.35, () -> {
                        currentMode = TeleopMode.INTAKE;
                        arm.setTeleopMode(currentMode);
                        arm.setIntakePush(true);
                        arm.update(opMode.opModeIsActive());

                        intake.intake();
                        intake.update(opMode.opModeIsActive());
                    })
                    .addParametricCallback(.99, () -> {
//                            relocalizeX();
                    })
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .setPathEndTimeoutConstraint(100)
                    .setPathEndTValueConstraint(.95);

            Point bucketFinalPoint = new Point(2.75 /*2.75*/, 20/*19.5*/, Point.CARTESIAN);
//            bucketScorePathFromIntake1.getPath(0).setConstantHeadingInterpolation(Math.toRadians(140));
//            bucketScorePathFromIntake1.getPath(0).setPathEndTValueConstraint(.9);

//            backupPath = new PathBuilder()
//                    .addPath(new Path(
//                            new BezierLine(
//                                    bucketScorePathFromIntake1.build().getPath(0).getLastControlPoint(),
//                                    new Point(9, 11, Point.CARTESIAN)
//                            )
//                    ))
//                    .setConstantHeadingInterpolation(Math.toRadians(140));

//            backupPath.getPath(0).setConstantHeadingInterpolation(Math.toRadians(140));
//            backupPath.getPath(0).setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(0));


//            turnPath = new PathBuilder()
//                    .addPath(new Path(
//                            new BezierPoint(
//                                    backupPath.getPath(backupPath.size() - 1).getLastControlPoint()
//                            )
//                    )).build();
//
//            turnPath.getPath(0).setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(0));


            Point intakeYellow1Start = specBackup.build().getPath(0).getLastControlPoint();
            double intakeYellow1StartHeading = 0;

            pushSamples = new PathBuilder()
                    .addPath(new BezierLine(
                            specBackup.build().getPath(0).getLastControlPoint(),
                            new Point(26, 10, Point.CARTESIAN) //24.25 10
                    ))
                    .setConstantHeadingInterpolation(0)
                    .setPathEndTValueConstraint(.99)
                    .setPathEndTimeoutConstraint(0);


            intakeYellow1 = new PathBuilder()
                    .addPath(new Path(
                            new BezierCurve(
                                    new Point(15, 10, Point.CARTESIAN), //24.25 10
                                    new Point(5, 10, Point.CARTESIAN), //24.25 10
                                    new Point(29, 10, Point.CARTESIAN) //24.25 10
                            )
                    ))
                    .setPathEndTValueConstraint(.99)
                    .setPathEndTimeoutConstraint(100)
                    .setConstantHeadingInterpolation(0);

//            intakeYellow1.getPath(0).setConstantHeadingInterpolation(Math.toRadians(0));
//            intakeYellow1.getPath(0).setPathEndTValueConstraint(.99);
//            intakeYellow1.getPath(0).setPathEndTimeoutConstraint(500);

            intakeYellow2 = new PathBuilder()
                    .addPath(new Path(
                            new BezierLine(
                                    bucketFinalPoint,
                                    new Point(17, 20, Point.CARTESIAN)
                            )
                    ))
                    .addParametricCallback(.6, () -> {
                        currentMode = TeleopMode.INTAKE;
                        arm.setTeleopMode(currentMode);
                        arm.update(opMode.opModeIsActive());
                    })
                    .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(0))
                    .addPath(new Path(
                            new BezierLine(
                                    new Point(17, 20, Point.CARTESIAN),
                                    new Point(23.3, 20, Point.CARTESIAN)
                            )
                    ))
                    .addParametricCallback(.95, () -> {
                        setSpeed(1);
                    })
//                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .setPathEndTValueConstraint(.97)
                    .setPathEndTimeoutConstraint(100);

//            intakeYellow2.getPath(0).setConstantHeadingInterpolation(Math.toRadians(0));
//            intakeYellow2.getPath(0).setPathEndTValueConstraint(.99);
//            intakeYellow2.getPath(0).setPathEndTimeoutConstraint(500);

            intakeYellow3 = new PathBuilder()
                    .addPath(new Path(
                            new BezierLine(
                                    bucketFinalPoint,
                                    new Point(28.1, 12, Point.CARTESIAN)
                            )
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(130), .875, .25)
                    .addParametricCallback(.3, () -> {
                        currentMode = TeleopMode.INTAKE;
                        arm.setTeleopMode(currentMode);
                        arm.intakeUpMode();
                        arm.update(opMode.opModeIsActive());
                    })
                    .addParametricCallback(.9, () -> {
                        setSpeed(1);
                    })
                    .setPathEndTValueConstraint(.97)
                    .addPath(new Path(
                            new BezierLine(
                                    new Point(26.5, 12, Point.CARTESIAN),
                                    new Point(26.5, 20, Point.CARTESIAN)
                            )
                    ))
                    .setConstantHeadingInterpolation(.875)
//                    .setLinearHeadingInterpolation(Math.toRadians(0), .875)
                    .setPathEndTValueConstraint(.93)
                    .setPathEndTimeoutConstraint(100);

            bucketScorePathFromIntake1 = new PathBuilder()
                    .addPath(new Path(
                            new BezierLine(
                                    intakeYellow1.build().getPath(intakeYellow1.build().size() - 1).getLastControlPoint(),
                                    bucketFinalPoint

                            )
                    ))
                    .addParametricCallback(.95, () -> {
                        setSpeed(1);
                    })
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(130), .5)
//                    .setConstantHeadingInterpolation(Math.toRadians(130))
                    .setPathEndTValueConstraint(.87);

            bucketScorePathFromIntake2 = new PathBuilder()
                    .addPath(new Path(
                            new BezierLine(
                                    intakeYellow2.build().getPath(intakeYellow2.build().size() - 1).getLastControlPoint(),
                                    bucketFinalPoint

                            )
                    ))
                    .addParametricCallback(.95, () -> {
                        setSpeed(1);
                    })
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(130), .5)
//                    .setConstantHeadingInterpolation(Math.toRadians(130))
                    .setPathEndTValueConstraint(.87);

            bucketScorePathFromIntake3 = new PathBuilder()
                    .addPath(new Path(
                            new BezierLine(
                                    intakeYellow3.build().getPath(intakeYellow3.build().size() - 1).getLastControlPoint(),
                                    bucketFinalPoint

                            )
                    ))
                    .addParametricCallback(.95, () -> {
                        setSpeed(1);
                    })
                    .setLinearHeadingInterpolation(.875, Math.toRadians(130), .5)
//                    .setConstantHeadingInterpolation(Math.toRadians(130))
                    .setPathEndTValueConstraint(.87);

//            intakeYellow3.getPath(0).setLinearHeadingInterpolation(Math.toRadians(0), .875);
//            intakeYellow3.getPath(0).setPathEndTValueConstraint(.99);
//            intakeYellow3.getPath(0).setsetAniPathEndTimeoutConstraint(500);

            park = new PathBuilder()
                    .addPath(new Path(
                                    new BezierLine(
                                            bucketFinalPoint,
                                            new Point(54, 1, Point.CARTESIAN)
                                    )
                            )
                    )
                    .addParametricCallback(.4, () -> {
                        currentMode = TeleopMode.TOUCH_POLE_AUTO;
                        arm.setTeleopMode(currentMode);
                        arm.update(opMode.opModeIsActive());
                        arm.setParkArmUp(true);
                        safeSleep(1500);
                        arm.setParkArmUp(false);
                        safeSleep(10000);
                    })

                    .setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(-90))
                    .addPath(new Path(
                            new BezierLine(
                                    new Point(54, 1, Point.CARTESIAN),
                                    new Point(54, -12.5, Point.CARTESIAN)
                            )
                    )).setConstantHeadingInterpolation(Math.toRadians(-90))
                    .setPathEndTValueConstraint(.9)
                    .setPathEndTimeoutConstraint(100);

//            park.getPath(0).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-90));
//            park.getPath(1).setConstantHeadingInterpolation(Math.toRadians(-90));

//            park.getPath(0).setPathEndTValueConstraint(.9);
//            park.getPath(1).setPathEndTValueConstraint(.9);
//            park.getPath(0).setPathEndTimeoutConstraint(10);
//            park.getPath(1).setPathEndTimeoutConstraint(10);
        }
        else if (autoLocation == AutoLocation.PEDRO_LEFT_SPECI_V2) {
            startPose = new Point(start_Speci_V2.getX(), start_Speci_V2.getY(), Point.CARTESIAN);

            double looseGrabP1 = .2;
            double looseGrabP2 = 1;
            double scoreX = 36;
            double score1X = scoreX;
            double initalScoreY = -30;
            double specSize = 1.5;

            specScore1 = new PathBuilder()
                    .addPath(new Path(
                            new BezierLine(
                                    startPose,
//                                    new Point(score1X /*2.75*/, initalScoreY/*19.5*/, Point.CARTESIAN),
                                    new Point(score1X /*2.75*/, initalScoreY/*19.5*/, Point.CARTESIAN)
                            )
                    ))
                    .addTemporalCallback(0, () -> {
//                        currentMode = TeleopMode.SPECIMEN_SCORE;
//                        arm.setTeleopMode(currentMode);
//                        arm.setAnimationType(AnimationType.NONE);
//                        arm.update(opMode.opModeIsActive());
//                        arm.setArmPositionSpecimen(48);
//                        arm.setSlidesPositionSpecimen(10);
//                        arm.update(opMode.opModeIsActive());

//                        arm.setArmPositionSpecimen(100);
//                        arm.setSlidesPositionSpecimen(9);
//                        arm.update(opMode.opModeIsActive());
                    })
                    .addParametricCallback(looseGrabP1, () -> {
//                        intake.looseGrab();
//                        intake.intake();
                    })
                    .addParametricCallback(looseGrabP2, () -> {
//                        intake.intake();
                    })
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .setPathEndTimeoutConstraint(0)
                    .setPathEndVelocityConstraint(100)
                    .setZeroPowerAccelerationMultiplier(5)
                    .setPathEndTValueConstraint(.9);

            backupFromSub = new PathBuilder()
                    .addPath(new Path(
                            new BezierLine(
                                    new Point(score1X /*2.75*/, initalScoreY/*19.5*/, Point.CARTESIAN),
                                    new Point(6 /*2.75*/, initalScoreY/*19.5*/, Point.CARTESIAN)
                            )
                    ))
                    .addTemporalCallback(.25, () -> {
                        arm.setTeleopMode(TeleopMode.INTAKE);
                        arm.intakeUpMode();
                        arm.setAnimationType(AnimationType.NORMAL);
                        arm.setIntakePosition(params.INTAKE_MAX_POS);
                        arm.update(opMode.opModeIsActive());

                        intake.setWristAngle(WristAngle.DOWN);
                        intake.update(opMode.opModeIsActive());
                    })
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .setPathEndTimeoutConstraint(0)
//                    .setPathEndVelocityConstraint(50)
                    .setZeroPowerAccelerationMultiplier(5)
                    .setPathEndTValueConstraint(.9);

            double depositHeading = 45;

            depositToHP = new PathBuilder()
                    .addPath(new Path(
                            new BezierCurve(
                                    new Point(sampleGrabOffsetSpecimen.getX(), sampleGrabOffsetSpecimen.getY(), Point.CARTESIAN),
                                    new Point(25, -45, Point.CARTESIAN),
                                    new Point(2 /*2.75*/, -90/*19.5*/, Point.CARTESIAN)
                            )
                    ))
                    .addTemporalCallback(.6, () -> {
                        arm.setTeleopMode(TeleopMode.CUSTOM_POSITION);
                        arm.setArmCustomPosition(90);
                        arm.setSlidesCustomPosition(1.5);
                        arm.update(true);

                        intake.setWristAngle(WristAngle.BUCKET_SCORE);
                        intake.setGrabAngle(GrabAngle.INVERTED);
                    })
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(depositHeading), 1)
                    .setPathEndTimeoutConstraint(0)
//                    .setPathEndVelocityConstraint(50)
                    .setZeroPowerAccelerationMultiplier(5)
                    .setPathEndTValueConstraint(.9);


            int sample1Y = -87;
            int sample2Y = -95;
            int sample3Y = -103;
            int sampleOffset = 10;
            int stopX = 20;
            int stopS1X = stopX;
            int afterSampleX = 50;
            int midZPM = 3;
            double pushSamplesHeading = 0;
            double pushSamplesStartHeading = 0;
            int endZPM = 6;
            Path pushSamplesStartPath = null;

            if(!specimenGrabFromSub) {
                pushSamplesStartPath = new Path(
                        new BezierCurve(
                                specScore1.build().getPath(specScore1.build().size() - 1).getLastControlPoint(),
                                new Point(0, -40, Point.CARTESIAN),
                                new Point(10, -78, Point.CARTESIAN),
                                new Point(afterSampleX - 15, sample1Y + sampleOffset, Point.CARTESIAN)
                        )
                );

                pushSamplesStartHeading = 180;
            } else {
                pushSamplesStartPath = new Path(
                        new BezierCurve(
                                depositToHP.build().getPath(depositToHP.build().size() - 1).getLastControlPoint(),
                                new Point(10, -78, Point.CARTESIAN),
                                new Point(afterSampleX - 15, sample1Y + sampleOffset, Point.CARTESIAN)
                        )
                );

                pushSamplesStartHeading = depositHeading;
            }

            double s1X = 20; //22
            double s1Y = -67;

            double s2X = 20; //23
            double s2Y = -76;

            double s3X = 20;
            double s3Y = -86;


            newPushSample1P1 = new PathBuilder()
                    .addPath(new Path(
                            new BezierLine(
                                    depositToHP.build().getPath(depositToHP.build().size() - 1).getLastControlPoint(),
                                    new Point(s1X /*2.75*/, s1Y/*19.5*/, Point.CARTESIAN)
                            )
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(depositHeading), Math.toRadians(-65))
                    .setPathEndTimeoutConstraint(0)
//                    .setPathEndVelocityConstraint(50)
                    .setZeroPowerAccelerationMultiplier(3)
                    .setPathEndTValueConstraint(.9);
            newPushSample1P2 = new PathBuilder()
                    .addPath(new Path(
                            new BezierLine(
                                    new Point(s1X /*2.75*/, s1Y/*19.5*/, Point.CARTESIAN),
                                    new Point(6/*2.75*/, -66/*19.5*/, Point.CARTESIAN)
                            )
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(-65), Math.toRadians(-119))
                    .setPathEndTimeoutConstraint(0)
//                    .setPathEndVelocityConstraint(50)
                    .setZeroPowerAccelerationMultiplier(3)
                    .setPathEndTValueConstraint(.9);

            newPushSample2P1 = new PathBuilder()
                    .addPath(new Path(
                            new BezierLine(
                                    newPushSample1P2.build().getPath(newPushSample1P2.build().size() - 1).getLastControlPoint(),
                                    new Point(s2X /*2.75*/, s2Y/*19.5*/, Point.CARTESIAN)
                            )
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(-119), Math.toRadians(-65), .25)
                    .setPathEndTimeoutConstraint(0)
//                    .setPathEndVelocityConstraint(50)
                    .setZeroPowerAccelerationMultiplier(3)
                    .setPathEndTValueConstraint(.9);
            newPushSample2P2 = new PathBuilder()
                    .addPath(new Path(
                            new BezierLine(
                                    new Point(s2X /*2.75*/, s2Y/*19.5*/, Point.CARTESIAN),
                                    new Point(6/*2.75*/, s2Y/*19.5*/, Point.CARTESIAN)
                            )
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(-65), Math.toRadians(-119))
                    .setPathEndTimeoutConstraint(0)
//                    .setPathEndVelocityConstraint(50)
                    .setZeroPowerAccelerationMultiplier(3)
                    .setPathEndTValueConstraint(.9);

            newPushSample3P1 = new PathBuilder()
                    .addPath(new Path(
                            new BezierLine(
                                    newPushSample2P2.build().getPath(newPushSample2P2.build().size() - 1).getLastControlPoint(),
                                    new Point(s3X /*2.75*/, s3Y/*19.5*/, Point.CARTESIAN)
                            )
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(-119), Math.toRadians(-65), .25)
                    .setPathEndTimeoutConstraint(0)
//                    .setPathEndVelocityConstraint(50)
                    .setZeroPowerAccelerationMultiplier(3)
                    .setPathEndTValueConstraint(.9);
            newPushSample3P2 = new PathBuilder()
                    .addPath(new Path(
                            new BezierLine(
                                    new Point(s3X /*2.75*/, s3Y/*19.5*/, Point.CARTESIAN),
                                    new Point(6/*2.75*/, s3Y/*19.5*/, Point.CARTESIAN)
                            )
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(-65), Math.toRadians(-119))
                    .setPathEndTimeoutConstraint(0)
//                    .setPathEndVelocityConstraint(50)
                    .setZeroPowerAccelerationMultiplier(3)
                    .setPathEndTValueConstraint(.9);

            pushSamples = new PathBuilder()
                    .addPath(pushSamplesStartPath)
                    .setZeroPowerAccelerationMultiplier(midZPM)
                    .setLinearHeadingInterpolation(Math.toRadians(pushSamplesStartHeading), Math.toRadians(pushSamplesHeading))
                    .addPath(new Path(
                            new BezierCurve(
                                    new Point(afterSampleX, sample1Y - sampleOffset, Point.CARTESIAN),
                                    new Point(afterSampleX, sample1Y, Point.CARTESIAN)
                            )
                    ))
                    .setZeroPowerAccelerationMultiplier(midZPM)
                    .setConstantHeadingInterpolation(Math.toRadians(pushSamplesHeading))
                    .addPath(new Path(
                            new BezierLine(
                                    new Point(afterSampleX, sample1Y, Point.CARTESIAN),
                                    new Point(stopS1X, sample1Y, Point.CARTESIAN)
                            )
                    ))
                    .setZeroPowerAccelerationMultiplier(2)
                    .setConstantHeadingInterpolation(Math.toRadians(pushSamplesHeading))
                    .addPath(new Path(
                            new BezierLine(
                                    new Point(stopS1X, sample1Y, Point.CARTESIAN),
                                    new Point(afterSampleX - 15, sample1Y, Point.CARTESIAN)
                            )
                    ))
                    .setPathEndVelocityConstraint(80)
                    .setConstantHeadingInterpolation(Math.toRadians(pushSamplesHeading))
                    .addPath(new Path(new BezierLine(
                            new Point(afterSampleX, sample2Y, Point.CARTESIAN),
                            new Point(stopX, sample2Y, Point.CARTESIAN)
                    )))
                    .setPathEndVelocityConstraint(80)
                    .setConstantHeadingInterpolation(Math.toRadians(pushSamplesHeading))
                    .addPath(new Path(new BezierCurve(
                            new Point(stopX, sample2Y, Point.CARTESIAN),
                            new Point(afterSampleX - 10, -100, Point.CARTESIAN),
                            new Point(afterSampleX - 5, sample3Y, Point.CARTESIAN)
                    )))
                    .setPathEndVelocityConstraint(80)
                    .setConstantHeadingInterpolation(Math.toRadians(pushSamplesHeading))
                    .addPath(new Path(new BezierLine(
                            new Point(afterSampleX - 5, sample3Y, Point.CARTESIAN),
                            new Point(8, -103, Point.CARTESIAN)
                    )))
                    .setZeroPowerAccelerationMultiplier(3)
                    .setPathEndVelocityConstraint(2)
                    .setConstantHeadingInterpolation(Math.toRadians(pushSamplesHeading))
                    .setPathEndTValueConstraint(.9);

            double grabY = -60;
            double grabX = 0;
            int intakeTimeout = 0;
            double intakeEndVelo = 80;
            double intakeZPM = 15;
            double intakeEndT = .85;
            double scoreEndT = .85;
            double scoreEndV = 25;
            double intakeHeading = 0;
            double scoreHeading = 0;
            Point intakeEndPoint = new Point(grabX, grabY, Point.CARTESIAN);
            Point intake1StartPoint = newPushSample3P2.build().getPath(newPushSample3P2.build().size() - 1).getLastControlPoint();
            double intake1StartHeading = newPushSample3P2.build().getPath(newPushSample3P2.build().size() - 1).getHeadingGoal(1);

            intakeSpec1 = new PathBuilder()
                    .addPath(new Path(
                            new BezierCurve(
//                                    intake1StartPoint,
                                    new Point(8, -103, Point.CARTESIAN),
                                    new Point(6, -95, Point.CARTESIAN),
                                    new Point(1.5, -90, Point.CARTESIAN)
//                                    new Point(grabX /*2.75*/, grabY/*19.5*/, Point.CARTESIAN)
                            )
                    ))
//                    .setPathEndVelocityConstraint(intakeEndVelo)
                    .setConstantHeadingInterpolation(Math.toRadians(0));
//                    .setZeroPowerAccelerationMultiplier(intakeZPM)
//                    .setPathEndTimeoutConstraint(intakeTimeout);

            double intakeTangentLength = 10;
            double scoreY = -35;

            specScore2 = new PathBuilder()
                    .addPath(new Path(
                            new BezierCurve(
                                    intakeSpec1.build().getPath(intakeSpec1.build().size() - 1).getLastControlPoint(),
                                    new Point(7.5 /*2.75*/, -55/*19.5*/, Point.CARTESIAN),
                                    new Point(scoreX /*2.75*/, scoreY/*19.5*/, Point.CARTESIAN)
                            )
                    ))
//                    .setPathEndTValueConstraint(scoreEndT)
//                    .setPathEndVelocityConstraint(scoreEndV)
//                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(scoreHeading));
                    .setConstantHeadingInterpolation(Math.toRadians(0));

            intakeSpec2 = new PathBuilder()
                    .addPath(new Path(
                            new DoubleHeadingTangentPath(
                                    specScore2.build().getPath(specScore2.build().size() - 1).getLastControlPoint(),
                                    intakeEndPoint
                            )
                    ))
//                    .setPathEndTValueConstraint(intakeEndT)
//                    .setPathEndVelocityConstraint(intakeEndVelo)
//                    .setLinearHeadingInterpolation(Math.toRadians(intakeHeading), Math.toRadians(intakeHeading))
//                    .setZeroPowerAccelerationMultiplier(intakeZPM)
//                    .setPathEndTimeoutConstraint(intakeTimeout);
                    .setConstantHeadingInterpolation(Math.toRadians(0));

            specScore3 = new PathBuilder()
                    .addPath(new Path(
                            new DoubleHeadingTangentPath(
                                    intakeSpec2.build().getPath(intakeSpec2.build().size() - 1).getLastControlPoint(),
//                                    new Point(5 /*2.75*/, -50/*19.5*/, Point.CARTESIAN),
                                    new Point(scoreX /*2.75*/, scoreY/*19.5*/, Point.CARTESIAN)
                            )
                    ))
//                    .setPathEndTValueConstraint(scoreEndT)
//                    .setPathEndVelocityConstraint(scoreEndV)
//                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(scoreHeading));
                    .setConstantHeadingInterpolation(Math.toRadians(0));

            intakeSpec3 = new PathBuilder()
                    .addPath(new Path(
                            new DoubleHeadingTangentPath(
                                    new Point(scoreX /*2.75*/, -40/*19.5*/, Point.CARTESIAN),
                                    intakeEndPoint
                            )
                    ))
                    .addParametricCallback(0, () -> {
                        intake.setGrabAngle(GrabAngle.VERTICAL_GRAB);
                    })
//                    .setPathEndTValueConstraint(intakeEndT)
//                    .setPathEndVelocityConstraint(intakeEndVelo)
//                    .setLinearHeadingInterpolation(Math.toRadians(intakeHeading), Math.toRadians(intakeHeading))
//                    .setZeroPowerAccelerationMultiplier(intakeZPM)
//                    .setPathEndTimeoutConstraint(intakeTimeout);
                    .setConstantHeadingInterpolation(Math.toRadians(0));

            specScore4 = new PathBuilder()
                    .addPath(new Path(
                            new DoubleHeadingTangentPath(
                                    intakeSpec2.build().getPath(intakeSpec2.build().size() - 1).getLastControlPoint(),
//                                    new Point(5 /*2.75*/, -60/*19.5*/, Point.CARTESIAN),
                                    new Point(scoreX /*2.75*/, scoreY/*19.5*/, Point.CARTESIAN)
                            )
                    ))
//                    .setPathEndVelocityConstraint(scoreEndV)
//                    .setPathEndTValueConstraint(scoreEndT)
//                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(scoreHeading));
                    .setConstantHeadingInterpolation(Math.toRadians(0));

            intakeSpec4 = new PathBuilder()
                    .addPath(new Path(
                            new DoubleHeadingTangentPath(
                                    specScore4.build().getPath(specScore4.build().size() - 1).getLastControlPoint(),
                                    intakeEndPoint
                            )
                    ))
//                    .setPathEndTValueConstraint(intakeEndT)
//                    .setPathEndVelocityConstraint(intakeEndVelo)
//                    .setLinearHeadingInterpolation(Math.toRadians(intakeHeading), Math.toRadians(intakeHeading))
//                    .setZeroPowerAccelerationMultiplier(intakeZPM)
//                    .setPathEndTimeoutConstraint(intakeTimeout);
                    .setConstantHeadingInterpolation(Math.toRadians(0));

            specScore5 = new PathBuilder()
                    .addPath(new Path(
                            new DoubleHeadingTangentPath(
                                    intakeSpec2.build().getPath(intakeSpec2.build().size() - 1).getLastControlPoint(),
//                                    new Point(5 /*2.75*/, -50/*19.5*/, Point.CARTESIAN),
                                    new Point(scoreX /*2.75*/, scoreY/*19.5*/, Point.CARTESIAN)
                            )
                    ))
//                    .setPathEndVelocityConstraint(scoreEndV)
//                    .setPathEndTValueConstraint(scoreEndT)
//                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(scoreHeading));
                    .setConstantHeadingInterpolation(Math.toRadians(0));

            specScore6 = new PathBuilder()
                    .addPath(new Path(
                            new DoubleHeadingTangentPath(
                                    intakeSpec2.build().getPath(intakeSpec2.build().size() - 1).getLastControlPoint(),
//                                    new Point(5 /*2.75*/, -50/*19.5*/, Point.CARTESIAN),
                                    new Point(scoreX /*2.75*/, scoreY/*19.5*/, Point.CARTESIAN)
                            )
                    ))
//                    .setPathEndVelocityConstraint(scoreEndV)
//                    .setPathEndTValueConstraint(scoreEndT)
//                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(scoreHeading));
                    .setConstantHeadingInterpolation(Math.toRadians(0));

            park = new PathBuilder()
                    .addPath(new Path(
                            new BezierCurve(
                                    specScore3.build().getPath(specScore3.build().size() - 1).getLastControlPoint(),
                                    new Point(5, -40, Point.CARTESIAN),
                                    new Point(0, -90, Point.CARTESIAN)
                            )
                    ))
                    .addParametricCallback(0, () -> {
                        currentMode = TeleopMode.IDLE;
                        arm.setTeleopMode(currentMode);
                        arm.setAnimationType(AnimationType.NONE);
                        arm.update(opMode.opModeIsActive());
                    })
                    .setConstantHeadingInterpolation(Math.toRadians(180))
//                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0), .1)
                    .setPathEndTimeoutConstraint(0)
                    .setPathEndTValueConstraint(.9);
        }


    }

    public void safeSleep(int sleep) {
        ElapsedTime time = new ElapsedTime();
        time.reset();

        while (sleep >= time.time(TimeUnit.MILLISECONDS)) {
            update();

            if (opMode.isStopRequested()) break;
            if (!opMode.opModeIsActive()) break;
        }
    }

    public void waitForSlides() {
        double position = arm.getSlidesTargetPos();

        while (!(arm.getSlidesPosition() + this.params.SLIDES_ERROR_TOLERANCE_AUTO > position && arm.getSlidesPosition() - this.params.SLIDES_ERROR_TOLERANCE_AUTO < position)) {
            update();

            if (opMode.isStopRequested()) break;
            if (!opMode.opModeIsActive()) break;
        }
    }

    public void waitForSlides(double pos, double error) {
        double position = pos;

        if(arm.slidesDisabled) {
            safeSleep(500);
        } else {

            while (!(arm.getSlidesPosition() + error > position && arm.getSlidesPosition() - error < position)) {
                update();

                if (opMode.isStopRequested()) break;
                if (!opMode.opModeIsActive()) break;
            }
        }
    }

    public void waitForArm() {
        double targetPosition = arm.getArmTargetPosition();
        double position = arm.getArmPosition();

        while (!(position + this.params.ARM_ERROR_TOLERANCE_AUTO > targetPosition && position - this.params.ARM_ERROR_TOLERANCE_AUTO < targetPosition)) {
            update();

            if (opMode.isStopRequested()) break;
            if (!opMode.opModeIsActive()) break;
        }
    }

    public void waitUntilBelowError(double xErrorTarget, double yErrorTarget, int timeout) {
        ElapsedTime timer = new ElapsedTime();

        timer.reset();
        timer.startTime();

        while (timer.time(TimeUnit.MILLISECONDS) < timeout) {
            update();

            double xError = Math.abs(follower.getCurrentPath().getLastControlPoint().getX() - follower.getPose().getX());
            double yError = Math.abs(follower.getCurrentPath().getLastControlPoint().getY() - follower.getPose().getY());

            if(xError < xErrorTarget && yError < yErrorTarget) break;
        }
    }

    public void waitForArmAndSlides(int timeout) {
        ElapsedTime timer = new ElapsedTime();
        double armPos = arm.getArmPosition();
        double armTargetPos = arm.getArmTargetPosition();
        double slidesPos = arm.getSlidesPosition();
        double slidesTargetPos = arm.getSlidesTargetPos();
        timer.reset();
        timer.startTime();

        while (!( (armPos + this.params.ARM_ERROR_TOLERANCE_AUTO > armTargetPos && armPos - this.params.ARM_ERROR_TOLERANCE_AUTO < armTargetPos) && (slidesPos + this.params.SLIDES_ERROR_TOLERANCE_AUTO > slidesTargetPos && slidesPos - this.params.SLIDES_ERROR_TOLERANCE_AUTO < slidesTargetPos))) {
            update();

            if (opMode.isStopRequested()) break;
            if (!opMode.opModeIsActive()) break;
            if (timer.time(TimeUnit.MILLISECONDS) >= timeout) break;
        }
    }

    public void useDistance(boolean set) {
        useDistanceReloc = set;
    }

    public Pose getOldPose() {
        return oldPose;
    }

    private void relocalizeX(double sensorOffset) {
        double newX = robot.distanceOne.getDistance(DistanceUnit.INCH) - sensorOffset;
        double delta = newX - follower.getPose().getX();

        oldPose = new Pose(follower.getPose().getX() + delta, follower.getPose().getY(), follower.getPose().getHeading());

        follower.setPose(new Pose(newX, follower.getPose().getY(), follower.getPose().getHeading()));
    }

    public void setSlowDown(double slowdownT, double slowdownSpeed) {
    }

    public void waitUntilStop(double velo, double timeout) {
        ElapsedTime timer = new ElapsedTime();

        timer.reset();
        timer.startTime();
        boolean reachedVelo = false;

        while (timer.time(TimeUnit.MILLISECONDS) < timeout) {
            update();

            double currentVelo = follower.getVelocityMagnitude();

            telemetry.addData("velo: ", currentVelo);
            telemetry.update();

            if(reachedVelo && currentVelo < velo) break;
            if(!reachedVelo && currentVelo - .5 >= velo) reachedVelo = true;
        }
    }
}