package org.firstinspires.ftc.teamcode.AutoPedro;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
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
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

import java.util.concurrent.TimeUnit;

@Config
public class AutoManagerPedro {
    public PathBuilder toBucketPath;
    //    public PathBuilder backupPath;
    public PathBuilder turnPath;
    public PathBuilder bucketScorePathFromIntake1;
    public PathBuilder bucketScorePathFromIntake2;
    public PathBuilder bucketScorePathFromIntake3;
    public PathBuilder bucketScorePathS1;
    public PathBuilder park;
    public PathBuilder intakeYellow1;
    public PathBuilder intakeYellow2;
    public PathBuilder intakeYellow3;
    public PathBuilder specScore1;
    public PathBuilder specScore2;
    public PathBuilder specScore3;
    public PathBuilder specScore4;
    public PathBuilder specScore5;
    public PathBuilder intakeSpec1;
    public PathBuilder pushSamples;
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
    public Pose start_4_0_V1 = new Pose(.399, -.48, Math.toRadians(90));
    public Pose start_3_1_V1 = new Pose(0, -23.6, Math.toRadians(0));
    public Pose start_0_4_V1 = new Pose(4.3, -49.9, Math.toRadians(180));
    private Point leftScoreLocation = new Point(19, 109, Point.CARTESIAN);
    private Runnable updateAction;
    private MultipleTelemetry telemetry;
    private Params params;
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

        this.follower.setSlowDownVoltage(8, .4);
    }

    private void run(PathChain path, boolean correct) {
        if (!pathingDisabled) follower.followPath(path, correct);

        while (follower.isBusy()) {
            try {
                update();

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

        follower.update();
        updateAction.run();

//        telemetry.addData("get closest T: ", follower.getCurrentTValue());
//        telemetry.addData("get path type: ", follower.getCurrentPath().pathType());
////            telemetry.update();
        try {
            follower.telemetryDebug(telemetry);
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
        if (autoLocation == AutoLocation.PEDRO_LEFT_4_0_V1 || autoLocation == AutoLocation.PEDRO_LEFT_3_1_V1) {
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
            } else if (autoLocation == AutoLocation.PEDRO_LEFT_4_0_V1) {
                startPose = new Point(start_4_0_V1.getX(), start_4_0_V1.getY(), Point.CARTESIAN);
                startHeading = start_4_0_V1.getHeading();
            }

            Point bucketFinalPoint = new Point(3 /*2.75*/, 19.5/*19.5*/, Point.CARTESIAN);

            bucketScorePathS1 = new PathBuilder()
                    .addPath(new Path(
                            new BezierLine(
                                    startPose,
                                    new Point(3 /*2.75*/, 20/*19.5*/, Point.CARTESIAN)
                            )
                    ))
                    .setLinearHeadingInterpolation(startHeading, Math.toRadians(130))
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
            double intakeYellow1StartHeading = 130;
            double xOffset = 0;

            if (autoLocation == AutoLocation.PEDRO_LEFT_3_1_V1) {
                intakeYellow1Start = specBackup.build().getPath(0).getLastControlPoint();
                intakeYellow1StartHeading = 0;
                xOffset = 0;
            }

            double intakeZPM = 3;


            intakeYellow1 = new PathBuilder()
                    .addPath(new Path(
                            new BezierLine(
                                    intakeYellow1Start,
                                    new Point(19 - xOffset, 9, Point.CARTESIAN) //24.25 10
                            )
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(intakeYellow1StartHeading), Math.toRadians(0), .5)
                    .addParametricCallback(.6, () -> {
                        currentMode = TeleopMode.INTAKE;
                        arm.setTeleopMode(currentMode);
                        if(autoLocation == AutoLocation.PEDRO_LEFT_4_0_V1) {
                            arm.setAnimationType(AnimationType.FAST);
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
                                    new Point(17 - xOffset, 19, Point.CARTESIAN)
                            )
                    ))
                    .addParametricCallback(.6, () -> {
                        currentMode = TeleopMode.INTAKE;
                        arm.setTeleopMode(currentMode);
                        if(autoLocation == AutoLocation.PEDRO_LEFT_4_0_V1) {
                            arm.setAnimationType(AnimationType.FAST);
                        }
                        arm.update(opMode.opModeIsActive());
                    })
                    .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(0))
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
                                    new Point(18 - xOffset, 6, Point.CARTESIAN) //28.1
                            )
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(130), .875, .75)
                    .addParametricCallback(.3, () -> {
                        intake.setGrabAngle(GrabAngle.CUSTOM);
                        intake.setCustomPivotAngle(135);
                        intake.update(opMode.opModeIsActive());

                        currentMode = TeleopMode.INTAKE;
                        arm.setTeleopMode(currentMode);
                        if(autoLocation == AutoLocation.PEDRO_LEFT_4_0_V1) {
                            arm.setAnimationType(AnimationType.FAST);
                        }
                        arm.intakeUpMode();
                        arm.update(opMode.opModeIsActive());
                    })
                    .addParametricCallback(.9, () -> {
                        setSpeed(1);
                    })
                    .setPathEndTValueConstraint(.97)
                    .addPath(new Path(
                            new BezierLine(
                                    new Point(23, 12, Point.CARTESIAN),
                                    new Point(23, 17.5, Point.CARTESIAN)
                            )
                    ))
                    .setConstantHeadingInterpolation(.875)
//                    .setLinearHeadingInterpolation(Math.toRadians(0), .875)
                    .setZeroPowerAccelerationMultiplier(intakeZPM)
                    .setPathEndTValueConstraint(.9)
                    .setPathEndTimeoutConstraint(0);


            double bucketScoreZPM = 3.5;


            bucketScorePathFromIntake1 = new PathBuilder()
                    .addPath(new Path(
                            new BezierCurve(
                                    intakeYellow1.build().getPath(intakeYellow1.build().size() - 1).getLastControlPoint(),
                                    new Point(15, 0, Point.CARTESIAN),
                                    bucketFinalPoint
                            )
                    ))
                    .addParametricCallback(.95, () -> {
                        setSpeed(1);
                    })
                    .setZeroPowerAccelerationMultiplier(bucketScoreZPM)
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(130), .75)
//                    .setConstantHeadingInterpolation(Math.toRadians(130))
                    .setPathEndTValueConstraint(.87);

            bucketScorePathFromIntake2 = new PathBuilder()
                    .addPath(new Path(
                            new BezierCurve(
                                    intakeYellow2.build().getPath(intakeYellow2.build().size() - 1).getLastControlPoint(),
                                    new Point(15, 5, Point.CARTESIAN),
                                    bucketFinalPoint

                            )
                    ))
                    .addParametricCallback(.95, () -> {
                        setSpeed(1);
                    })
                    .setZeroPowerAccelerationMultiplier(bucketScoreZPM)
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(130), .75)
//                    .setConstantHeadingInterpolation(Math.toRadians(130))
                    .setPathEndTValueConstraint(.87);

            bucketScorePathFromIntake3 = new PathBuilder()
                    .addPath(new Path(
                            new BezierCurve(
                                    intakeYellow3.build().getPath(intakeYellow3.build().size() - 1).getLastControlPoint(),
                                    new Point(15, 5, Point.CARTESIAN),
                                    bucketFinalPoint
                            )
                    ))
                    .addParametricCallback(.95, () -> {
                        setSpeed(1);
                    })
                    .setZeroPowerAccelerationMultiplier(bucketScoreZPM)
                    .setLinearHeadingInterpolation(.875, Math.toRadians(130), .75)
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
        } else if (autoLocation == AutoLocation.PEDRO_LEFT_0_4_V1) {
            startPose = new Point(start_0_4_V1.getX(), start_0_4_V1.getY(), Point.CARTESIAN);

            double looseGrabP1 = .2;
            double looseGrabP2 = 1;
            double scoreX = 32.5;
            double initalScoreY = -40;

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
                    .setZeroPowerAccelerationMultiplier(1.5)
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

            double grabY = -80;
            double grabX = 7;
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
                                    new Point(scoreX /*2.75*/, initalScoreY-(5*1)/*19.5*/, Point.CARTESIAN)
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
                                    new Point(scoreX /*2.75*/, initalScoreY-(5*2)/*19.5*/, Point.CARTESIAN)
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
                                    new Point(scoreX /*2.75*/, initalScoreY-(5*1)/*19.5*/, Point.CARTESIAN)
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
                                    new Point(scoreX /*2.75*/, initalScoreY-(5*2.5)/*19.5*/, Point.CARTESIAN)
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
        } else if (autoLocation == AutoLocation.PEDRO_LEFT_3_1_V2) {
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
}