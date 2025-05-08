package org.firstinspires.ftc.teamcode.AutoPedro;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Enums.AutoLocation;
import org.firstinspires.ftc.teamcode.Enums.GrabAngle;
import org.firstinspires.ftc.teamcode.Enums.GrabStyle;
import org.firstinspires.ftc.teamcode.Enums.TeleopMode;
import org.firstinspires.ftc.teamcode.Enums.WristAngle;
import org.firstinspires.ftc.teamcode.Hardware.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Hardware.Params;
import org.firstinspires.ftc.teamcode.bedroBathing.follower.Follower;
import org.firstinspires.ftc.teamcode.bedroBathing.localization.Pose;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.Point;

import java.util.HashSet;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "Sample V1 Pedro", group = "0", preselectTeleOp = "0: Main TeleOp")
//@Disabled
public class V1_Sample_Pedro extends LinearOpMode {
    private Follower follower;
    private ElapsedTime autoTime = new ElapsedTime();
    private int autoState = 1;
    private HWProfile robot;
    private Params params;
    private ArmSubsystem arm;
    private IntakeSubsystem intake;
    private TeleopMode currentMode = TeleopMode.CUSTOM_POSITION;
    private int armWaitSleep = 750;
    private int outtakeSleep = 600;
    private int waitForArmIntakeDown = 500;
    private int waitToIntake = 1250;
    private ElapsedTime time = new ElapsedTime();
    private int waitForGrab = 150;
    private int waitForHalt = 500;
    private int intakeSleep = 1000;
    private double s1Xerror = 0;
    private double s1Yerror = 0;
    private double s2Xerror = 0;
    private boolean g1ACooldown = false;
    private boolean g1BCooldown = false;
    private double s2Yerror = 0;
    private double s3Xerror = 0;
    private double s3Yerror = 0;
    private boolean autoStart = true;
    private boolean sampleFromHP = false;
    private int samplesFromSub = 0;
    private boolean selectingSubSamples = false;
    private GrabAngle grabAngle = GrabAngle.VERTICAL_GRAB;
    private GrabStyle grabStyle = GrabStyle.OUTSIDE_GRAB;
    private WristAngle wristAngle = WristAngle.SPECIMEN_SCORE_1;
    private boolean opModeRunning = true;
    private AutoManagerPedro autoManager;
    private Thread armHandlerThread = new Thread(() -> {
        while (opModeIsActive()) {
//            arm.update(opModeIsActive());
//            intake.update(opModeIsActive());
//            autoManager.update();

            telemetry.addData("X:", follower.getPose().getX());
            telemetry.addData("Y:", follower.getPose().getY());
            telemetry.addData("heading:", Math.toDegrees(follower.getTotalHeading()));
            telemetry.addData("autoManager state:", autoState);
            telemetry.update();
        }
    });
    private final boolean debug = false;
    private boolean pathStarted = true;
    private PathChain currentPath;
    private int lastAutoState = 0;
    private AutoLocation autoLocation = AutoLocation.PEDRO_LEFT_SAMPLE_V1;
    private int nextAutoState = 0;
    private final String autoName = "Left Intake Autonomous: PEDRO PATHING";
    private Telemetry telemetryA;
    private AutoSampleIntakeLib intakeLib;

    public String getAutoName() {
        return autoName;
    }

    public V1_Sample_Pedro() {
    }

    public void runOpMode() {
        follower = new Follower(hardwareMap);
//        follower.resetIMU();
        telemetry.update();

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new HWProfile();
        robot.init(hardwareMap, false, false);
        params = new Params();
        arm = new ArmSubsystem(robot, this, params);
        intake = new IntakeSubsystem(robot, this, params);

        currentMode = TeleopMode.IDLE;
        arm.setTeleopMode(currentMode);
        arm.setAutoMode(true);
        arm.setPedroAuto(true);
        arm.resetSlidesPosition();

        intake.setWristAngle(wristAngle);
        intake.setGrabAngle(grabAngle);
        intake.setGrabStyle(grabStyle);
        intake.setPedroAuto(true);
        intake.intake();

        intake.update(opModeIsActive());
        arm.update(opModeIsActive());

        intakeLib = new AutoSampleIntakeLib(telemetry, gamepad1);

        autoManager = new AutoManagerPedro(this, follower, () -> {
            arm.update(opModeIsActive());
            intake.update(opModeIsActive());

//            telemetryA.addData("limelight fps: ", robot.limelight.getStatus().getFps());
            telemetryA.addData("s1 x: ", s1Xerror);
            telemetryA.addData("s1 y: ", s1Yerror);
            telemetryA.addData("s2 x: ", s2Xerror);
            telemetryA.addData("s2 y: ", s2Yerror);
            telemetryA.addData("s3 x: ", s3Xerror);
            telemetryA.addData("s3 y: ", s3Yerror);
        }, arm, intake, robot);
        autoManager.buildPaths(autoLocation);
        follower.setPose(autoManager.start_4_0_V1);

        currentMode = TeleopMode.CUSTOM_POSITION;
        arm.setTeleopMode(currentMode);
        intake.intake();

        autoManager.safeSleep(350);

        arm.setSlidesMultiplier(0);

        while (!opModeIsActive()) {
            arm.update(opModeInInit());
//            intake.update(opModeIsInit());
            samplesFromSub = intakeLib.getLocations().size();
            telemetry.addData("Amount of samples to intake from the sub: ", samplesFromSub);
            this.telemetry.addLine();

            if (!selectingSubSamples) {
                telemetry.addLine("Press Gamepad1 A to enable the fifth sample");
                telemetry.addLine("Press Gamepad1 B to switch to selecting sub-samples");
                telemetry.addData("Run fifth sample: ", sampleFromHP);
                telemetry.addLine();
                telemetry.addLine("The robot must be started on the third tile");
                telemetry.addLine("to the right from the buckets, the robot must be");
                telemetry.addLine("started on the left line of the tile, and the robot");
                telemetry.addLine("needs to be facing towards the submersible");
                telemetry.addLine();
                follower.setPose(autoManager.start_4_0_V1);
                telemetry.addData("slides raw enc:", robot.slidesMotor1.getCurrentPosition());
                telemetry.addData("x: ", follower.getPose().getX());
                telemetry.addData("y: ", follower.getPose().getY());

                if (gamepad1.a && !g1ACooldown) {
                    g1ACooldown = true;

                    sampleFromHP = !sampleFromHP;
                }

                if (!gamepad1.a) g1ACooldown = false;
            } else {
                telemetry.addLine("Press B again to return back.");
                this.telemetry.addLine();

                intakeLib.displayInfo();
                intakeLib.update();
            }

            if (gamepad1.b && !g1BCooldown) {
                g1BCooldown = true;

                selectingSubSamples = !selectingSubSamples;
            }

            if (!gamepad1.b) g1BCooldown = false;

            arm.setArmCustomPosition(params.ARM_INSPECTION_POS_AUTO);
            arm.setSlidesCustomPosition(0);

//            LLResult result = robot.limelight.getLatestResult();

//            if (result != null) {
//                telemetry.addData("detectX: ", result.getPythonOutput()[0]);
//                telemetry.addData("detectY: ", result.getPythonOutput()[1]);
//            }
            telemetry.update();
        }

        samplesFromSub = intakeLib.getLocations().size();

        follower.setPose(autoManager.start_3_1_V1);

        waitForStart();

        arm.setSlidesMultiplier(1);

//        limelightHandler.start();

        time.reset();
        arm.setArmPower(1);

        autoManager.setSpeed(params.AUTO_DEFAULT_SPEED);
        follower.updatePose();
        follower.update();
        autoManager.safeSleep(100);
        follower.setPose(autoManager.start_4_0_V1);
        autoManager.safeSleep(100);
        params.AUTO_SCORE = 35;
        if (!sampleFromHP) {
//            Params.AUTO_END_HEADING = -90;
        } else {
//            Params.AUTO_END_HEADING = -45;
        }
//        autoManager.runPath(autoManager.toBucketPath, false);

//        currentMode = TeleopMode.BUCKET_SCORE;
//        arm.setTeleopMode(currentMode);
//        arm.setBucket(2);
//        autoManager.safeSleep(750);

        bucketScore(1);
        if (sampleFromHP) {
            intake(4);
            bucketScore(5);
        }
        intake(1);
        bucketScore(2);
        intake(2);
        bucketScore(3);
        intake(3);
        bucketScore(4);
        if (samplesFromSub > 0) {
            HashSet<Pose> locations = intakeLib.getLocations();

            for (int i = 0; i < locations.size(); i++) {
                Pose location = (Pose) locations.toArray()[i];

                intakeFromSub(location);
                bucketScoreFromSub();
            }
        } else {
            if (!sampleFromHP) {
                park();
            }
        }

        follower.holdPoint(follower.getPose());
//        Params.AUTO_END_HEADING = follower.getPose().getHeading();

        params.TELEOP_START_MODE = TeleopMode.TOUCH_POLE_AUTO;
        params.AUTO_SCORE = 32;

        telemetryA.addData("time: ", time.time(TimeUnit.SECONDS));
        telemetryA.update();
        telemetryA.addData("s1 x: ", s1Xerror);
        telemetryA.addData("s1 y: ", s1Yerror);
        telemetryA.addData("s2 x: ", s2Xerror);
        telemetryA.addData("s2 y: ", s2Yerror);
        telemetryA.addData("s3 x: ", s3Xerror);
        telemetryA.addData("s3 y: ", s3Yerror);
        autoManager.safeSleep(10000);
    }

    public void park() {
        autoManager.setSpeed(1);
//        arm.setArmPower(.6);
        params.TELEOP_START_MODE = TeleopMode.TOUCH_POLE_AUTO;
        autoManager.runPath(autoManager.park, false);
//        arm.update(opModeIsActive());
//        autoManager.safeSleep(750);
//        arm.setParkArmUp(false);
//        autoManager.safeSleep(10000);
    }

    public void intakeFromSub(Pose sampleLocation) {
        Pose sampleGrabOffset = new Pose(39.6, 5.5);

        // setup arm
        arm.intakeUpMode();
        arm.setAutoMode(true);
        arm.setPedroAuto(true);
        arm.setSlidesMultiplier(1);
        arm.setSlidesPowerLimit(1);

        intake.outtake();
        intake.setWristAngle(WristAngle.DOWN);
        intake.setGrabAngle(GrabAngle.CUSTOM);
        intake.setCustomPivotAngle(Math.toDegrees(sampleLocation.getHeading()));

        arm.update(opModeIsActive());
        intake.update(opModeIsActive());

        // calc grab location
        Point grabPoint = new Point(sampleGrabOffset.getX() + sampleLocation.getY(), sampleGrabOffset.getY() - sampleLocation.getX(), Point.CARTESIAN);

        PathBuilder path = new PathBuilder()
                .addPath(new Path(
                        new BezierCurve(
                                new Point(follower.getPose().getX(), follower.getPose().getY()),
                                new Point(50, 15, Point.CARTESIAN),
                                grabPoint
                        )
                ))
                .addTemporalCallback(.1, () -> {
                    arm.setTeleopMode(TeleopMode.INTAKE);
                    arm.update(opModeIsActive());
                })
                .addParametricCallback(.6, () -> {
                    arm.setIntakePosition(Params.INTAKE_MAX_POS); //jank
                })
                .setZeroPowerAccelerationMultiplier(1)
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), Math.toRadians(-90));

        // run path
        autoManager.runPath(path, true);
        autoManager.safeSleep(750);

        // intake
        arm.intakeDownMode();
        autoManager.safeSleep(250);
        intake.intake();

        autoManager.safeSleep(250);

        // get ready to score
        arm.intakeUpMode();
        arm.setIntakePosition(0);
        intake.setWristAngle(WristAngle.BUCKET_SCORE);
        intake.setGrabAngle(GrabAngle.INVERTED);
        arm.update(opModeIsActive());
        intake.update(opModeIsActive());

        autoManager.safeSleep(500);

        autoManager.setSpeed(.75);
        autoManager.buildPaths(AutoLocation.PEDRO_LEFT_SAMPLE_V1); // pedro bug, workaround
    }

    public void intake(int sampleNum) {
        arm.setArmPower(1);

        arm.setIntakePosition(0);
        arm.intakeDownMode();
        intake.setGrabAngle(GrabAngle.CUSTOM);
        intake.setWristAngle(WristAngle.CUSTOM);
        intake.setCustomPivotAngle(-100);
        intake.setCustomWristAngle(40);
        intake.setShortRange(true);
        intake.update(opModeIsActive());


        autoManager.setSpeed(params.AUTO_DEFAULT_SPEED);
        if (sampleNum == 1) {
            autoManager.setSpeed(1);
            autoManager.runPath(autoManager.intakeYellow1, true);
            autoManager.safeSleep(1200);
        } else if (sampleNum == 2) {
            autoManager.setSpeed(1);
            autoManager.runPath(autoManager.intakeYellow2, true);
            autoManager.safeSleep(1200);
        } else if (sampleNum == 3) {
            autoManager.setSpeed(1);
            arm.setAutoLastSample(true);
            arm.setArmPower(1);
            autoManager.runPath(autoManager.intakeYellow3, true);
            autoManager.safeSleep(1200);
//            autoManager.safeSleep(2500);
        } else if (sampleNum == 4) {
            autoManager.setSpeed(1);
            autoManager.runPath(autoManager.intakeYellow4, true);
            autoManager.safeSleep(300);
        }


        autoManager.setSpeed(1);

        if (sampleNum != 3) {
            arm.setIntakePosition(10);
            autoManager.safeSleep(500);
        } else {
            arm.setIntakePosition(7);
            autoManager.safeSleep(350);
        }

        intake.intake();
        autoManager.safeSleep(150);

//        s1Xerror = Math.abs(follower.getPose().getX() - xTarget);
//        s1Yerror = endPose.getY();
//
        autoManager.setSpeed(params.AUTO_DEFAULT_SPEED);

        arm.setArmPower(1);
        currentMode = TeleopMode.BUCKET_SCORE;
        arm.setTeleopMode(currentMode);
//        if(sampleNum != 3) arm.setAnimationType(AnimationType.FAST);
        arm.setBucket(2);
        arm.update(opModeIsActive());
        autoManager.safeSleep(1350); //1250
    }

    public void bucketScore(int sampleNum) {
        intake.setWristAngle(WristAngle.BUCKET_SCORE);
        if (sampleNum == 1) {
            intake.setGrabAngle(GrabAngle.VERTICAL_GRAB);
        } else {
            intake.setGrabAngle(GrabAngle.INVERTED);
        }
        intake.update(opModeIsActive());

        if (arm.getTeleopMode() != TeleopMode.BUCKET_SCORE) {
            currentMode = TeleopMode.BUCKET_SCORE;
            arm.setTeleopMode(currentMode);
            arm.setBucket(2);
            arm.update(opModeIsActive());
            if (sampleNum == 1) {
                autoManager.safeSleep(1250);
            } else {
                autoManager.safeSleep(100);
            }
        }

        autoManager.setSpeed(1);
        if (sampleNum == 1) {
            autoManager.runPath(autoManager.bucketScorePathS1, true);
        } else if (sampleNum == 2) {
            autoManager.runPath(autoManager.bucketScorePathFromIntake1, true);
        } else if (sampleNum == 3) {
            autoManager.runPath(autoManager.bucketScorePathFromIntake2, true);
        } else if (sampleNum == 4) {
            autoManager.runPath(autoManager.bucketScorePathFromIntake3, true);
        } else if (sampleNum == 5) {
            autoManager.runPath(autoManager.bucketScorePathFromIntake4, true);
        }

        autoManager.setSpeed(params.AUTO_DEFAULT_SPEED);

        autoManager.waitUntilBelowError(.25, .25, 200);
//        arm.setArmTipBucketScore(true);
//        autoManager.safeSleep(300);
        intake.outtake();
//        arm.setArmTipBucketScore(false);
        autoManager.safeSleep(700);

        if (sampleNum != 4) {
//            autoManager.setSpeed(.6);
//            autoManager.runPath(autoManager.backupPath, false);
//            currentMode = TeleopMode.DOWN;
//            arm.setTeleopMode(currentMode);
//            arm.setIntakePosition(params.PEDRO_AUTO_INTAKE_Y1_POS);
//            arm.intakeUpMode();
//            arm.update(opModeIsActive());
//            autoManager.safeSleep(250);
//            autoManager.setSpeed(1);
        }
    }

    public void bucketScoreFromSub() {
        autoManager.runPath(autoManager.bucketScoreFromSubPath);
        autoManager.safeSleep(750);
        intake.outtake();
        autoManager.safeSleep(500);
    }
}
