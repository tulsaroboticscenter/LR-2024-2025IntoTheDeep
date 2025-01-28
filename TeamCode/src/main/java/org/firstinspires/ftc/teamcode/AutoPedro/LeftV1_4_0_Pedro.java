//package org.firstinspires.ftc.teamcode.AutoPedro;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.Enums.AnimationType;
//import org.firstinspires.ftc.teamcode.Enums.AutoLocation;
//import org.firstinspires.ftc.teamcode.Enums.GrabAngle;
//import org.firstinspires.ftc.teamcode.Enums.GrabStyle;
//import org.firstinspires.ftc.teamcode.Enums.TeleopMode;
//import org.firstinspires.ftc.teamcode.Enums.WristAngle;
//import org.firstinspires.ftc.teamcode.Hardware.ArmSubsystem;
//import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
//import org.firstinspires.ftc.teamcode.Hardware.IntakeSubsystem;
//import org.firstinspires.ftc.teamcode.Hardware.Params;
//import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
//import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
//
//import java.util.concurrent.TimeUnit;
//
//@Autonomous(name = "Left 4+0 V1 Pedro", group = "0", preselectTeleOp = "0: Main TeleOp")
//@Disabled
//public class LeftV1_4_0_Pedro extends LinearOpMode {
//    private Follower follower;
//    private ElapsedTime autoTime = new ElapsedTime();
//    private int autoState = 1;
//    private HWProfile robot;
//    private Params params;
//    private ArmSubsystem arm;
//    private IntakeSubsystem intake;
//    private TeleopMode currentMode = TeleopMode.IDLE;
//    private int armWaitSleep = 750;
//    private int outtakeSleep = 600;
//    private int waitForArmIntakeDown = 500;
//    private int waitToIntake = 1250;
//    private ElapsedTime time = new ElapsedTime();
//    private int waitForGrab = 150;
//    private int waitForHalt = 500;
//    private int intakeSleep = 1000;
//    private double s1Xerror = 0;
//    private double s1Yerror = 0;
//    private double s2Xerror = 0;
//    private double s2Yerror = 0;
//    private double s3Xerror = 0;
//    private double s3Yerror = 0;
//    private boolean autoStart = true;
//    private GrabAngle grabAngle = GrabAngle.VERTICAL_GRAB;
//    private GrabStyle grabStyle = GrabStyle.OUTSIDE_GRAB;
//    private WristAngle wristAngle = WristAngle.SPECIMEN_SCORE_1;
//    private boolean opModeRunning = true;
//    private AutoManagerPedro autoManager;
//    private Thread armHandlerThread = new Thread(() -> {
//        while (opModeIsActive()) {
////            arm.update(opModeIsActive());
////            intake.update(opModeIsActive());
////            autoManager.update();
//
//            telemetry.addData("X:", follower.getPose().getX());
//            telemetry.addData("Y:", follower.getPose().getY());
//            telemetry.addData("heading:", Math.toDegrees(follower.getTotalHeading()));
//            telemetry.addData("autoManager state:", autoState);
//            telemetry.update();
//        }
//    });
//    private final boolean debug = false;
//    private boolean pathStarted = true;
//    private PathChain currentPath;
//    private int lastAutoState = 0;
//    private AutoLocation autoLocation = AutoLocation.PEDRO_LEFT_4_0_V1;
//    private int nextAutoState = 0;
//    private final String autoName = "Left Intake Autonomous: PEDRO PATHING";
//    private Telemetry telemetryA;
//
//    public String getAutoName() {
//        return autoName;
//    }
//
//    public LeftV1_4_0_Pedro() {
//    }
//
//    public void runOpMode() {
//        follower = new Follower(hardwareMap);
////        follower.resetIMU();
//        telemetry.update();
//
//        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        robot = new HWProfile();
//        robot.init(hardwareMap, false, false);
//        params = new Params();
//        arm = new ArmSubsystem(robot, this, params);
//        intake = new IntakeSubsystem(robot, this, params);
//
//        currentMode = TeleopMode.IDLE;
//        arm.setTeleopMode(currentMode);
//        arm.poleToucherIn();
//        arm.setAutoMode(true);
//        arm.setPedroAuto(true);
//        arm.resetSlidesPosition();
//
//        intake.setWristAngle(wristAngle);
//        intake.setGrabAngle(grabAngle);
//        intake.setGrabStyle(grabStyle);
//        intake.intake();
//
//        intake.update(opModeIsActive());
//        arm.update(opModeIsActive());
//
//        autoManager = new AutoManagerPedro(this, follower, () -> {
//            arm.update(opModeIsActive());
//            intake.update(opModeIsActive());
//
////            telemetryA.addData("limelight fps: ", robot.limelight.getStatus().getFps());
//            telemetryA.addData("s1 x: ", s1Xerror);
//            telemetryA.addData("s1 y: ", s1Yerror);
//            telemetryA.addData("s2 x: ", s2Xerror);
//            telemetryA.addData("s2 y: ", s2Yerror);
//            telemetryA.addData("s3 x: ", s3Xerror);
//            telemetryA.addData("s3 y: ", s3Yerror);
//        }, arm, intake, robot);
//        autoManager.buildPaths(autoLocation);
//        follower.setPose(autoManager.start_4_0_V1);
//
//        currentMode = TeleopMode.IDLE;
//        arm.setTeleopMode(currentMode);
//        intake.intake();
//
//        autoManager.safeSleep(350);
//
//        arm.setSlidesMultiplier(1);
//
//        while (!opModeIsActive()) {
//            arm.update(opModeIsActive());
//            intake.update(opModeIsActive());
//            if(robot.limelight.getStatus().getFps() == 0) {
//                for(int i = 0; i <= 30; i++) {
//                    telemetry.addLine("UNPLUG AND REPLUG THE LIMELIGHT");
//                }
//            }
//            telemetry.addLine("The robot must be started on the third tile");
//            telemetry.addLine("to the right from the buckets, the robot must be");
//            telemetry.addLine("started on the left line of the tile, and the robot");
//            telemetry.addLine("needs to be facing towards the submersible");
//            telemetry.addLine();
//            follower.setPose(autoManager.start_4_0_V1);
//            telemetry.addData("limelight fps: ", robot.limelight.getStatus().getFps());
//            telemetry.addData("slides raw enc:", robot.slidesMotor1.getCurrentPosition());
//            telemetry.addData("x: ", follower.getPose().getX());
//            telemetry.addData("y: ", follower.getPose().getY());
//
////            LLResult result = robot.limelight.getLatestResult();
//
////            if (result != null) {
////                telemetry.addData("detectX: ", result.getPythonOutput()[0]);
////                telemetry.addData("detectY: ", result.getPythonOutput()[1]);
////            }
//            telemetry.update();
//        }
//
//        follower.setPose(autoManager.start_3_1_V1);
//
//        waitForStart();
//
////        limelightHandler.start();
//
//        if (!robot.limelight.isRunning()) {
//            robot.limelight.start();
//        } else if(robot.limelight.getLatestResult().getPythonOutput()[0] != 0) {
//            autoManager.safeSleep(50);
//            robot.limelight.pipelineSwitch(1);
//            autoManager.safeSleep(50);
//            robot.limelight.pipelineSwitch(3);
//            autoManager.safeSleep(50); //jank fix for an issue
//        }
//
//        time.reset();
//        arm.setArmPower(1);
//
//        autoManager.setSpeed(params.AUTO_DEFAULT_SPEED);
//        follower.updatePose();
//        follower.update();
//        autoManager.safeSleep(100);
//        follower.setPose(autoManager.start_4_0_V1);
//        autoManager.safeSleep(100);
//        params.AUTO_SCORE = 35;
//        params.AUTO_END_HEADING = -90;
////        autoManager.runPath(autoManager.toBucketPath, false);
//
////        currentMode = TeleopMode.BUCKET_SCORE;
////        arm.setTeleopMode(currentMode);
////        arm.setBucket(2);
////        autoManager.safeSleep(750);
//
//        bucketScore(1);
//        intake(1);
//        bucketScore(2);
//        intake(2);
//        bucketScore(3);
//        intake(3);
//        bucketScore(4);
//        park();
//
//        params.AUTO_END_HEADING = Math.toDegrees(follower.getTotalHeading());
//        params.TELEOP_START_MODE = TeleopMode.TOUCH_POLE_AUTO;
//        params.AUTO_SCORE = 32;
//
//        telemetryA.addData("time: ", time.time(TimeUnit.SECONDS));
//        telemetryA.update();
//        telemetryA.addData("s1 x: ", s1Xerror);
//        telemetryA.addData("s1 y: ", s1Yerror);
//        telemetryA.addData("s2 x: ", s2Xerror);
//        telemetryA.addData("s2 y: ", s2Yerror);
//        telemetryA.addData("s3 x: ", s3Xerror);
//        telemetryA.addData("s3 y: ", s3Yerror);
//        autoManager.safeSleep(10000);
//    }
//
//    public void park() {
//        autoManager.setSpeed(1);
//        params.TELEOP_START_MODE = TeleopMode.TOUCH_POLE_AUTO;
//        autoManager.runPath(autoManager.park, false);
////        arm.update(opModeIsActive());
////        autoManager.safeSleep(750);
////        arm.setParkArmUp(false);
////        autoManager.safeSleep(10000);
//    }
//
//    public void intake(int sampleNum) {
//        arm.setArmPower(1);
//
//        arm.setIntakePosition(params.PEDRO_AUTO_INTAKE_Y1_POS);
//        arm.intakeUpMode();
//        if (sampleNum != 3) {
////            arm.intakeDownMode();
//            arm.intakeUpMode();
//        } else {
//            arm.intakeUpMode();
//        }
//
//        Pose endPose = new Pose();
//
//        autoManager.setSpeed(params.AUTO_DEFAULT_SPEED);
//        if (sampleNum == 1) {
//            autoManager.setSpeed(1);
//            autoManager.runPath(autoManager.intakeYellow1, true);
//            autoManager.waitUntilBelowError(.1, .1, 1000);
////            autoManager.waitForArmAndSlides(300);
////            autoManager.waitForSlides();
//            endPose = autoManager.homeToSample(Math.toRadians(0));
////            autoManager.safeSleep(2500);
//        } else if (sampleNum == 2) {
//            autoManager.setSpeed(1);
//            autoManager.runPath(autoManager.intakeYellow2, true);
//            autoManager.safeSleep(700);
//            autoManager.waitUntilBelowError(.25, .1, 500);
//            endPose = autoManager.homeToSample(Math.toRadians(0));
////            autoManager.safeSleep(2500);
//        } else if (sampleNum == 3) {
//            autoManager.setSpeed(1);
//            arm.setAutoLastSample(true);
//            arm.setArmPower(1);
//            arm.intakeUpMode();
//            autoManager.runPath(autoManager.intakeYellow3, true);
//            arm.intakeUpMode();
//            autoManager.waitForArmAndSlides(250);
//            endPose = autoManager.homeToSample(Math.toRadians(0));
////            autoManager.safeSleep(2500);
//        }
//
//        if(endPose == new Pose()) endPose = follower.getPose();
//
//        autoManager.setSpeed(1);
//
//        double xTarget = 0;
//
//        if(sampleNum == 1) {
//            arm.intakeDownMode();
//            arm.setArmPower(.6);
//
//            xTarget = endPose.getX() + 4.5;
//
//            autoManager.holdPoint(new Pose(
//                    endPose.getX() + 4.5,
//                    autoManager.intakeYellow1.build().getPath(autoManager.intakeYellow1.build().size() - 1).getLastControlPoint().getY(),
//                    autoManager.intakeYellow1.build().getPath(autoManager.intakeYellow1.build().size() - 1).getPathEndHeadingConstraint()
//            ));
//        } else if(sampleNum == 2) {
//            arm.intakeDownMode();
//            arm.setArmPower(.6);
//
//            xTarget = endPose.getX() + 4.5;
//
//            autoManager.holdPoint(new Pose(
//                    endPose.getX() + 4.5,
//                    autoManager.intakeYellow2.build().getPath(autoManager.intakeYellow2.build().size() - 1).getLastControlPoint().getY(),
//                    autoManager.intakeYellow2.build().getPath(autoManager.intakeYellow2.build().size() - 1).getPathEndHeadingConstraint()
//            ));
//        } else if(sampleNum == 3) {
//            xTarget = endPose.getX() + 3;
//
//            autoManager.holdPoint(new Pose(
//                    endPose.getX() + 3,
//                    20,
//                    follower.getTotalHeading()
//            ));
//            autoManager.safeSleep(300);
//            arm.intakeDownMode();
//
//            arm.setArmPower(1);
//        }
//        autoManager.safeSleep(650);
////        autoManager.waitForArmAndSlides(1000);
//
//        intake.intake();
//        autoManager.safeSleep(250);
//
////        s1Xerror = Math.abs(follower.getPose().getX() - xTarget);
////        s1Yerror = endPose.getY();
////
//        if (sampleNum == 1) {
//            s1Xerror = Math.abs(follower.getPose().getX() - xTarget);
//            s1Yerror = endPose.getY();
//        } else if (sampleNum == 2) {
//            s2Xerror = Math.abs(follower.getPose().getX() - xTarget);
//            s2Yerror = endPose.getY();
//        } else if (sampleNum == 3) {
//            s3Xerror = Math.abs(follower.getPose().getX() - xTarget);
//            s3Yerror = endPose.getY();
//        }
//
//        autoManager.setSpeed(params.AUTO_DEFAULT_SPEED);
//
//        arm.setArmPower(1);
//        currentMode = TeleopMode.BUCKET_SCORE;
//        arm.setTeleopMode(currentMode);
////        if(sampleNum != 3) arm.setAnimationType(AnimationType.FAST);
//        arm.setBucket(2);
//        arm.update(opModeIsActive());
//        autoManager.safeSleep(800); //1250
//        arm.setArmPower(.6);
//    }
//
//    public void bucketScore(int sampleNum) {
//        intake.setGrabAngle(GrabAngle.VERTICAL_GRAB);
//        intake.update(opModeIsActive());
//
//        if (arm.getTeleopMode() != TeleopMode.BUCKET_SCORE) {
//            currentMode = TeleopMode.BUCKET_SCORE;
//            arm.setTeleopMode(currentMode);
//            arm.setBucket(2);
//            arm.update(opModeIsActive());
//            if (sampleNum == 1) {
//                autoManager.safeSleep(1250);
//            } else {
//                autoManager.safeSleep(100);
//            }
//        }
//
//        autoManager.setSpeed(1);
//        if (sampleNum == 1) {
//            autoManager.runPath(autoManager.bucketScorePathS1, true);
//        } else if (sampleNum == 2) {
//            autoManager.runPath(autoManager.bucketScorePathFromIntake1, true);
//        } else if (sampleNum == 3) {
//            autoManager.runPath(autoManager.bucketScorePathFromIntake2, true);
//        } else if (sampleNum == 4) {
//            autoManager.runPath(autoManager.bucketScorePathFromIntake3, true);
//        }
//
//        autoManager.setSpeed(params.AUTO_DEFAULT_SPEED);
//
//        autoManager.waitUntilBelowError(.25, .25,200);
//        arm.setArmTipBucketScore(true);
//        autoManager.safeSleep(300);
//        intake.outtake();
//        autoManager.safeSleep(100);
//        arm.setArmTipBucketScore(false);
//        if(sampleNum != 4) {
//            autoManager.safeSleep(600);
//        } else {
//            autoManager.safeSleep(800);
//        }
//
//        if (sampleNum != 4) {
////            autoManager.setSpeed(.6);
////            autoManager.runPath(autoManager.backupPath, false);
////            currentMode = TeleopMode.DOWN;
////            arm.setTeleopMode(currentMode);
////            arm.setIntakePosition(params.PEDRO_AUTO_INTAKE_Y1_POS);
////            arm.intakeUpMode();
////            arm.update(opModeIsActive());
////            autoManager.safeSleep(250);
////            autoManager.setSpeed(1);
//        }
//    }
//}
