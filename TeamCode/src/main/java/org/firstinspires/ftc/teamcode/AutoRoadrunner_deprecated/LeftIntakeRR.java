//package org.firstinspires.ftc.teamcode.AutoRoadrunner_deprecated;
//
//import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.Abstracts.AutoProgram;
//import org.firstinspires.ftc.teamcode.Enums.AutoCorrectionType;
//import org.firstinspires.ftc.teamcode.Enums.AutoLocation;
//import org.firstinspires.ftc.teamcode.Enums.GrabAngle;
//import org.firstinspires.ftc.teamcode.Enums.GrabStyle;
//import org.firstinspires.ftc.teamcode.Enums.TeleopMode;
//import org.firstinspires.ftc.teamcode.Hardware.ArmSubsystem;
//import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
//import org.firstinspires.ftc.teamcode.Hardware.IntakeSubsystem;
//import org.firstinspires.ftc.teamcode.Hardware.Params;
//
//public class LeftIntakeRR extends AutoProgram {
//    private LinearOpMode opMode;
//    private MecanumDrive drive;
//    private ElapsedTime autoTime = new ElapsedTime();
//    private int autoState = 1;
//    private HWProfile robot;
//    private Params params;
//    private ArmSubsystem arm;
//    private IntakeSubsystem intake;
//    private TeleopMode currentMode = TeleopMode.IDLE;
//    private long armWaitSleep = 750;
//    private long outtakeSleep = 500;
//    private long waitForArmIntakeDown = 1250;
//    private long waitToIntake = 1250;
//    private long waitForGrab = 350;
//    private long waitForHalt = 500;
//    private long intakeSleep = 1000;
//    private boolean autoStart = true;
//    private GrabAngle grabAngle = GrabAngle.VERTICAL_GRAB;
//    private GrabStyle grabStyle = GrabStyle.OUTSIDE_GRAB;
//    private AutoManager autoManager;
//    private HardwareMap hardwareMap;
//    private Telemetry telemetry;
//    private int threadCycle = 0;
//    private TrajectoryActionBuilder currentPath;
//    private Thread armHandlerThread = new Thread(() -> {
//        while (opMode.opModeIsActive()) {
//            arm.update();
//            intake.update();
//            threadCycle++;
//
//            telemetry.addData("slides transistion:", arm.armTransistionStage);
//            telemetry.addData("X:", drive.pose.position.x);
//            telemetry.addData("Arm pos:", arm.getArmPosition());
//            telemetry.addData("Slides pos:", arm.getSlidesPosition());
//            telemetry.addData("slides at pos:", arm.slidesAtPosition());
//            telemetry.addData("thread cycle:", threadCycle);
//            telemetry.addData("Y:", drive.pose.position.y);
//            telemetry.addData("heading:", Math.toDegrees(drive.pose.heading.toDouble()));
//            telemetry.update();
//        }
//    });
//    private final boolean debug = false;
//    private boolean pathStarted = true;
//    private int lastAutoState = 0;
//    private AutoLocation autoLocation = AutoLocation.RR_LEFT_SCORE_TWO_GOLD;
//    private int nextAutoState = 0;
//    private final String autoName = "Left Intake Autonomous Old";
//
//    public String getAutoName() {
//        return autoName;
//    }
//
//    public LeftIntakeRR() {
//
//    }
//
//    public void init(LinearOpMode _opMode) {
//        opMode = _opMode;
//        telemetry = opMode.telemetry;
//        hardwareMap = opMode.hardwareMap;
//
//        telemetry.update();
//
//        robot = new HWProfile();
//        robot.init(hardwareMap, false, false);
//        params = new Params();
//        arm = new ArmSubsystem(robot, opMode, params);
//        intake = new IntakeSubsystem(robot, opMode, params);
//
//        currentMode = TeleopMode.IDLE;
//        arm.setTeleopMode(currentMode);
//        arm.setAutoMode(true);
//        arm.poleToucherIn();
//
//        intake.setGrabAngle(grabAngle);
//        intake.setGrabStyle(grabStyle);
//        intake.intake();
//
//        intake.update();
//        arm.update();
//
//        arm.resetSlidesPosition();
//
//        autoManager = new AutoManager(opMode);
//        drive = new MecanumDrive(hardwareMap, autoManager.leftIntakeTwoStartPosition, false);
//
//        drive.updatePoseEstimate();
//
//        autoManager.setDrive(drive);
//        robot.slidesMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        while (!opMode.opModeIsActive()) {
//            arm.update();
//            telemetry.addData("X:", drive.pose.position.x);
//            telemetry.addData("slides raw enc:", robot.slidesMotor1.getCurrentPosition());
//            telemetry.addData("Y:", drive.pose.position.y);
//            telemetry.addData("heading:", Math.toDegrees(drive.pose.heading.toDouble()));
//            telemetry.update();
//        }
//
//
//        startAuto();
//    }
//
//    public void intakePath(double intakePosition) {
//        arm.setArmPower(params.ARM_POWER_DEFAULT);
//        arm.setSlidesMultiplier(1);
//        autoManager.updatePose(drive.pose);
//        autoManager.buildPaths(autoLocation);
//        autoManager.runPath(autoManager.backupPath);
//
//        currentMode = TeleopMode.DOWN;
//        arm.setTeleopMode(currentMode);
//        arm.setIntakePosition(params.TWO_AUTO_INTAKE_Y1_POS);
//        arm.intakeUpMode();
////        arm.setArmPower(params.ARM_POWER_SLOW_AUTO);
//        arm.update();
//        waitForArm();
//
//        autoManager.updatePose(drive.pose);
//        autoManager.buildPaths(autoLocation);
//        drive.setCorrectionType(AutoCorrectionType.PRECISE);
//        if(intakePosition == 1) {
//            drive.updatePoseEstimate();
//            currentPath = drive.actionBuilder(drive.pose)
//                .strafeToLinearHeading(autoManager.intakeYellow1Path, autoManager.intakeYellow1PathHeading);
//            autoManager.runPath(currentPath);
//        } else if(intakePosition == 2) {
//            drive.updatePoseEstimate();
//            currentPath = drive.actionBuilder(drive.pose)
//                    .strafeToLinearHeading(autoManager.intakeYellow2Path, autoManager.intakeYellow2PathHeading);
//            autoManager.runPath(currentPath);
//        }
//        drive.setCorrectionType(AutoCorrectionType.TIME_BASED);
//
//        opMode.sleep(500);
//        arm.intakeDownMode();
//        opMode.sleep(waitForArmIntakeDown);
////        waitForArm();
//        intake.intake();
//        opMode.sleep(waitForGrab);
//
//        currentMode = TeleopMode.IDLE;
//        arm.setTeleopMode(currentMode);
//        opMode.sleep(armWaitSleep);
//    }
//
//    public void bucketScore() {
//        autoManager.updatePose(drive.pose);
//        autoManager.buildPaths(autoLocation);
//        autoManager.runPath(autoManager.toBucketPath);
//
//        currentMode = TeleopMode.BUCKET_SCORE;
//        arm.setTeleopMode(currentMode);
//        arm.setBucket(2);
//        arm.update();
//        opMode.sleep(armWaitSleep);
//
//        autoManager.updatePose(drive.pose);
//        autoManager.buildPaths(autoLocation);
//        autoManager.runPath(autoManager.bucketScorePath);
//
//        arm.setArmTipBucketScore(true);
//        opMode.sleep(outtakeSleep);
//        intake.outtake();
//        arm.setArmTipBucketScore(false);
//        opMode.sleep(750);
//    }
//
//    public void startAuto() {
//        if(!armHandlerThread.isAlive()) armHandlerThread.start();
//
//        arm.poleToucherOut();
//        arm.setArmPower(params.ARM_POWER_DEFAULT);
//
//        autoManager.updatePose(drive.pose);
//        autoManager.buildPaths(autoLocation);
//
//        arm.setSlidesMultiplier(params.SLIDE_MOTOR_POWER);
//        autoManager.disablePathing(debug);
//        arm.update();
//
////        autoManager.updatePose(drive.pose);
////        autoManager.buildPaths(autoLocation);
////        autoManager.runPath(autoManager.toBucketPath);
//
//        bucketScore();
//        intakePath(1);
//        bucketScore();
//        intakePath(2);
//        bucketScore();
//
////        autoManager.updatePose(drive.pose);
////        autoManager.buildPaths(autoLocation);
////        autoManager.runPath(autoManager.toBucketPath);
////
////        currentMode = TeleopMode.AUTO_SLIDES_IN;
////        arm.setTeleopMode(currentMode);
//
//        opMode.sleep(10000);
//
//    }
//
//    public void waitForArm() {
//        while (!arm.armAtPosition() && !arm.slidesAtPosition()) drive.updatePoseEstimate(); arm.update();
//    }
//}
