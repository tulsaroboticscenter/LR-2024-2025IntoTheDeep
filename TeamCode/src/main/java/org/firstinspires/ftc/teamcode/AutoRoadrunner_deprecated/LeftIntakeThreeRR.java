//package org.firstinspires.ftc.teamcode.AutoRoadrunner_deprecated;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.InstantAction;
//import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
//import org.firstinspires.ftc.teamcode.OpenCV.SampleDetectionPipelinePNP;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvWebcam;
//
//public class LeftIntakeThreeRR extends AutoProgram {
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
//    private long outtakeSleep = 600;
//    private long waitForArmIntakeDown = 500;
//    private long waitToIntake = 1250;
//    private SampleDetectionPipelinePNP detectionPipe;
//    private OpenCvWebcam camera;
//    private long waitForGrab = 150;
//    private long waitForHalt = 500;
//    private long intakeSleep = 1000;
//    private boolean autoStart = true;
//    private GrabAngle grabAngle = GrabAngle.VERTICAL_GRAB;
//    private GrabStyle grabStyle = GrabStyle.OUTSIDE_GRAB;
//    private AutoManager autoManager;
//    private HardwareMap hardwareMap;
//    private Telemetry telemetry;
//    private int threadCycle = 0;
//    private boolean ltopmodewasstart = false;
//    private Thread armHandlerThread = new Thread(() -> {
//        while (opMode.opModeIsActive()) {
//            arm.update();
//            intake.update();
//
//            if (arm.teleopModeStart) ltopmodewasstart = true;
//
//            telemetry.addData("arm transistion stage:", arm.armTransistionStage);
//            telemetry.addData("teleopModeStartWorked:", ltopmodewasstart);
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
//
//        intake.outtake();
//        intake.update();
//        params.AUTO_END_HEADING = Math.toDegrees(drive.pose.heading.toDouble());
//    });
//    private final boolean debug = false;
//    private boolean pathStarted = true;
//    private int lastAutoState = 0;
//    private AutoLocation autoLocation = AutoLocation.RR_LEFT_SCORE_THREE_GOLD;
//    private int nextAutoState = 0;
//    private final String autoName = "Left Intake Autonomous New";
//    private TrajectoryActionBuilder currentPath;
//    private InstantAction updateAction;
//    private boolean sense = false;
//
//
//    public String getAutoName() {
//        return autoName;
//    }
//
//    public LeftIntakeThreeRR() {
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
//        drive = new MecanumDrive(hardwareMap, autoManager.leftIntakeThreeStartPosition, false);
//
//        drive.updatePoseEstimate();
//
//        autoManager.setDrive(drive);
//        robot.slidesMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        detectionPipe = new SampleDetectionPipelinePNP();
//
//        camera.setPipeline(detectionPipe);
//
//        FtcDashboard.getInstance().startCameraStream(camera, 30);
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
//        // OR...  Do Not Activate the Camera Monitor View
//        //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
//
//        /*
//         * Specify the image processing pipeline we wish to invoke upon receipt
//         * of a frame from the camera. Note that switching pipelines on-the-fly
//         * (while a streaming session is in flight) *IS* supported.
//         */
//        /*
//         * Open the connection to the camera device. New in v1.4.0 is the ability
//         * to open the camera asynchronously, and this is now the recommended way
//         * to do it. The benefits of opening async include faster init time, and
//         * better behavior when pressing stop during init (i.e. less of a chance
//         * of tripping the stuck watchdog)
//         *
//         * If you really want to open synchronously, the old method is still available.
//         */
//        camera.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
//        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                /*
//                 * Tell the webcam to start streaming images to us! Note that you must make sure
//                 * the resolution you specify is supported by the camera. If it is not, an exception
//                 * will be thrown.
//                 *
//                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
//                 * supports streaming from the webcam in the uncompressed YUV image format. This means
//                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
//                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
//                 *
//                 * Also, we specify the rotation that the webcam is used in. This is so that the image
//                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
//                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
//                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
//                 * away from the user.
//                 */
//                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//                /*
//                 * This will be called if the camera could not be opened
//                 */
//            }
//        });
//
//
//        startAuto();
//    }
//
//    public void intakePath(double intakePosition, boolean quickArm) {
//        arm.setArmPower(params.ARM_POWER_DEFAULT);
//        arm.setSlidesMultiplier(1);
////        autoManager.updatePose(drive.pose);
////        autoManager.buildPaths(autoLocation);
////        autoManager.backupPath = autoManager.backupPath
////                .afterTime(.35, new InstantAction(() -> {
////                    if(quickArm) {
////                        grabAngle = GrabAngle.HORIZONTAL_GRAB;
////                        intake.setGrabAngle(grabAngle);
////
////                        currentMode = TeleopMode.DOWN;
////                        arm.setTeleopMode(currentMode);
////                        arm.setIntakePosition(params.THREE_AUTO_INTAKE_Y1_POS);
////                        arm.intakeUpMode();
////                    }
////                }))
////                .strafeToLinearHeading(new Vector2d(20, 0), Math.toRadians(45));
////        autoManager.runPath(autoManager.backupPath);
//
//        autoManager.updatePose(drive.pose);
//        autoManager.buildPaths(autoLocation);
//        if (intakePosition == 1) {
//            drive.setCorrectionType(AutoCorrectionType.TIME_BASED);
//            intake.setShortRange(false);
//
//            drive.updatePoseEstimate();
//            currentPath = drive.actionBuilder(drive.pose)
//                    .afterTime(.75, new InstantAction(() -> {
//                        if (quickArm) {
//                            grabAngle = GrabAngle.HORIZONTAL_GRAB;
//                            intake.setGrabAngle(grabAngle);
//
//                            currentMode = TeleopMode.DOWN;
//                            arm.setTeleopMode(currentMode);
//                            arm.setIntakePosition(params.THREE_AUTO_INTAKE_Y1_POS);
//                            arm.intakeUpMode();
//                        }
//                    }))
//                    .strafeToLinearHeading(new Vector2d(33, -5.5), Math.toRadians(90))
//                    .afterTime(0, new InstantAction(() -> {
//                        drive.setCorrectionType(AutoCorrectionType.PRECISE);
//                    }))
//                    .strafeToLinearHeading(new Vector2d(38.5, -5.35), Math.toRadians(90));
//            autoManager.runPath(currentPath);
//
//            drive.setCorrectionType(AutoCorrectionType.TIME_BASED);
//
//            if (sense) {
//                AutoHomeAction autoHomeAction = new AutoHomeAction(drive, camera, detectionPipe, opMode);
//                boolean homed = false;
//
//                while (!homed && opMode.opModeIsActive()) {
//                    homed = !autoHomeAction.run(new TelemetryPacket());
//                }
//            }
//        } else if (intakePosition == 2) {
//            intake.setShortRange(false);
//            drive.updatePoseEstimate();
//
//            currentPath = drive.actionBuilder(drive.pose)
//                    .afterTime(.75, new InstantAction(() -> {
//                        if (quickArm) {
//                            grabAngle = GrabAngle.HORIZONTAL_GRAB;
//                            intake.setGrabAngle(grabAngle);
//
//                            currentMode = TeleopMode.DOWN;
//                            arm.setTeleopMode(currentMode);
//                            arm.setIntakePosition(params.THREE_AUTO_INTAKE_Y1_POS);
//                            arm.intakeUpMode();
//                        }
//                    }))
//                    .strafeToLinearHeading(autoManager.intakeYellow2StrafeTo, autoManager.intakeYellow2StrafeToHeading)
//                    .afterTime(0, new InstantAction(() -> {
//                        drive.setCorrectionType(AutoCorrectionType.PRECISE);
//                    }))
//                    .strafeToLinearHeading(autoManager.intakeYellow2Path, autoManager.intakeYellow2PathHeading);
//            autoManager.runPath(currentPath);
//
//            if (sense) {
//                AutoHomeAction autoHomeAction = new AutoHomeAction(drive, camera, detectionPipe, opMode);
//                boolean homed = false;
//
//                while (!homed && opMode.opModeIsActive()) {
//                    homed = !autoHomeAction.run(new TelemetryPacket());
//                }
//            }
//
//            drive.setCorrectionType(AutoCorrectionType.TIME_BASED);
//
//        } else if (intakePosition == 3) {
//            intake.setShortRange(false);
//            intake.outtake();
//            intake.update();
//
//            arm.setAutoLastSample(true);
//            arm.update();
//
//            drive.updatePoseEstimate();
//            currentPath = drive.actionBuilder(drive.pose)
//                    .afterTime(.35, new InstantAction(() -> {
//                        if (quickArm) {
//                            grabAngle = GrabAngle.HORIZONTAL_GRAB;
//                            intake.setGrabAngle(grabAngle);
//
//                            currentMode = TeleopMode.DOWN;
//                            arm.setTeleopMode(currentMode);
//                            arm.setIntakePosition(params.THREE_AUTO_INTAKE_Y1_POS);
//                            arm.intakeUpMode();
//                        }
//                    }))
//                    .strafeToLinearHeading(autoManager.intakeYellow3StrafeTo, autoManager.intakeYellow3StrafeToHeading)
//                    .afterTime(0, new InstantAction(() -> {
//                        drive.setCorrectionType(AutoCorrectionType.PRECISE);
//                    }))
//                    .strafeToLinearHeading(autoManager.intakeYellow3Path, autoManager.intakeYellow3PathHeading);
//            autoManager.runPath(currentPath);
//
//            if (sense) {
////                AutoHomeAction autoHomeAction = new AutoHomeAction(drive, camera, detectionPipe, opMode);
////                boolean homed = false;
////
////                while (!homed && opMode.opModeIsActive()) {
////                    homed = !autoHomeAction.run(new TelemetryPacket());
////                }
//            }
//            drive.setCorrectionType(AutoCorrectionType.TIME_BASED);
//        }
//
//        if (!quickArm) {
//            grabAngle = GrabAngle.HORIZONTAL_GRAB;
//            intake.setGrabAngle(grabAngle);
//
//            currentMode = TeleopMode.DOWN;
//            arm.setTeleopMode(currentMode);
//            arm.setIntakePosition(params.THREE_AUTO_INTAKE_Y1_POS);
//            arm.intakeUpMode();
////        arm.setArmPower(params.ARM_POWER_SLOW_AUTO);
//            arm.update();
//            waitForArm();
//        }
//        drive.setCorrectionType(AutoCorrectionType.TIME_BASED);
//
//        opMode.sleep(100);
//        arm.intakeDownMode();
//        if(intakePosition != 3) {
//            opMode.sleep(waitForArmIntakeDown);
//        } else {
//            opMode.sleep(waitForArmIntakeDown + 350);
//        }
////        waitForArm();
//        intake.intake();
//        opMode.sleep(waitForGrab);
//
//        if (quickArm) {
//            currentMode = TeleopMode.BUCKET_SCORE;
//            arm.setTeleopMode(currentMode);
//            arm.setBucket(2);
//            arm.update();
//            opMode.sleep(100);
//        } else {
//            currentMode = TeleopMode.IDLE;
//            arm.setTeleopMode(currentMode);
//            opMode.sleep(armWaitSleep);
//        }
//    }
//
//    public void bucketScore(boolean goToStandby, boolean quickArm) {
//        grabAngle = GrabAngle.VERTICAL_GRAB;
//        intake.setGrabAngle(grabAngle);
//
//        if (quickArm) {
//            currentMode = TeleopMode.BUCKET_SCORE;
//            arm.setTeleopMode(currentMode);
//            arm.setBucket(2);
//            arm.update();
//            opMode.sleep(100);
//        }
//
//        if (goToStandby) {
//            autoManager.updatePose(drive.pose);
//            autoManager.buildPaths(autoLocation);
//            autoManager.runPath(autoManager.toBucketPath);
//        }
//
//        if (!quickArm) {
//            currentMode = TeleopMode.BUCKET_SCORE;
//            arm.setTeleopMode(currentMode);
//            arm.setBucket(2);
//            arm.update();
//            opMode.sleep(armWaitSleep);
//        }
//
//        autoManager.updatePose(drive.pose);
//        autoManager.buildPaths(autoLocation);
//        autoManager.runPath(autoManager.bucketScorePath);
//
//        arm.setArmTipBucketScore(true);
//        opMode.sleep(outtakeSleep);
//        intake.outtake();
//        arm.setArmTipBucketScore(false);
//        opMode.sleep(500);
//    }
//
//    public void startAuto() {
//        if (!armHandlerThread.isAlive()) armHandlerThread.start();
//
//        updateAction = new InstantAction(() -> {
////            arm.update();
////            intake.update();
//        });
//
//        drive.setUpdateAction(updateAction);
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
//        bucketScore(false, false);
//        intakePath(1, true);
//        bucketScore(true, true);
//        intakePath(2, true);
//        bucketScore(true, true);
//        intakePath(3, true);
//        bucketScore(true, true);
//
////        autoManager.updatePose(drive.pose);
////        autoManager.buildPaths(autoLocation);
////        autoManager.runPath(autoManager.toBucketPath);
////
////        currentMode = TeleopMode.AUTO_SLIDES_IN;
////        arm.setTeleopMode(currentMode);
//
//        params.AUTO_END_HEADING = Math.toDegrees(drive.pose.heading.toDouble());
//        opMode.sleep(10000);
//
//    }
//
//    public void waitForArm() {
//        while (!arm.armAtPosition() && !arm.slidesAtPosition()) drive.updatePoseEstimate();
//        arm.update();
//    }
//}
