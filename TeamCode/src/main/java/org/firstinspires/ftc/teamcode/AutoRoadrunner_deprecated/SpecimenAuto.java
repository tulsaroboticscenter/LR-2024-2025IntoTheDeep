//package org.firstinspires.ftc.teamcode.AutoRoadrunner_deprecated;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.roadrunner.InstantAction;
//import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
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
//import org.firstinspires.ftc.teamcode.Enums.AutoSpeed;
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
//import java.util.concurrent.TimeUnit;
//
//public class SpecimenAuto extends AutoProgram {
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
//    private final boolean debug = false;
//    private boolean pathStarted = true;
//    private int lastAutoState = 0;
//    private AutoLocation autoLocation = AutoLocation.RR_SPECIMEN;
//    private int nextAutoState = 0;
//    private final String autoName = "Specimen Auto";
//    private TrajectoryActionBuilder currentPath;
//    private InstantAction updateAction;
//    private boolean sense = false;
//    private int specimenScoreCount = 0;
//    private final double specimenOffset = 3.5;
//    private Thread armHandlerThread = new Thread(() -> {
//        while (opMode.opModeIsActive()) {
//            arm.update();
//            intake.update();
//
//            if (arm.teleopModeStart) ltopmodewasstart = true;
//
////            telemetry.addData("distance: ", robot.distanceOne.getDistance(DistanceUnit.INCH));
//            telemetry.addData("stop requested: ", opMode.isStopRequested());
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
//
//
//    public String getAutoName() {
//        return autoName;
//    }
//
//    public SpecimenAuto() {
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
//        drive = new MecanumDrive(hardwareMap, autoManager.specimenAutoStart, false);
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
//    public void intakePath(boolean afterPush) {
//        drive.setCorrectionType(AutoCorrectionType.TIME_BASED);
//        drive.setAutoSpeed(AutoSpeed.STANDARD);
//
//        if (afterPush) {
//            Actions.runBlocking(drive.actionBuilder(drive.pose)
//                    .strafeToLinearHeading(autoManager.specimenIntakePath1, autoManager.specimenIntakePath1Heading)
//                    .build()
//            );
//        } else {
//            Actions.runBlocking(drive.actionBuilder(drive.pose)
//                    .strafeToLinearHeading(autoManager.specimenIntakeNoPushPath, autoManager.specimenIntakeNoPushPathHeading)
//                    .strafeToLinearHeading(autoManager.specimenIntakePath1, autoManager.specimenIntakePath1Heading)
//                    .build()
//            );
//        }
//
//        currentMode = TeleopMode.DOWN;
//        arm.setTeleopMode(currentMode);
//        arm.intakeSpecimen = true;
//        arm.intakeUpSpecimen = 0;
//        arm.update();
//
//        drive.setCorrectionType(AutoCorrectionType.PRECISE);
//        drive.setAutoSpeed(AutoSpeed.STANDARD);
//
////        Actions.runBlocking(drive.actionBuilder(drive.pose)
////                .afterTime(.5, new InstantAction(() -> {
////                    relocalizeX();
////                }))
////                .strafeToLinearHeading(autoManager.specimenIntakePath2, autoManager.specimenIntakePath2Heading)
////                .build()
////        );
//
//        autoManager.setDrive(drive);
//        autoManager.homeToSpecimen(robot);
//
//        drive.setAutoSpeed(AutoSpeed.STANDARD);
//
////        while (!opMode.isStopRequested()) continue;
//
//        arm.intakeUpSpecimen = 0;
//        safeWait(500);
//        intake.intake();
//        safeWait(500);
//        arm.intakeUpSpecimen = 2;
//        safeWait(500);
//
//        currentMode = TeleopMode.IDLE;
//        arm.setTeleopMode(currentMode);
//        arm.update();
//    }
//
//    public void specimenScore(boolean afterIntake) {
//        if (afterIntake) {
//            currentMode = TeleopMode.SPECIMEN_SCORE;
//            arm.setTeleopMode(currentMode);
//            arm.update();
//        }
//
//        arm.setArmPositionSpecimen(100);
//        arm.setSlidesPositionSpecimen(11);
//
//        if (!afterIntake) safeWait(750);
//
//        if (afterIntake) {
//            drive.setAutoSpeed(AutoSpeed.FAST);
//            drive.setCorrectionType(AutoCorrectionType.TIME_BASED);
//
//            Actions.runBlocking(drive.actionBuilder(drive.pose)
////                    .strafeToLinearHeading(autoManager.specimenToScore1, autoManager.specimenToScore1Heading)
//                            .strafeToLinearHeading(autoManager.specimenToScore2, autoManager.specimenToScore2Heading)
//                            .build()
//            );
//        }
//
//        drive.setAutoSpeed(AutoSpeed.STANDARD);
//        drive.setCorrectionType(AutoCorrectionType.TIME_BASED);
//
//        autoManager.specimenScorePath = new Vector2d(autoManager.specimenScorePath.x, autoManager.specimenScorePath.y + (specimenScoreCount * specimenOffset));
//
//        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                .afterTime(.25, new InstantAction(() -> {
//                    if (!afterIntake) {
//                        currentMode = TeleopMode.SPECIMEN_SCORE;
//                        arm.setTeleopMode(currentMode);
//                        arm.update();
//
//                        arm.setArmPositionSpecimen(100);
//                        arm.setSlidesPositionSpecimen(11);
//                    }
//                }))
//                .strafeToLinearHeading(autoManager.specimenScorePath, autoManager.specimenScorePathHeading)
//                .build()
//        );
//
//        arm.setArmPositionSpecimen(85);
//        arm.setSlidesPositionSpecimen(6);
//
//        safeWait(750);
//        intake.outtake();
//        intake.update();
//
//        specimenScoreCount++;
//    }
//
//    public void pushSample(int sampleNum) {
//        drive.setCorrectionType(AutoCorrectionType.FAST);
//        drive.setAutoSpeed(AutoSpeed.FAST);
//
//        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                .strafeToLinearHeading(autoManager.specimenPushMoveToHP1, autoManager.specimenPushMoveToHP1Heading)
//                .strafeToLinearHeading(autoManager.specimenPushMoveToHP2, autoManager.specimenPushMoveToHP2Heading)
//                .strafeToLinearHeading(autoManager.specimenPushMoveToHP3, autoManager.specimenPushMoveToHP3Heading)
//                .build()
//        );
//
//        currentMode = TeleopMode.IDLE;
//        arm.setTeleopMode(currentMode);
//        arm.update();
//
//        if (sampleNum == 1) {
//            Actions.runBlocking(drive.actionBuilder(drive.pose)
//                    .strafeToLinearHeading(autoManager.specimenPushFirstPath1, autoManager.specimenPushFirstPath1Heading)
//                    .stopAndAdd(new InstantAction(() -> {
//                        drive.setAutoSpeed(AutoSpeed.STANDARD);
//                    }))
//                    .strafeToLinearHeading(autoManager.specimenPushFirstPath2, autoManager.specimenPushFirstPath2Heading)
//                    .build()
//            );
//        } else if (sampleNum == 2) {
//            Actions.runBlocking(drive.actionBuilder(drive.pose)
//                    .strafeToLinearHeading(autoManager.specimenPushSecondPath1, autoManager.specimenPushSecondPath1Heading)
//                    .stopAndAdd(new InstantAction(() -> {
//                        drive.setAutoSpeed(AutoSpeed.STANDARD);
//                    }))
//                    .strafeToLinearHeading(autoManager.specimenPushSecondPath2, autoManager.specimenPushSecondPath2Heading)
//                    .build()
//            );
//        }
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
//        drive.setAutoSpeed(AutoSpeed.FAST);
//        drive.setCorrectionType(AutoCorrectionType.TIME_BASED);
//
////        Actions.runBlocking(drive.actionBuilder(drive.pose)
////                .strafeToLinearHeading(autoManager.moveForwardPath, autoManager.moveForwardPathHeading)
////                .build()
////        );
//
//        specimenScore(false);
//        pushSample(1);
////        intakePath(true);
////        specimenScore(true);
////        pushSample(2);
////        intakePath(false);
////        specimenScore(true);
//
////        autoManager.updatePose(drive.pose);
////        autoManager.buildPaths(autoLocation);
////        autoManager.runPath(autoManager.toBucketPath);
////
////        currentMode = TeleopMode.AUTO_SLIDES_IN;
////        arm.setTeleopMode(currentMode);
//
//        params.AUTO_END_HEADING = Math.toDegrees(drive.pose.heading.toDouble());
//        safeWait(10000);
//
//    }
//
//    public void waitForArm() {
//        while (!arm.armAtPosition() && !arm.slidesAtPosition()) drive.updatePoseEstimate();
//        arm.update();
//    }
//
//    public void safeWait(int millis) {
//        ElapsedTime timer = new ElapsedTime();
//        timer.reset();
//
//        while (timer.time(TimeUnit.MILLISECONDS) < millis) {
//            arm.update();
//            intake.update();
//            drive.updatePoseEstimate();
//
//            if (opMode.isStopRequested()) break;
//        }
//    }
//
//    public void relocalizeX() {
////        double distance = robot.distanceOne.getDistance(DistanceUnit.INCH);
////
////        drive.setPose(new Pose2d(new Vector2d(distance - params.DISTANCE_ONE_OFFSET, drive.pose.position.y), drive.pose.heading));
//    }
//}
