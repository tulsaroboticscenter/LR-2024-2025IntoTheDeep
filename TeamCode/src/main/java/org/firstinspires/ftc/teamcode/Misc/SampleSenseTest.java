package org.firstinspires.ftc.teamcode.Misc;

import android.graphics.Canvas;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.AutoPedro.AutoManagerPedro;
import org.firstinspires.ftc.teamcode.Enums.TeleopMode;
import org.firstinspires.ftc.teamcode.Hardware.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Hardware.Params;
import org.firstinspires.ftc.teamcode.OpenCV.SampleDetectCustom;
import org.firstinspires.ftc.teamcode.OpenCV.SampleDetectionPipeline;
import org.firstinspires.ftc.teamcode.OpenCV.SampleDetectionPipelinePNP;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "2: Sample Sense Test", group = "1")
@Config
public class SampleSenseTest extends LinearOpMode {
    SampleDetectionPipeline detectionPipe;
    OpenCvWebcam webcam;
    private Follower follower;
    double startX = 0;
    public static double kP = .001;
    public static double kI = 0;
    private boolean homed = false;
    private boolean runHome = false;
    public static double kD = 0;
    private PIDController drivePid = new PIDController(kP, kI, kD);
    AutoManagerPedro autoManager;
    private IntakeSubsystem intake;
    private HWProfile robot;
    private double currentTargetX = 0;
    private double ang = 0;
    private ArmSubsystem arm;
    private ElapsedTime timer;


    @Override
    public void runOpMode() {
        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using a webcam. Note that you will need to
         * make sure you have added the webcam to your configuration file and
         * adjusted the name here to match what you named it in said config file.
         *
         * We pass it the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */

        // OR...  Do Not Activate the Camera Monitor View
        //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        follower = new Follower(hardwareMap);

        autoManager = new AutoManagerPedro(this, follower, () -> {
            arm.update(opModeIsActive());
            intake.update(opModeIsActive());
        }, arm, intake, robot);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        detectionPipe = new SampleDetectionPipeline();

        webcam.setPipeline(detectionPipe);

        webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT, OpenCvWebcam.StreamFormat.MJPEG);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        FtcDashboard.getInstance().startCameraStream(webcam, 90);

        telemetry.addLine("Waiting for start");
        telemetry.update();

        robot = new HWProfile();
        robot.init(hardwareMap, false, false);
        intake = new IntakeSubsystem(robot, this, new Params());

        /*
         * Wait for the user to press start on the Driver Station
         */

//        arm = new ArmSubsystem(robot, this, new Params());
//        arm.setAutoMode(true);
//        arm.setPedroAuto(true);
//        arm.setArmPower(1);
//        arm.setSlidesMultiplier(1);
//
//        arm.setTeleopMode(TeleopMode.INTAKE);
//        arm.setIntakePosition(new Params().PEDRO_AUTO_INTAKE_Y1_POS);
//        arm.intakeUpMode();
//        arm.update(opModeIsActive());

        while (opModeInInit()) {
//            arm.update(opModeIsActive());
        }

        waitForStart();

        timer = new ElapsedTime();

        follower.startTeleopDrive();

        while (opModeIsActive()) {
//            arm.update(opModeIsActive());
//            intake.outtake();
//            follower.update();

            /*
             * Send some stats to the telemetry
             */
//            telemetry.addData("Start X: ", startX);
//            telemetry.addData("Robot X: ", drive.pose.position.x);
//            telemetry.addData("Robot Y: ", drive.pose.position.y);
//            telemetry.addData("Robot Heading: ", Math.toDegrees(drive.pose.heading.toDouble()));
//            telemetry.addData("Frame Count", webcam.getFrameCount());
//            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));

            drivePid.setPID(kP, kI, kD);

            telemetry.addData("robot x: ", follower.getPose().getX());
            telemetry.addData("robot y: ", follower.getPose().getY());

            ArrayList<SampleDetectionPipeline.AnalyzedStone> stones = detectionPipe.getDetectedStones();

            if(!stones.isEmpty()) {
                SampleDetectionPipeline.AnalyzedStone stone = stones.get(0);

                telemetry.addData("x: ", stone.x);
                telemetry.addData("y: ", stone.y);
            }

            telemetry.update();

//            safeWait(100);
        }
    }

    public void safeWait(int time) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (timer.time(TimeUnit.MILLISECONDS) <= time) {
            follower.update();
            arm.update(opModeIsActive());
            intake.update(opModeIsActive());

            if (isStopRequested()) break;
            if (!opModeIsActive()) break;
        }
    }
}