package org.firstinspires.ftc.teamcode.AutoPedro;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Enums.AnimationType;
import org.firstinspires.ftc.teamcode.Enums.AutoLocation;
import org.firstinspires.ftc.teamcode.Enums.GrabAngle;
import org.firstinspires.ftc.teamcode.Enums.GrabStyle;
import org.firstinspires.ftc.teamcode.Enums.TeleopMode;
import org.firstinspires.ftc.teamcode.Enums.WristAngle;
import org.firstinspires.ftc.teamcode.Hardware.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Hardware.Params;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;

import java.util.concurrent.TimeUnit;

@Config
@Autonomous(name = "0+4 V1 Pedro", group = "0", preselectTeleOp = "0: Main TeleOp")
public class V1_0_4_Pedro extends LinearOpMode {
    private Follower follower;
    private ElapsedTime autoTime = new ElapsedTime();
    private int autoState = 1;
    private HWProfile robot;
    private Params params;
    private ArmSubsystem arm;
    private IntakeSubsystem intake;
    private TeleopMode currentMode = TeleopMode.IDLE;
    private ElapsedTime time = new ElapsedTime();
    private GrabAngle grabAngle = GrabAngle.VERTICAL_GRAB;
    private GrabStyle grabStyle = GrabStyle.OUTSIDE_GRAB;
    private WristAngle wristAngle = WristAngle.SPECIMEN_SCORE_1;
    private AutoManagerPedro autoManager;
    private boolean resetSlides = false;
    private Thread armHandlerThread = new Thread(() -> {
        while (opModeIsActive()) {
//            arm.update(opModeIsActive());
//            intake.update(opModeIsActive());
//            autoManager.update(opModeIsActive());

            if(gamepad1.right_bumper) {
                intake.toggle();
            }

            telemetry.addData("X:", follower.getPose().getX());
            telemetry.addData("Y:", follower.getPose().getY());
            telemetry.addData("heading:", Math.toDegrees(follower.getTotalHeading()));
            telemetry.addData("autoManager state:", autoState);
            telemetry.update();
        }
    });
    public static boolean debug = false;
    private boolean pathStarted = true;
    private PathChain currentPath;
    private int lastAutoState = 0;
    private AutoLocation autoLocation = AutoLocation.PEDRO_LEFT_0_4_V1;
    private int nextAutoState = 0;
    private Telemetry telemetryA;
    private Pose startPose;
    private boolean runPushForever = false;
    private boolean lbCooldown = false;

    public V1_0_4_Pedro() {
    }

    public void runOpMode() throws InterruptedException {
        follower = new Follower(hardwareMap);
        follower.resetIMU();
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
        intake.intake();

//        intake.update(!isStopRequested());
        arm.update(!isStopRequested());

        autoManager = new AutoManagerPedro(this, follower, () -> {
            arm.update(opModeIsActive());
            if(opModeIsActive()) {
                intake.update(opModeIsActive());
            }
            telemetryA.addData("x: ", follower.getPose().getX());
            telemetryA.addData("y: ", follower.getPose().getY());
            telemetryA.update();
        }, arm, intake, robot);
        autoManager.buildPaths(autoLocation);

        startPose = autoManager.start_0_4_V1;

        follower.setPose(startPose);

        currentMode = TeleopMode.SPECIMEN_SCORE;
        arm.setTeleopMode(currentMode);
        arm.setAnimationType(AnimationType.NONE);
        intake.intake();

        autoManager.safeSleep(350);

//        arm.setArmPower(0);
        arm.setSlidesMultiplier(0);

        while (!opModeIsActive()) {
            arm.update(opModeIsActive());
//            intake.update(opModeIsActive());

            arm.setArmPositionSpecimen(params.ARM_INSPECTION_POS);
            arm.setSlidesPositionSpecimen(0);

            arm.update(opModeIsActive());

            telemetry.addLine("The robot must be started on the fourth tile");
            telemetry.addLine("to the right from the buckets, the robot must be");
            telemetry.addLine("started on the left line of the tile, and the robot");
            telemetry.addLine("needs to be facing towards the submersible");
            telemetry.addLine();
            telemetry.addData("slides raw enc:", robot.slidesMotor1.getCurrentPosition());
            follower.setPose(startPose);
            telemetry.addData("x: ", follower.getPose().getX());
            telemetry.addData("y: ", follower.getPose().getY());
            telemetry.update();

            if(gamepad1.left_bumper && !lbCooldown) {
                intake.toggle();

                lbCooldown = true;
            } else if (!gamepad1.left_bumper) lbCooldown = false;

            if (gamepad1.right_trigger > .1) {
                arm.setSlidesMultiplier(0);
                robot.slidesMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                resetSlides = true;
            } else if (gamepad1.right_trigger < .1 && resetSlides) {
                arm.resetSlidesPosition();
                robot.slidesMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                arm.setSlidesMultiplier(1);

                resetSlides = false;
            }
        }

        waitForStart();

        arm.setSlidesMultiplier(1);

//        requestOpModeStop();

        intake.update(opModeIsActive());

        follower.updatePose();
        follower.update();
        autoManager.safeSleep(100);
        follower.setPose(startPose);
        autoManager.safeSleep(100);

        params.AUTO_SCORE = 37;
        params.AUTO_END_HEADING = 0;

        time.reset();
        arm.setArmPower(.6);

        autoManager.setSpeed(params.AUTO_DEFAULT_SPEED);

//        follower.setPose(new Pose(robot.distanceOne.getDistance(DistanceUnit.INCH) + .5, autoManager.start_3_1_V1.getY(), autoManager.start_3_1_V1.getHeading()));
//        follower.updatePose();

//        armHandlerThread.start();
//        autoManager.runPath(autoManager.toBucketPath, false);

//        currentMode = TeleopMode.BUCKET_SCORE;
//        arm.setTeleopMode(currentMode);
//        arm.setBucket(2);
//        autoManager.safeSleep(750);

        if (debug) {
            autoManager.safeSleep(1000000000);
        } else {

            params.TELEOP_START_MODE = TeleopMode.IDLE;
            params.AUTO_END_HEADING = 180;

//        bucketScore(1);
            specimenScore(1);
            while (opModeIsActive() && !isStopRequested()) {
                push();

                if(!runPushForever) break;
            }
            intakeSpec(1);
            specimenScore(2);
            intakeSpec(2);
            specimenScore(3);
            intakeSpec(3);
            specimenScore(4);
            intakeSpec(4);
            specimenScore(5);
            park();
            telemetryA.addData("time: ", time.time(TimeUnit.SECONDS));

            params.TELEOP_START_MODE = TeleopMode.IDLE;
            params.AUTO_END_HEADING = Math.toDegrees(follower.getPose().getHeading());

            telemetryA.update();
            follower.breakFollowing();
            autoManager.safeSleep(10000);
        }
    }

    public void push() {
        currentMode = TeleopMode.INTAKE;

        wristAngle = WristAngle.SPECIMEN_INTAKE;
        intake.setWristAngle(wristAngle);

        autoManager.setSpeed(1);
        autoManager.runPath(autoManager.pushSamples, false);
        autoManager.setSpeed(params.AUTO_DEFAULT_SPEED);
    }

    public void specimenScore(int specNum) {
        autoManager.setSpeed(1);

        currentMode = TeleopMode.SPECIMEN_SCORE;
//        arm.setTeleopMode(currentMode);
//        arm.setSlidesMultiplier(4);
        arm.update(opModeIsActive());

        wristAngle = WristAngle.SPECIMEN_SCORE_1;
        intake.setWristAngle(wristAngle);
        intake.setGrabAngle(GrabAngle.VERTICAL_GRAB);

        if (specNum == 1) {
            autoManager.runPath(autoManager.specScore1);
        } else if (specNum == 2) {
            autoManager.runPath(autoManager.specScore2);
        } else if (specNum == 3) {
            autoManager.runPath(autoManager.specScore3);
        } else if (specNum == 4) {
            autoManager.runPath(autoManager.specScore4);
        } else if (specNum == 5) {
            autoManager.runPath(autoManager.specScore5);
        }

        autoManager.setSpeed(1);
//        autoManager.waitForArmAndSlides(1000);
        autoManager.safeSleep(50);
        autoManager.setSpeed(params.AUTO_DEFAULT_SPEED);

        arm.setSlidesMultiplier(1);

        arm.setArmPositionSpecimen(params.ARM_SPECIMEN_POLE_2_START); //params.ARM_SCORE_SPECIMEN
        arm.setSlidesPositionSpecimen(params.SLIDES_SPECIMEN_POLE_2_SCORE_CLAW);
        wristAngle = WristAngle.SPECIMEN_SCORE_2;
        intake.setWristAngle(wristAngle);
        autoManager.waitForSlides(params.SLIDES_SPECIMEN_POLE_2_SCORE_CLAW, 5.5);
        autoManager.safeSleep(50);

//        arm.setSlidesPositionSpecimen(params.SLIDES_ENSURE_SCORE_SPECIMEN);
//        autoManager.safeSleep(350);

        if (specNum != 3) {
            intake.outtake();
            intake.update(opModeIsActive());
            autoManager.safeSleep(50);
            arm.setSlidesPositionSpecimen(7);
            autoManager.safeSleep(50);
        }
    }

    public void park() {
//        autoManager.safeSleep(250);
        arm.setTeleopMode(TeleopMode.IDLE);
        intake.setWristAngle(WristAngle.DOWN);
//        autoManager.setSpeed(1);
//        autoManager.runPath(autoManager.park, false);
//        arm.update(opModeIsActive());
//        autoManager.safeSleep(750);
//        arm.setParkArmUp(false);
//        autoManager.safeSleep(10000);
    }

    public void intakeSpec(int specNum) {
        if (specNum != 3) {
            intake.setShortRange(true);
            currentMode = TeleopMode.INTAKE;
            arm.setTeleopMode(currentMode);
            arm.setAnimationType(AnimationType.NONE);
//            arm.setArmPositionSpecimen(34);

            wristAngle = WristAngle.SPECIMEN_INTAKE;
            intake.setWristAngle(wristAngle);
            intake.setGrabAngle(GrabAngle.VERTICAL_GRAB);

            arm.update(opModeIsActive());
            intake.update(opModeIsActive());
        } else {
            arm.setSlidesPositionSpecimen(params.SLIDES_SPECIMEN_POLE_2_START + 2);
            arm.update(opModeIsActive());
        }

        autoManager.setSpeed(1);

        if (specNum == 1) {
//            autoManager.runPath(autoManager.intakeSpec1, true);
        } else if (specNum == 2) {
            autoManager.runPath(autoManager.intakeSpec2, true);
        } else if (specNum == 3) {
            autoManager.runPath(autoManager.intakeSpec3, true);
        } else if (specNum == 4) {
            autoManager.runPath(autoManager.intakeSpec4, true);
        }
//        arm.setArmPositionSpecimen(params.ARM_AUTO_SPECIMEN_INTAKE);
//        autoManager.safeSleep(150);
        if(specNum == 1) {
            autoManager.setSpeed(.4);
        } else {
            autoManager.setSpeed(.5);
        }

        autoManager.holdPoint(new Pose(-5, follower.getPose().getY(), Math.toRadians(180)));

//        autoManager.useDistance(true);
        if(specNum == 1) {
            autoManager.safeSleep(500);
        } else {
            autoManager.safeSleep(300);
        }

        autoManager.setSpeed(0);
//        arm.setArmPositionSpecimen(params.ARM_AUTO_SPECIMEN_INTAKE - 1.5);
//        arm.update(opModeIsActive());
//        intake.setWristAngle(WristAngle.CUSTOM);
//        intake.setCustomWristAngle(40);
//        intake.update(opModeIsActive());

        autoManager.safeSleep(150);

//        intake.setWristAngle(WristAngle.CUSTOM);
//        intake.setCustomWristAngle(60);
//        intake.update(opModeIsActive());
//
//        autoManager.safeSleep(300);


//        autoManager.holdPoint(new Pose(follower.getPose().getX() - .1, follower.getPose().getY(), Math.toRadians(180)));
//        autoManager.setSpeed(.1);
//        autoManager.safeSleep(200);
//        autoManager.useDistance(false);

        intake.intake();
        autoManager.safeSleep(150);

        autoManager.setSpeed(1);
//        follower.setPose(autoManager.getOldPose());

        currentMode = TeleopMode.SPECIMEN_SCORE;
        arm.setTeleopMode(currentMode);
        wristAngle = WristAngle.SPECIMEN_SCORE_1;
        autoManager.safeSleep(50);
        intake.setWristAngle(wristAngle);
//        autoManager.waitForArmAndSlides(750);
        autoManager.setSpeed(params.AUTO_DEFAULT_SPEED);
    }
}
