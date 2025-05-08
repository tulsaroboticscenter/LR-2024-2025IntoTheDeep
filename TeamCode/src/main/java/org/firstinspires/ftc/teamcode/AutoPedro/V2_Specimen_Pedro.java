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
import org.firstinspires.ftc.teamcode.Hardware.SpecimenArm;
import org.firstinspires.ftc.teamcode.bedroBathing.follower.Follower;
import org.firstinspires.ftc.teamcode.bedroBathing.localization.Pose;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.Point;

import java.util.HashMap;
import java.util.HashSet;
import java.util.concurrent.TimeUnit;

@Config
@Autonomous(name = "Specimen V2 Pedro", group = "0", preselectTeleOp = "0: Main TeleOp")
public class V2_Specimen_Pedro extends LinearOpMode {
    private Follower follower;
    private ElapsedTime autoTime = new ElapsedTime();
    private int autoState = 1;
    private HWProfile robot;
    private Params params;
    private ArmSubsystem arm;
    private IntakeSubsystem intake;
    private TeleopMode currentMode = TeleopMode.IDLE;
    private ElapsedTime time = new ElapsedTime();
    private ElapsedTime individualMethodTimer = new ElapsedTime();
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

            if (gamepad1.right_bumper) {
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
    private AutoLocation autoLocation = AutoLocation.PEDRO_LEFT_SPECI_V2;
    private int nextAutoState = 0;
    private Telemetry telemetryA;
    private Pose startPose;
    private boolean runPushForever = false;
    private boolean lbCooldown = false;
    private AutoSampleIntakeLib intakeLib;
    private boolean selectingSubSamples = false;
    private int samplesFromSub = 0;
    private boolean g1BCooldown = false;
    private HashMap<String, Long> times = new HashMap<>();
    private SpecimenArm specimenArm;

    public V2_Specimen_Pedro() {
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

        specimenArm = new SpecimenArm(robot);

//        intake.update(!isStopRequested());
        arm.update(!isStopRequested());

        autoManager = new AutoManagerPedro(this, follower, () -> {
            arm.update(opModeIsActive());
            specimenArm.update(opModeIsActive());
            if (opModeIsActive()) {
                intake.update(opModeIsActive());
            }
            telemetryA.addData("x: ", follower.getPose().getX());
            telemetryA.addData("y: ", follower.getPose().getY());
            telemetryA.update();
        }, arm, intake, robot);
        startPose = autoManager.start_Speci_V2;

        follower.setPose(startPose);

        currentMode = TeleopMode.SPECIMEN_SCORE;
        arm.setTeleopMode(currentMode);
        arm.setAnimationType(AnimationType.NONE);
        intake.intake();

        autoManager.safeSleep(350);

//        arm.setArmPower(0);
        arm.setSlidesMultiplier(0);

        intakeLib = new AutoSampleIntakeLib(telemetry, gamepad1);
        specimenArm.setTeleopMode(TeleopMode.IDLE);
        specimenArm.idleArmPowered(true);
        specimenArm.update(true);
        specimenArm.idleIntakeClawClosed(true);

        while (!opModeIsActive()) {
            arm.update(opModeIsActive());
            specimenArm.update(opModeIsActive());
//            intake.update(opModeIsActive());

            arm.setArmPositionSpecimen(params.ARM_INSPECTION_POS_AUTO);
            arm.setSlidesPositionSpecimen(0);

            arm.update(opModeIsActive());

            samplesFromSub = intakeLib.getLocations().size();
            telemetry.addData("Amount of samples to intake from the sub: ", samplesFromSub);
            this.telemetry.addLine();

            if (!selectingSubSamples) {
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

            if (gamepad1.left_bumper && !lbCooldown) {
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

        autoManager.setSpecimenGrabFromSub(samplesFromSub > 0);
        autoManager.buildPaths(autoLocation);

        arm.setSlidesMultiplier(1);

//        requestOpModeStop();

        intake.intake();
        intake.update(opModeIsActive());

        follower.updatePose();
        follower.update();
        autoManager.safeSleep(100);
        follower.setPose(startPose);
        autoManager.safeSleep(100);

        params.AUTO_SCORE = 37;

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

//        bucketScore(1);
            individualMethodTimer.reset();
            specimenScore(1);
            times.put("specScore1", individualMethodTimer.time(TimeUnit.MILLISECONDS));

            if (samplesFromSub > 0) {
                individualMethodTimer.reset();
//                backupFromScore();
                times.put("backup", individualMethodTimer.time(TimeUnit.MILLISECONDS));

                HashSet<Pose> locations = intakeLib.getLocations();

                for (int i = 0; i < locations.size(); i++) {
                    Pose location = (Pose) locations.toArray()[i];

                    individualMethodTimer.reset();
                    intakeFromSub(location);
                    times.put("intakeFromSub" + i, individualMethodTimer.time(TimeUnit.MILLISECONDS));

                    individualMethodTimer.reset();
                    depositToHP();
                    times.put("depositToHp" + i, individualMethodTimer.time(TimeUnit.MILLISECONDS));
                }
            }
////            pushNew();
            individualMethodTimer.reset();
            pushOld();
            times.put("pushTime", individualMethodTimer.time(TimeUnit.MILLISECONDS));
//
            individualMethodTimer.reset();
            intakeSpec(1);
            specimenScore(2);
            intakeSpec(2);
            specimenScore(3);
            intakeSpec(3);
            specimenScore(4);
            intakeSpec(4);
            specimenScore(5);

            if (samplesFromSub > 0) {
                HashSet<Pose> locations = intakeLib.getLocations();

                int specCount = 5;

                for (int i = 0; i < locations.size(); i++) {
                    intakeSpec(specCount);
                    specimenScore(specCount + 1);

                    specCount++;
                }
            }
            park();
            times.put("scoring", individualMethodTimer.time(TimeUnit.MILLISECONDS));

            telemetryA.addData("time: ", time.time(TimeUnit.SECONDS) + 1);

            times.forEach((String methodName, Long time) -> {
                telemetryA.addData(methodName + ": ", time);
            });

            telemetryA.update();

            params.TELEOP_START_MODE = TeleopMode.IDLE;

            telemetryA.update();
            follower.breakFollowing();
            autoManager.safeSleep(10000);
        }
    }

    public void pushOld() {
//        currentMode = TeleopMode.INTAKE;

        arm.setTeleopMode(TeleopMode.CUSTOM_POSITION);
        arm.setArmCustomPosition(65);
        arm.setSlidesCustomPosition(4);
        arm.update(opModeIsActive());

        specimenArm.setTeleopMode(TeleopMode.INTAKE);
        specimenArm.closeClaw();
        specimenArm.update(opModeIsActive());

        wristAngle = WristAngle.SPECIMEN_INTAKE;
        grabAngle = GrabAngle.VERTICAL_GRAB;
        intake.setWristAngle(wristAngle);
        intake.setGrabAngle(grabAngle);
        intake.update(opModeIsActive());

        autoManager.setSpeed(1);
        autoManager.runPath(autoManager.pushSamples, true);
        autoManager.setSpeed(params.AUTO_DEFAULT_SPEED);
    }

    public void pushNew() {
        currentMode = TeleopMode.INTAKE;

        wristAngle = WristAngle.DOWN;
        grabAngle = GrabAngle.HORIZONTAL_GRAB;
        intake.setWristAngle(wristAngle);
        intake.setGrabAngle(grabAngle);
        intake.update(opModeIsActive());

        arm.setTeleopMode(currentMode);
        arm.setIntakePosition(params.INTAKE_MAX_POS);

        for (int i = 0; i < 3; i++) {
            arm.intakeUpMode();
            arm.update(opModeIsActive());

            autoManager.setSpeed(1);

            if(i == 0) {
                autoManager.runPath(autoManager.newPushSample1P1);
            } else if(i == 1) {
                autoManager.runPath(autoManager.newPushSample2P1);
            } else {
                autoManager.runPath(autoManager.newPushSample3P1);
            }

            autoManager.safeSleep(400);

            arm.intakeDownMode();
            arm.update(opModeIsActive());

            intake.intake();
            intake.update(opModeIsActive());

            autoManager.safeSleep(250);

            arm.intakeUpMode();
            arm.update(opModeIsActive());

            if(i == 0) {
                autoManager.runPath(autoManager.newPushSample1P2);
            } else if(i == 1) {
                autoManager.runPath(autoManager.newPushSample2P2);
            } else {
                autoManager.runPath(autoManager.newPushSample3P2);
            }

            autoManager.safeSleep(100);

            arm.intakeUpMode();
            arm.update(opModeIsActive());

            intake.outtake();
            intake.update(opModeIsActive());
        }

        arm.intakeSpecimen = true;
        arm.setIntakePosition(0);
        intake.setWristAngle(WristAngle.SPECIMEN_INTAKE);
        intake.setGrabAngle(GrabAngle.VERTICAL_GRAB);

        arm.update(opModeIsActive());
        intake.update(opModeIsActive());
    }

    public void backupFromScore() {
        autoManager.setSpeed(1);
        autoManager.runPath(autoManager.backupFromSub);
    }

    public void specimenScore(int specNum) {
        autoManager.setSpeed(1);

//        arm.setTeleopMode(currentMode);
//        arm.setSlidesMultiplier(4);
        arm.update(opModeIsActive());

        if (specNum == 1 && !intakeLib.getLocations().isEmpty()) {
            wristAngle = WristAngle.DOWN;
            intake.outtake();
//            wristAngle = WristAngle.SPECIMEN_SCORE_FRONT;
        } else {
            wristAngle = WristAngle.BUCKET_SCORE;
//            wristAngle = WristAngle.SPECIMEN_SCORE_1;
        }
        intake.setWristAngle(wristAngle);
        intake.setGrabAngle(GrabAngle.VERTICAL_GRAB);

        specimenArm.setTeleopMode(TeleopMode.SPECIMEN_SCORE);
        specimenArm.update(opModeIsActive());

        if(specNum == 1) {
            arm.setTeleopMode(TeleopMode.INTAKE);
            arm.setAnimationType(AnimationType.NONE);
            arm.setIntakePosition(params.INTAKE_MAX_POS);
            arm.intakeUpMode();
            arm.update(opModeIsActive());

            autoManager.safeSleep(150);
        } else {

        }

//        autoManager.setSlowDown(.8, .1);

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
        } else if (specNum == 6) {
            autoManager.runPath(autoManager.specScore6);
        }

//        autoManager.setSlowDown(1, 1);

        autoManager.setSpeed(1);
//        autoManager.waitForArmAndSlides(1000);
        autoManager.safeSleep(50);
        autoManager.setSpeed(params.AUTO_DEFAULT_SPEED);

        arm.setSlidesMultiplier(1);

        if (specNum != 1) {
//            arm.setArmPositionSpecimen(params.ARM_SPECIMEN_POLE_2_START); //params.ARM_SCORE_SPECIMEN
//            arm.setSlidesPositionSpecimen(params.SLIDES_SPECIMEN_POLE_2_SCORE_CLAW);
//            wristAngle = WristAngle.SPECIMEN_SCORE_2;
//            intake.setWristAngle(wristAngle);
//            autoManager.waitForSlides(params.SLIDES_SPECIMEN_POLE_2_SCORE_CLAW, 5.5);
//            autoManager.safeSleep(50);
        }

//        arm.setSlidesPositionSpecimen(params.SLIDES_ENSURE_SCORE_SPECIMEN);
//        autoManager.safeSleep(350);

//        if (specNum != 3) {
//        intake.outtake();
//        intake.update(opModeIsActive());
//        autoManager.safeSleep(50);
//        arm.setSlidesPositionSpecimen(7);
//        autoManager.safeSleep(50);
        specimenArm.openClaw();

        if(specNum != 1) {
            specimenArm.setTeleopMode(TeleopMode.INTAKE);
            specimenArm.openClaw();
            specimenArm.update(opModeIsActive());
        }
//        }
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

    public void depositToHP() {
        autoManager.runPath(autoManager.depositToHP);

        intake.outtake();
        intake.update(opModeIsActive());
    }

    public void intakeFromSub(Pose sampleLocation) {
        intake.setGrabAngle(GrabAngle.CUSTOM);
        intake.setCustomPivotAngle(Math.toDegrees(sampleLocation.getHeading()));
        intake.update(opModeIsActive());

        // calc grab location
        Point grabPoint = new Point(autoManager.sampleGrabOffsetSpecimen.getX() + sampleLocation.getY(), autoManager.sampleGrabOffsetSpecimen.getY() - sampleLocation.getX(), Point.CARTESIAN);

        PathBuilder path = new PathBuilder()
                .addPath(new Path(
                        new BezierLine(
                                new Point(follower.getPose().getX(), follower.getPose().getY()),
                                grabPoint
                        )
                ))
                .setZeroPowerAccelerationMultiplier(3)
                .setPathEndVelocityConstraint(1)
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), Math.toRadians(0));

        // run path
        autoManager.runPath(path, true);
        autoManager.safeSleep(250);

        // intake
        arm.intakeDownMode();
        autoManager.safeSleep(250);
        intake.intake();

        autoManager.safeSleep(150);

        // get ready to score
        arm.intakeUpMode();
        arm.setIntakePosition(0);
        intake.setWristAngle(WristAngle.BUCKET_SCORE);
        intake.setGrabAngle(GrabAngle.INVERTED);
        arm.update(opModeIsActive());
        intake.update(opModeIsActive());

        autoManager.safeSleep(250);

        autoManager.setSpeed(1);
        autoManager.buildPaths(AutoLocation.PEDRO_LEFT_SAMPLE_V1); // pedro bug, workaround
    }

    public void intakeSpec(int specNum) {
//        if (specNum != 3) {
//        intake.setShortRange(true);
//        currentMode = TeleopMode.INTAKE;
//        arm.setTeleopMode(currentMode);
//        arm.setAnimationType(AnimationType.NONE);
//            arm.setArmPositionSpecimen(34);

//        wristAngle = WristAngle.SPECIMEN_INTAKE;
//        intake.setWristAngle(wristAngle);
//        intake.setGrabAngle(GrabAngle.VERTICAL_GRAB);

//        arm.update(opModeIsActive());
//        intake.update(opModeIsActive());
//        } else {
//            arm.setSlidesPositionSpecimen(params.SLIDES_SPECIMEN_POLE_2_START + 3.5);
//            arm.update(opModeIsActive());
//        }

        specimenArm.openClaw();

        autoManager.setSpeed(1);

        if (specNum == 1) {
            autoManager.runPath(autoManager.intakeSpec1, true);
        } else if (specNum == 2) {
            autoManager.runPath(autoManager.intakeSpec2, false);
        } else if (specNum == 3) {
            autoManager.runPath(autoManager.intakeSpec3, false);
        } else if (specNum == 4) {
            autoManager.runPath(autoManager.intakeSpec4, false);
        } else if (specNum == 5) {
            autoManager.runPath(autoManager.intakeSpec4, false);
        }
//        arm.setArmPositionSpecimen(params.ARM_AUTO_SPECIMEN_INTAKE);
//        autoManager.safeSleep(150);
        if (specNum == 1) {
            autoManager.setSpeed(1);
        } else {
            autoManager.setSpeed(.55);
        }

//        autoManager.holdPoint(new Pose(-5, follower.getPose().getY(), Math.toRadians(180)));
//        autoManager.waitUntilStop(4, 10000);
//        follower.breakFollowing();
        autoManager.setSpeed(0);
//        arm.setArmPositionSpecimen(params.ARM_AUTO_SPECIMEN_INTAKE - 1.5);
//        arm.update(opModeIsActive());
//        intake.setWristAngle(WristAngle.CUSTOM);
//        intake.setCustomWristAngle(40);
//        intake.update(opModeIsActive());

//        autoManager.safeSleep(150);

//        intake.setWristAngle(WristAngle.CUSTOM);
//        intake.setCustomWristAngle(60);
//        intake.update(opModeIsActive());
//
//        autoManager.safeSleep(300);


//        autoManager.holdPoint(new Pose(follower.getPose().getX() - .1, follower.getPose().getY(), Math.toRadians(180)));
//        autoManager.setSpeed(.1);
//        autoManager.safeSleep(200);
//        autoManager.useDistance(false);

        specimenArm.closeClaw();
        autoManager.safeSleep(150);

        autoManager.setSpeed(1);
//        follower.setPose(autoManager.getOldPose());

//        currentMode = TeleopMode.SPECIMEN_SCORE;
//        arm.setTeleopMode(currentMode);
//        arm.setIntakePosition(0);
//        wristAngle = WristAngle.SPECIMEN_SCORE_1;
//        autoManager.safeSleep(50);
//        intake.setWristAngle(wristAngle);
//        autoManager.waitForArmAndSlides(750);
        autoManager.setSpeed(1);
    }
}
