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
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.Point;

import java.util.HashMap;
import java.util.HashSet;
import java.util.concurrent.TimeUnit;

@Config
@Autonomous(name = "Specimen V3 Pedro", group = "0", preselectTeleOp = "0: Main TeleOp")
public class V3_Specimen_Pedro extends LinearOpMode {
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
    private boolean resetSlides = false;
    public static boolean debug = false;
    private boolean pathStarted = true;
    private PathChain currentPath;
    private int lastAutoState = 0;
    private AutoLocation autoLocation = AutoLocation.PEDRO_LEFT_SPECI_V2;
    private int nextAutoState = 0;
    private Telemetry telemetryA;
    private double depositHeading = -100;
    private Pose startPose = new Pose(0, -47.5, Math.toRadians(0));
    private boolean runPushForever = false;
    private boolean lbCooldown = false;
    private AutoSampleIntakeLib intakeLib;
    private boolean selectingSubSamples = false;
    private int samplesFromSub = 0;
    private boolean g1BCooldown = false;
    private HashMap<String, Long> times = new HashMap<>();
    private SpecimenArm specimenArm;
    private Runnable updates;
    private Pose sampleGrabOffset = new Pose(12, -22);

    public V3_Specimen_Pedro() {
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

        intake.setWristAngle(WristAngle.BUCKET_SCORE);
        intake.setGrabAngle(GrabAngle.VERTICAL_GRAB);
        intake.setGrabStyle(GrabStyle.OUTSIDE_GRAB);
        intake.intake();

        specimenArm = new SpecimenArm(robot);
        arm.update(!isStopRequested());

        follower.setPose(startPose);

        currentMode = TeleopMode.SPECIMEN_SCORE;
        arm.setTeleopMode(currentMode);
        arm.setAnimationType(AnimationType.NONE);
        intake.intake();

        updates = () -> {
            arm.update(opModeIsActive());
            intake.update(opModeIsActive());
            specimenArm.update(opModeIsActive());
        };

//        arm.setArmPower(0);
        arm.setSlidesMultiplier(0);

        intakeLib = new AutoSampleIntakeLib(telemetry, gamepad1);
        specimenArm.setTeleopMode(TeleopMode.IDLE);
        specimenArm.idleArmPowered(true);
        specimenArm.update(true);
        specimenArm.idleIntakeClawClosed(true);
        specimenArm.setAutoMode(true);

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

        arm.setSlidesMultiplier(1);

        intake.intake();

        follower.update();
        follower.setPose(startPose);
        follower.update();

        params.AUTO_SCORE = 60;

        time.reset();
        arm.setArmPower(.6);

        follower.setMaxPower(1);
        params.TELEOP_START_MODE = TeleopMode.IDLE;

        specimenScore(1);

        if (samplesFromSub > 0) {
            HashSet<Pose> locations = intakeLib.getLocations();

            for (int i = 0; i < locations.size(); i++) {
                Pose location = (Pose) locations.toArray()[i];

                intakeFromSub(location);
                depositToHP();
            }
        }
////            pushNew();
        pushSamples();
        intakeFromWall(1);
        specimenScore(2);
        intakeFromWall(2);
        specimenScore(3);
        intakeFromWall(3);
        specimenScore(4);
        intakeFromWall(4);
        specimenScore(5);

        if (samplesFromSub > 0) {
            HashSet<Pose> locations = intakeLib.getLocations();

            int specCount = 5;

            for (int i = 0; i < locations.size(); i++) {
                intakeFromWall(specCount);
                specimenScore(specCount + 1);

                specCount++;
            }
        }

        park();

        telemetryA.addData("time taken: ", time.time(TimeUnit.MILLISECONDS));
        telemetryA.update();

        params.TELEOP_START_MODE = TeleopMode.IDLE;

        PedroUtils.safeSleep(follower, 30_000, updates, this);
    }

    public void pushSamples() {
//        currentMode = TeleopMode.INTAKE;

        arm.setTeleopMode(TeleopMode.CUSTOM_POSITION);
        arm.setArmCustomPosition(65);
        arm.setSlidesCustomPosition(4);
        arm.update(opModeIsActive());

        specimenArm.setTeleopMode(TeleopMode.INTAKE);
        specimenArm.closeClaw();
        specimenArm.update(opModeIsActive());

        intake.setWristAngle(WristAngle.SPECIMEN_INTAKE);
        intake.setGrabAngle(GrabAngle.VERTICAL_GRAB);
        intake.update(opModeIsActive());

        follower.setMaxPower(1);

        double sample1Y = -87;
        double sample2Y = -95;
        double sample3Y = -101.5;
        double sampleOffset = 10;
        double stopX = 20;
        double stopS1X = stopX;
        double afterSampleX = 50;
        double midZPM = 3;
        double pushSamplesHeading = 0;
        PathBuilder path = new PathBuilder();

        if (samplesFromSub < 0) {
            path = path.addPath(new Path(
                            new BezierCurve(
                                    new Point(follower.getPose()),
                                    new Point(0, -40, Point.CARTESIAN),
                                    new Point(10, -78, Point.CARTESIAN),
                                    new Point(afterSampleX - 15, sample1Y + sampleOffset, Point.CARTESIAN)
                            )
                    ))
                    .setZeroPowerAccelerationMultiplier(10)
                    .setPathEndTValueConstraint(.9)
                    .setConstantHeadingInterpolation(0);
        } else {
            Point startPoint = new Point(follower.getPose());

            path = path
                    .addPath(new Path(
                            new BezierLine(
                                    startPoint,
                                    new Point(startPoint.getX() + 1, startPoint.getY() + 1)
                            )
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(depositHeading), 0, 0)
                    .addPath(new Path(
                            new BezierCurve(
                                    startPoint,
                                    new Point(10, -78, Point.CARTESIAN),
                                    new Point(afterSampleX - 7, sample1Y + sampleOffset, Point.CARTESIAN)
                            )
                    ))
                    .setZeroPowerAccelerationMultiplier(10)
                    .setPathEndTValueConstraint(.9)
                    .setConstantHeadingInterpolation(0);
        }

        PedroUtils.runPath(follower, (path
                .addPath(new Path(
                        new BezierCurve(
                                new Point(afterSampleX - 7, sample1Y - sampleOffset, Point.CARTESIAN),
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
                        new Point(afterSampleX, -100, Point.CARTESIAN),
                        new Point(afterSampleX, sample3Y, Point.CARTESIAN)
                )))
                .setPathEndVelocityConstraint(80)
                .setConstantHeadingInterpolation(Math.toRadians(pushSamplesHeading))
                .addPath(new Path(new BezierLine(
                        new Point(afterSampleX, sample3Y, Point.CARTESIAN),
                        new Point(8, -103, Point.CARTESIAN)
                )))
                .setZeroPowerAccelerationMultiplier(5)
                .setPathEndVelocityConstraint(2)
                .setConstantHeadingInterpolation(Math.toRadians(pushSamplesHeading))
                .setPathEndTValueConstraint(.9).build()), updates, this);
    }

    public void specimenScore(int specNum) {
        follower.setMaxPower(1);
        arm.update(opModeIsActive());

        if (specNum == 1 && !intakeLib.getLocations().isEmpty()) {
            intake.setWristAngle(WristAngle.DOWN);
            intake.outtake();
        } else {
            intake.setWristAngle(WristAngle.BUCKET_SCORE);
        }
        intake.setGrabAngle(GrabAngle.VERTICAL_GRAB);

        specimenArm.setTeleopMode(TeleopMode.SPECIMEN_SCORE);
        specimenArm.update(opModeIsActive());

        if (specNum == 1) {
            arm.setTeleopMode(TeleopMode.INTAKE);
            arm.setAnimationType(AnimationType.NONE);
            arm.setIntakePosition(params.INTAKE_MAX_POS);
            arm.intakeUpMode();
            arm.update(opModeIsActive());
        }

//        autoManager.setSlowDown(.8, .1);

        double xPos = 35;

        if (specNum == 1) {
            PedroUtils.safeSleep(follower, 50, updates, this);
            PedroUtils.runPath(follower,
                    new PathBuilder()
                            .addPath(new Path(
                                    new BezierCurve(
                                            new Point(follower.getPose()),
                                            new Point(xPos, -35, Point.CARTESIAN)
                                    )
                            ))
                            .setConstantHeadingInterpolation(0)
                            .build(), updates, this
            );
        } else if (specNum == 2) {
            PedroUtils.runPath(follower,
                    new PathBuilder()
                            .addPath(new Path(
                                    new BezierCurve(
                                            new Point(follower.getPose()),
                                            new Point(5, -25, Point.CARTESIAN),
                                            new Point(xPos, -35, Point.CARTESIAN)
                                    )
                            ))
                            .setConstantHeadingInterpolation(0)
                            .build(), updates, this
            );
        } else if (specNum >= 3) {
            PedroUtils.runPath(follower,
                    new PathBuilder()
                            .addPath(new Path(
                                    new BezierLine(
                                            new Point(follower.getPose()),
                                            new Point(xPos, -35, Point.CARTESIAN)
                                    )
                            ))
                            .setConstantHeadingInterpolation(0)
                            .build(), updates, this
            );
        }

//        autoManager.setSlowDown(1, 1);

        PedroUtils.safeSleep(follower, 50, updates, this);

        arm.setSlidesMultiplier(1);
        specimenArm.openClaw();

        if (specNum != 1) {
            specimenArm.setTeleopMode(TeleopMode.INTAKE);
            specimenArm.openClaw();
            specimenArm.update(opModeIsActive());
        }
//        }
    }

    public void park() {
        Point startPoint = new Point(follower.getPose());

        PedroUtils.runPath(follower, new PathBuilder()
                .addPath(new Path(
                        new BezierLine(
                                startPoint,
                                new Point(startPoint.getX() - 10, startPoint.getY())
                        ))
                )
                .setConstantHeadingInterpolation(0)
                .build(), updates, this);
    }

    public void depositToHP() {
        PedroUtils.runPath(follower, new PathBuilder()
                .addPath(new Path(
                        new BezierCurve(
                                new Point(follower.getPose()),
                                new Point(25, -35, Point.CARTESIAN),
                                new Point(2 /*2.75*/, -60/*19.5*/, Point.CARTESIAN)
                        )
                ))
                .addParametricCallback(.5, () -> {
                    arm.setIntakePosition(20);
                })
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(depositHeading), .45)
                .setPathEndTimeoutConstraint(0)
                .setZeroPowerAccelerationMultiplier(5)
                .setPathEndTValueConstraint(.9).build(), updates, this);

        arm.setIntakePosition(0);

        intake.outtake();
        intake.update(opModeIsActive());
    }

    public void intakeFromSub(Pose sampleLocation) {
        intake.setGrabAngle(GrabAngle.CUSTOM);
        intake.setCustomPivotAngle(Math.toDegrees(sampleLocation.getHeading()));
        intake.update(opModeIsActive());

        // calc grab location
        Point grabPoint = new Point(sampleGrabOffset.getX() + sampleLocation.getY(), sampleGrabOffset.getY() - sampleLocation.getX(), Point.CARTESIAN);

        PathChain path = new PathBuilder()
                .addPath(new Path(
                        new BezierLine(
                                new Point(follower.getPose().getX(), follower.getPose().getY()),
                                grabPoint
                        )
                ))
                .setZeroPowerAccelerationMultiplier(3)
                .setPathEndVelocityConstraint(1)
                .addParametricCallback(.5, () -> {
                    arm.intakeDownMode();
                })
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), Math.toRadians(0)).build();

        // run path
        PedroUtils.runPath(follower, path, true, updates, this);
        PedroUtils.safeSleep(follower, 250, updates, this);

//        while (opModeIsActive()) {
//            updates.run();
//            follower.update();
//        }

        // intake
        intake.intake();

        PedroUtils.safeSleep(follower, 150, updates, this);

        // get ready to score
        arm.intakeUpMode();
        arm.setIntakePosition(0);
        intake.setWristAngle(WristAngle.IDLE);
        arm.update(opModeIsActive());
        intake.update(opModeIsActive());

        PedroUtils.safeSleep(follower, 150, updates, this);
    }

    public void intakeFromWall(int specNum) {
        specimenArm.setTeleopMode(TeleopMode.INTAKE);
        specimenArm.openClaw();
        specimenArm.update(opModeIsActive());

        if (specNum == 1) {
            PedroUtils.runPath(follower,
                    new PathBuilder()
                            .addPath(new Path(
                                    new BezierCurve(
                                            new Point(follower.getPose()),
                                            new Point(10, -95, Point.CARTESIAN),
                                            new Point(1.5, -90, Point.CARTESIAN)
                                    )
                            ))
                            .setConstantHeadingInterpolation(0)
                            .build(), updates, this
            );
            PedroUtils.safeSleep(follower, 50, updates, this);
        } else if (specNum >= 2) {
            PedroUtils.runPath(follower,
                    new PathBuilder()
                            .addPath(new Path(
                                    new BezierLine(
                                            new Point(follower.getPose()),
//                                            new Point(35, -35, Point.CARTESIAN),
                                            new Point(1.5, -60, Point.CARTESIAN)
                                    )
                            ))
                            .setConstantHeadingInterpolation(0)
                            .build(), updates, this
            );
            PedroUtils.safeSleep(follower, 50, updates, this);
        }
        specimenArm.closeClaw();
        PedroUtils.safeSleep(follower, 75, updates, this);

        specimenArm.setTeleopMode(TeleopMode.SPECIMEN_SCORE);
        specimenArm.update(opModeIsActive());

        PedroUtils.safeSleep(follower, 100, updates, this);
    }
}
