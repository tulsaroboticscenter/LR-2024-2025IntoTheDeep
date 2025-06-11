package org.firstinspires.ftc.teamcode.AutoRoadrunner;

import static org.firstinspires.ftc.teamcode.Hardware.MecanumDrive.PARAMS;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Enums.GrabAngle;
import org.firstinspires.ftc.teamcode.Enums.TeleopMode;
import org.firstinspires.ftc.teamcode.Enums.WristAngle;
import org.firstinspires.ftc.teamcode.Hardware.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.Hardware.Params;
import org.firstinspires.ftc.teamcode.Hardware.SpecimenArm;
import org.opencv.core.Mat;

import java.util.concurrent.TimeUnit;

import kotlin.Unit;

@Autonomous(name = "5-6 Specimen Auto", group = "Auto")
public class RRSpecimenAuto extends LinearOpMode {
    private HWProfile robot = new HWProfile();
    private Params params = new Params();
    private ArmSubsystem arm;
    private IntakeSubsystem intake;
    private SpecimenArm specArm;
    private MecanumDrive drive;
    private ElapsedTime autoTimer = new ElapsedTime();
    private int autoTime = 0;
    private double intakeAndScoreYOffset = 8;

    public void safeSleep(int millis) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (timer.time(TimeUnit.MILLISECONDS) < millis) {
            drive.updatePoseEstimate();

            telemetry.addData("voltage: ", robot.voltageSensor.getVoltage());
            telemetry.update();

            arm.update(opModeIsActive());
            intake.update(opModeIsActive());
            specArm.update(opModeIsActive());
        }
    }

    public void intakeSpec(int specNum) {
        specArm.setTeleopMode(TeleopMode.INTAKE);
        specArm.openClaw();
        specArm.update(opModeIsActive());

        if (specNum == 1) {
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .strafeToLinearHeading(new Vector2d(0, -90), Math.toRadians(0), drive.defaultVelConstraint, new ProfileAccelConstraint(-35, PARAMS.maxProfileAccel))
                    .build());
            safeSleep(50);
        } else if (specNum >= 2) {
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .strafeToLinearHeading(new Vector2d(1, -60 - intakeAndScoreYOffset), Math.toRadians(0), drive.defaultVelConstraint, new ProfileAccelConstraint(-200, PARAMS.maxProfileAccel))
                    .build());
            safeSleep(50);
        }
        specArm.closeClaw();
        safeSleep(250);
    }

    public void specimenScore(int specNum) {
        specArm.setTeleopMode(TeleopMode.SPECIMEN_SCORE);
        specArm.update(opModeIsActive());
        specArm.closeClaw();

        double xPos = 35;
        int scoreMinAccel = -130;
        double yPos = -40;

        if (specNum == 1) {
            safeSleep(1000);

            Actions.runBlocking(drive.actionBuilder(drive.pose)
//                    .splineToLinearHeading(new Pose2d(xPos, -25, Math.toRadians(0)), new Rotation2d(Math.toRadians(90), .5))
                    .strafeToLinearHeading(new Vector2d(xPos - 2, -30), Math.toRadians(0), drive.defaultVelConstraint, new ProfileAccelConstraint(-80, PARAMS.maxProfileAccel))
                    .build());
        } else if (specNum == 2) {
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .strafeToLinearHeading(new Vector2d(xPos + 5, -35), 0, drive.defaultVelConstraint, new ProfileAccelConstraint(-130, PARAMS.maxProfileAccel))
//                    .splineToLinearHeading(new Pose2d(xPos, -25, Math.toRadians(0)), new Rotation2d(Math.toRadians(90), .5))
                    .build());
        } else if (specNum == 3) {
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .strafeToLinearHeading(new Vector2d(xPos, yPos), 0, drive.defaultVelConstraint, new ProfileAccelConstraint(scoreMinAccel, PARAMS.maxProfileAccel))
//                    .splineToLinearHeading(new Pose2d(xPos, -35, Math.toRadians(0)), new Rotation2d(Math.toRadians(90), 0), drive.defaultVelConstraint, new ProfileAccelConstraint(-75, PARAMS.maxProfileAccel))
                    .build());
        } else if (specNum == 4) {
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .strafeToLinearHeading(new Vector2d(xPos, yPos), 0, drive.defaultVelConstraint, new ProfileAccelConstraint(scoreMinAccel, PARAMS.maxProfileAccel))
//                    .splineToLinearHeading(new Pose2d(xPos, -37, Math.toRadians(0)), new Rotation2d(Math.toRadians(90), 0), drive.defaultVelConstraint, new ProfileAccelConstraint(-75, PARAMS.maxProfileAccel))
                    .build());
        } else if (specNum == 5) {
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .strafeToLinearHeading(new Vector2d(xPos - 1.5, yPos), 0, drive.defaultVelConstraint, new ProfileAccelConstraint(scoreMinAccel, PARAMS.maxProfileAccel))
//                    .splineToLinearHeading(new Pose2d(xPos, -37, Math.toRadians(0)), new Rotation2d(Math.toRadians(90), 0), drive.defaultVelConstraint, new ProfileAccelConstraint(-75, PARAMS.maxProfileAccel))
                    .build());
        } else if (specNum == 6) {
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .strafeToLinearHeading(new Vector2d(xPos - 1.5, yPos), 0, drive.defaultVelConstraint, new ProfileAccelConstraint(scoreMinAccel, PARAMS.maxProfileAccel))
//                    .splineToLinearHeading(new Pose2d(xPos, -37, Math.toRadians(0)), new Rotation2d(Math.toRadians(90), 0), drive.defaultVelConstraint, new ProfileAccelConstraint(-75, PARAMS.maxProfileAccel))
                    .build());
        }
        safeSleep(50);
        specArm.openClaw();
    }

    public void pushSamples(boolean afterSubIntake) {
        arm.setTeleopMode(TeleopMode.CUSTOM_POSITION);
        arm.setArmCustomPosition(65);
        arm.setSlidesCustomPosition(4);
        arm.update(opModeIsActive());

        specArm.setTeleopMode(TeleopMode.INTAKE);
        specArm.closeClaw();
        specArm.update(opModeIsActive());

        intake.setWristAngle(WristAngle.DOWN);
        intake.setGrabAngle(GrabAngle.VERTICAL_GRAB);
        intake.update(opModeIsActive());

        TrajectoryActionBuilder builder = drive.actionBuilder(drive.pose);

        if(afterSubIntake) {
            builder = builder
                    .strafeToLinearHeading(new Vector2d(10, -35), Math.toRadians(45));
        } else {
            builder = builder
                    .strafeToLinearHeading(new Vector2d(5, -75), 0);
        }

        double sampleAfterX = 50;
        double sample1Y = -84;
        double sample2Y = -91;
        double sample3Y = -100;
        double sampleBeforeOffset = 10;
        double hpX = 5;
        double endAccel = -130;

        builder = builder
                .strafeToLinearHeading(new Vector2d(sampleAfterX, sample1Y + sampleBeforeOffset), 0, drive.defaultVelConstraint, new ProfileAccelConstraint(endAccel, PARAMS.maxProfileAccel))
                .strafeToLinearHeading(new Vector2d(sampleAfterX, sample1Y), 0, drive.defaultVelConstraint, new ProfileAccelConstraint(endAccel, PARAMS.maxProfileAccel))
                .strafeToLinearHeading(new Vector2d(hpX, sample1Y), 0, drive.defaultVelConstraint, new ProfileAccelConstraint(endAccel, PARAMS.maxProfileAccel))
                .strafeToLinearHeading(new Vector2d(sampleAfterX, sample1Y), 0, drive.defaultVelConstraint, new ProfileAccelConstraint(endAccel, PARAMS.maxProfileAccel))
                .strafeToLinearHeading(new Vector2d(sampleAfterX, sample2Y), 0, drive.defaultVelConstraint, new ProfileAccelConstraint(endAccel, PARAMS.maxProfileAccel))
                .strafeToLinearHeading(new Vector2d(hpX, sample2Y), 0, drive.defaultVelConstraint, new ProfileAccelConstraint(endAccel, PARAMS.maxProfileAccel))
                .strafeToLinearHeading(new Vector2d(sampleAfterX, sample2Y), 0, drive.defaultVelConstraint, new ProfileAccelConstraint(endAccel, PARAMS.maxProfileAccel))
                .strafeToLinearHeading(new Vector2d(sampleAfterX, sample3Y), 0, drive.defaultVelConstraint, new ProfileAccelConstraint(endAccel, PARAMS.maxProfileAccel))
                .strafeToLinearHeading(new Vector2d(10, sample3Y), 0, drive.defaultVelConstraint, new ProfileAccelConstraint(endAccel, PARAMS.maxProfileAccel))
        ;

        Actions.runBlocking(builder.build());
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, false, false);
        arm = new ArmSubsystem(robot, this, params);
        intake = new IntakeSubsystem(robot, this, params);
        specArm = new SpecimenArm(robot);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, -47.5, Math.toRadians(0)), false);

        arm.resetSlidesPosition();
        arm.update(true);

        specArm.setTeleopMode(TeleopMode.IDLE);
        specArm.idleIntakeClawClosed(true);
        specArm.closeClaw();

        arm.setTeleopMode(TeleopMode.CUSTOM_POSITION);
        intake.setWristAngle(WristAngle.BUCKET_SCORE);
        intake.outtake();

        arm.setArmCustomPosition(params.ARM_INSPECTION_POS);
        arm.setSlidesCustomPosition(0);
        arm.setSlidesPowerLimit(0);

        while (opModeInInit()) {
            arm.update(opModeInInit());
//            intake.update(opModeIsActive());
            specArm.update(opModeInInit());
        }

        waitForStart();

        drive.setUpdateAction(new InstantAction(() -> {
            specArm.update(opModeIsActive());
            arm.update(opModeIsActive());
            intake.update(opModeIsActive());

            telemetry.addData("heading: ", Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.addData("x: ", drive.pose.position.x);
            telemetry.addData("y: ", drive.pose.position.y);
            telemetry.update();
        }));

        arm.setArmCustomPosition(60);
        arm.setSlidesCustomPosition(6);
        arm.setSlidesPowerLimit(0);
        autoTimer.reset();

        specimenScore(1);

        pushSamples(false);

        intakeSpec(1);
        specimenScore(2);
        intakeSpec(2);
        specimenScore(3);
        intakeSpec(3);
        specimenScore(4);
        intakeSpec(4);
        specimenScore(5);
        intakeSpec(5);
        specimenScore(6);

        autoTime = (int) autoTimer.time(TimeUnit.MILLISECONDS);
        while (opModeIsActive() && !isStopRequested()) {
            drive.updatePoseEstimate();

            telemetry.addData("Time Taken: ", autoTime);
            telemetry.update();

            arm.update(opModeIsActive());
            intake.update(opModeIsActive());
            specArm.update(opModeIsActive());
        }
    }
}
