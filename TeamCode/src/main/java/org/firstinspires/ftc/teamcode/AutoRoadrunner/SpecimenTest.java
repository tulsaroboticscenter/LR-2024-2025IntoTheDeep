package org.firstinspires.ftc.teamcode.AutoRoadrunner;

import static org.firstinspires.ftc.teamcode.Hardware.MecanumDrive.PARAMS;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Enums.TeleopMode;
import org.firstinspires.ftc.teamcode.Enums.WristAngle;
import org.firstinspires.ftc.teamcode.Hardware.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.Hardware.Params;
import org.firstinspires.ftc.teamcode.Hardware.SpecimenArm;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "Roadrunner Fast Specimen Test", group = "Test")
public class SpecimenTest extends LinearOpMode {
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
                    .setReversed(true)
                    .splineToLinearHeading(new Pose2d(1.5, -90, 0), Math.toRadians(180))
                    .build());
            safeSleep(5_000);
            autoTimer.reset();
        } else if (specNum >= 2) {
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .strafeToLinearHeading(new Vector2d(1, -60 - intakeAndScoreYOffset), Math.toRadians(0), drive.defaultVelConstraint, new ProfileAccelConstraint(-120, PARAMS.maxProfileAccel))
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
        int scoreMinAccel = -110;
        double yPos = -38;

        if (specNum == 1) {
            safeSleep(1000);

            Actions.runBlocking(drive.actionBuilder(drive.pose)
//                    .splineToLinearHeading(new Pose2d(xPos, -25, Math.toRadians(0)), new Rotation2d(Math.toRadians(90), .5))
                    .strafeToLinearHeading(new Vector2d(xPos, -30), Math.toRadians(0), drive.defaultVelConstraint, new ProfileAccelConstraint(-120, PARAMS.maxProfileAccel))
                    .build());
        } else if (specNum == 2) {
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .strafeToLinearHeading(new Vector2d(xPos + 5, -35), 0, drive.defaultVelConstraint, new ProfileAccelConstraint(scoreMinAccel, PARAMS.maxProfileAccel))
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

        specimenScore(1);
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
