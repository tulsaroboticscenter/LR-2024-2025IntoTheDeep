package org.firstinspires.ftc.teamcode.AutoPedro;

import static org.firstinspires.ftc.teamcode.Hardware.MecanumDrive.PARAMS;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutoPedro.AutoManagerPedro;
import org.firstinspires.ftc.teamcode.Enums.AutoLocation;
import org.firstinspires.ftc.teamcode.Enums.TeleopMode;
import org.firstinspires.ftc.teamcode.Enums.WristAngle;
import org.firstinspires.ftc.teamcode.Hardware.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.Hardware.Params;
import org.firstinspires.ftc.teamcode.Hardware.SpecimenArm;
import org.firstinspires.ftc.teamcode.bedroBathing.follower.Follower;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.Point;
import org.opencv.core.Mat;

import java.lang.annotation.ElementType;
import java.util.Arrays;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "Pedro Specimen Test", group = "Test")
public class SpecimenTestPedro extends LinearOpMode {
    private HWProfile robot = new HWProfile();
    private Params params = new Params();
    private ArmSubsystem arm;
    private IntakeSubsystem intake;
    private SpecimenArm specArm;
    private AutoManagerPedro autoManager;
    private ElapsedTime autoTimer = new ElapsedTime();
    private int autoTime = 0;
    private Follower follower;

    public void safeSleep(int millis) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (timer.time(TimeUnit.MILLISECONDS) < millis) {
            autoManager.update();

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
            autoManager.runPath(
                    new PathBuilder()
                            .addPath(new Path(
                                    new BezierCurve(
                                            new Point(follower.getPose()),
                                            new Point(35, -45, Point.CARTESIAN),
                                            new Point(1.5, -90, Point.CARTESIAN)
                                    )
                            ))
                            .setConstantHeadingInterpolation(0)
                            .build()
            );
            safeSleep(5_000);
            autoTimer.reset();
        } else if (specNum >= 2) {
            autoManager.runPath(
                    new PathBuilder()
                            .addPath(new Path(
                                    new BezierLine(
                                            new Point(follower.getPose()),
//                                            new Point(35, -35, Point.CARTESIAN),
                                            new Point(1.5, -60, Point.CARTESIAN)
                                    )
                            ))
                            .setConstantHeadingInterpolation(0)
                            .build()
            );
            safeSleep(50);
        }
        specArm.closeClaw();
        safeSleep(250);
    }

    public void specimenScore(int specNum) {
        if(specNum != 10) {
            specArm.setTeleopMode(TeleopMode.SPECIMEN_SCORE);
            specArm.update(opModeIsActive());
            specArm.closeClaw();
        }

        double xPos = 35;

        if (specNum == 1) {
            safeSleep(1000);

            autoManager.runPath(
                    new PathBuilder()
                            .addPath(new Path(
                                    new BezierCurve(
                                            new Point(follower.getPose()),
                                            new Point(xPos, -35, Point.CARTESIAN)
                                    )
                            ))
                            .setConstantHeadingInterpolation(0)
                            .build()
            );
        } else if (specNum == 2) {
            autoManager.runPath(
                    new PathBuilder()
                            .addPath(new Path(
                                    new BezierCurve(
                                            new Point(follower.getPose()),
                                            new Point(5, -25, Point.CARTESIAN),
                                            new Point(xPos, -35, Point.CARTESIAN)
                                    )
                            ))
                            .setConstantHeadingInterpolation(0)
                            .build()
            );
        }  else if (specNum >= 3) {
            autoManager.runPath(
                    new PathBuilder()
                            .addPath(new Path(
                                    new BezierLine(
                                            new Point(follower.getPose()),
                                            new Point(xPos, -35, Point.CARTESIAN)
                                    )
                            ))
                            .setConstantHeadingInterpolation(0)
                            .build()
            );
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
        follower = new Follower(hardwareMap);
        autoManager = new AutoManagerPedro(this, follower, () -> {
            arm.update(opModeIsActive());
            intake.update(opModeIsActive());
            specArm.update(opModeIsActive());
        }, arm, intake, robot);

        autoManager.buildPaths(AutoLocation.PEDRO_LEFT_SPECI_V2);

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
        follower.setPose(autoManager.start_Speci_V2);
        follower.update();
        autoManager.update();

        arm.setArmCustomPosition(60);
        arm.setSlidesCustomPosition(6);
        arm.setSlidesPowerLimit(0);

        specimenScore(1);
        intakeSpec(1);
        specimenScore(2);
//        intakeFromWall(2);
//        specimenScore(3);
//        intakeFromWall(3);
//        specimenScore(4);
//        intakeFromWall(4);
//        specimenScore(5);
//        intakeFromWall(5);
//        specimenScore(6);

        autoTime = (int) autoTimer.time(TimeUnit.MILLISECONDS);
        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Time Taken: ", autoTime);
            telemetry.update();

            intakeSpec(10);
            specimenScore(10);

            arm.update(opModeIsActive());
            intake.update(opModeIsActive());
            specArm.update(opModeIsActive());
        }
    }
}
