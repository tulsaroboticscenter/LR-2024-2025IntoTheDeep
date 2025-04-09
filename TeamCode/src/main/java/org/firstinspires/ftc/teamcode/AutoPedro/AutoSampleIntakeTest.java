package org.firstinspires.ftc.teamcode.AutoPedro;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

import java.util.HashSet;

@Autonomous(name = "Auto Sample Intake Test")
public class AutoSampleIntakeTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Follower follower = new Follower(hardwareMap);
        AutoSampleIntakeLib intakeLib = new AutoSampleIntakeLib(telemetry, gamepad1);
        HWProfile robot = new HWProfile();
        robot.init(hardwareMap, false, false);
        ArmSubsystem arm = new ArmSubsystem(robot, this, new Params());
        IntakeSubsystem intake = new IntakeSubsystem(robot, this, new Params());

        AutoManagerPedro autoManager = new AutoManagerPedro(this, follower, () -> {
            arm.update(opModeIsActive());
            intake.update(opModeIsActive());
        }, arm, intake, robot);

        while (opModeInInit()) {
            intakeLib.displayInfo();
            intakeLib.update();

            arm.update(opModeIsActive());
            intake.update(opModeIsActive());
        }

        waitForStart();

        Pose sampleGrabOffset = new Pose(39.6, 3);
        autoManager.setSpeed(1);
        autoManager.buildPaths(AutoLocation.PEDRO_LEFT_4_0_V1);

        follower.setPose(autoManager.start_4_0_V1);
        follower.setStartingPose(autoManager.start_4_0_V1);

        HashSet<Pose> locations = intakeLib.getLocations();

        follower.updatePose();
        follower.update();
        autoManager.safeSleep(100);
        follower.setPose(autoManager.start_4_0_V1);
        autoManager.safeSleep(100);

        for (int i = 0; i < locations.size(); i++) {
            Pose location = (Pose) locations.toArray()[i];

            arm.intakeUpMode();
            arm.setAutoMode(true);
            arm.setPedroAuto(true);
            arm.setSlidesMultiplier(1);
            arm.setSlidesPowerLimit(1);

            intake.outtake();
            intake.setWristAngle(WristAngle.DOWN);
            intake.setGrabAngle(GrabAngle.CUSTOM);
            intake.setCustomPivotAngle(Math.toDegrees(location.getHeading()));

            arm.update(opModeIsActive());
            intake.update(opModeIsActive());

            Point grabPoint = new Point(sampleGrabOffset.getX() + location.getY(), sampleGrabOffset.getY() - location.getX(), Point.CARTESIAN);

            PathBuilder path = new PathBuilder()
                    .addPath(new Path(
                            new BezierCurve(
                                    new Point(follower.getPose().getX(), follower.getPose().getY()),
                                    new Point(50, 15, Point.CARTESIAN),
                                    grabPoint
                            )
                    ))
                    .addTemporalCallback(.1, () -> {
                        arm.setTeleopMode(TeleopMode.INTAKE);
                        arm.update(opModeIsActive());
                    })
                    .addParametricCallback(.6, () -> {
                        arm.setIntakePosition(Params.INTAKE_MAX_POS); //jank
                    })
                    .setZeroPowerAccelerationMultiplier(1)
                    .setLinearHeadingInterpolation(follower.getPose().getHeading(), Math.toRadians(-90));

            autoManager.runPath(path, true);
            autoManager.safeSleep(750);

            arm.intakeDownMode();
            autoManager.safeSleep(250);
            intake.intake();

            autoManager.safeSleep(250);

            arm.intakeUpMode();
            arm.setIntakePosition(0);
            intake.setWristAngle(WristAngle.BUCKET_SCORE);
            intake.setGrabAngle(GrabAngle.INVERTED);
            arm.update(opModeIsActive());
            intake.update(opModeIsActive());

            autoManager.safeSleep(500);

            autoManager.setSpeed(.75);
            autoManager.buildPaths(AutoLocation.PEDRO_LEFT_4_0_V1); // pedro bug, workaround
            autoManager.runPath(autoManager.bucketScoreFromSubPath);
            autoManager.safeSleep(750);
            intake.outtake();
            autoManager.safeSleep(500);
        }
    }
}
