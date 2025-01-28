package org.firstinspires.ftc.teamcode.Misc.P2P;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;
import java.util.concurrent.TimeUnit;

@Config
public class P2PDrive {
    // TODO: Tune
    public static double forwardMovementP = 0.11;
    public static double forwardMovementI = 0;
    public static double forwardMovementD = 0.01;

    // TODO: Tune
    public static double strafeMovementP = 0.12;
    public static double strafeMovementI = 0;
    public static double strafeMovementD = 0.02;

    public static double headingP = 0.000005;
    public static double headingI = 0;
    public static double headingD = 0.01;

    private LinearOpMode linearOpMode;
    private Runnable updateRunnable = null;
    private DcMotor motorLF;
    private DcMotor motorLR;
    private DcMotor motorRF;
    private DcMotor motorRR;
    private Localizer localizer;
    private HardwareMap hwMap;
    public Pose2d pose;
    PIDFController forwardMovementController = new PIDFController(forwardMovementP, forwardMovementI, forwardMovementD, 0);
    PIDFController strafeMovementController = new PIDFController(strafeMovementP, strafeMovementI, strafeMovementD, 0);
    PIDFController headingController = new PIDFController(headingP, headingI, headingD, 0);

    public P2PDrive(HardwareMap hwMap, LinearOpMode opMode) {
        this.linearOpMode = opMode;
        this.hwMap = hwMap;
        init();
    }

    public void runEveryUpdate(Runnable updateAction) {
        this.updateRunnable = updateAction;
    }

    private void initMotor(DcMotor motor, boolean reversed) {
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setPower(0);
        if (reversed) {
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
        }
    }

    private void init() {
        localizer = new RRLocalizer(hwMap); //todo: change to your localizer
        localizer.setPose(new Pose2d(0, 0, 0));

        forwardMovementController.setTolerance(.5);
        strafeMovementController.setTolerance(.5);
        headingController.setTolerance(2);

        forwardMovementController.setSetPoint(0);
        strafeMovementController.setSetPoint(0);
        headingController.setSetPoint(0);

        forwardMovementController.calculate(0);
        strafeMovementController.calculate(0);
        headingController.calculate(0);

        motorLF = hwMap.get(DcMotor.class, "motorLF");
        initMotor(motorLF, true);

        motorLR = hwMap.get(DcMotor.class, "motorLR");
        initMotor(motorLR, true);

        motorRF = hwMap.get(DcMotor.class, "motorRF");
        initMotor(motorRF, false);

        motorRR = hwMap.get(DcMotor.class, "motorRR");
        initMotor(motorRR, false);
    }

    public void updateDrive(double y, double x, double rx, double botHeading) {
        botHeading *= -1;

        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        motorLF.setPower(frontLeftPower);
        motorLR.setPower(backLeftPower);
        motorRF.setPower(frontRightPower);
        motorRR.setPower(backRightPower);

        linearOpMode.telemetry.addData("w1: ", frontLeftPower);
        linearOpMode.telemetry.addData("w2: ", backLeftPower);
        linearOpMode.telemetry.addData("w3: ", frontRightPower);
        linearOpMode.telemetry.addData("w4: ", backRightPower);
    }

    private void follow(Path path) {
        forwardMovementController.setSetPoint(path.getXTarget());
        strafeMovementController.setSetPoint(path.getYTarget());
        headingController.setSetPoint(path.getHeadingTarget());

        if (updateRunnable != null) {
            updateRunnable.run();
        }

        localizer.update();
        pose = localizer.getPose();

        forwardMovementController.calculate(pose.getX());
        strafeMovementController.calculate(pose.getY());
        headingController.calculate(pose.getHeading());

        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        HashMap<Double, Runnable> timedRunnables;
        timedRunnables = path.getTimedRunnables();

        while (!forwardMovementController.atSetPoint() || !strafeMovementController.atSetPoint() || !headingController.atSetPoint()) {
            localizer.update();

            updatePose();

            if (!timedRunnables.isEmpty()) {
                for (int i = 0; i < timedRunnables.size(); i++) {
                    double time = (double) timedRunnables.keySet().toArray()[0];

                    if (time <= timer.time(TimeUnit.SECONDS)) {
                        timedRunnables.get(time).run();
                        timedRunnables.remove(time);
                    }
                }
            }

            forwardMovementController.setPIDF(forwardMovementP, forwardMovementI, forwardMovementD, 0);
            strafeMovementController.setPIDF(strafeMovementP, strafeMovementI, strafeMovementD, 0);
            headingController.setPIDF(headingP, headingI, headingD, 0);

            double xOut = forwardMovementController.calculate(pose.getX());
            double yOut = -strafeMovementController.calculate(pose.getY());
            double headingOut = -headingController.calculate(pose.getHeading());

            updateDrive(xOut, yOut, headingOut, Math.toRadians(pose.getHeading()));

            if (linearOpMode.isStopRequested()) break;

            linearOpMode.telemetry.addData("imu: ", pose.getHeading());
            linearOpMode.telemetry.addData("xOut: ", xOut);
            linearOpMode.telemetry.addData("yOut: ", yOut);
            linearOpMode.telemetry.addData("headingOut: ", headingOut);
            linearOpMode.telemetry.update();
        }
    }

    public void followPath(Path path) {
        forwardMovementController.setTolerance(.5);
        strafeMovementController.setTolerance(.5);

        follow(path);
    }

    public void setPose(Pose2d newPose) {
        this.pose = newPose;
        localizer.setPose(newPose);
        localizer.update();
    }

    public void followPathBundle(PathGroup paths) {
        int pathBundleLen = paths.getPaths().length;

        for (int i = 0; i < pathBundleLen; i++) {
            Path path = paths.getPaths()[i];

            if ((i + 1) == pathBundleLen) {
                forwardMovementController.setTolerance(.5);
                strafeMovementController.setTolerance(.5);
            } else {
                forwardMovementController.setTolerance(5);
                strafeMovementController.setTolerance(5);
            }

            follow(path);
        }
    }

    public void updatePose() {
        localizer.update();

        pose = localizer.getPose();
    }
}
