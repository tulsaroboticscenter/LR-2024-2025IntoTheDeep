package org.firstinspires.ftc.teamcode.AutoPedro;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

/**
 * This is the StraightBackAndForth autonomous OpMode. It runs the robot in a specified distance
 * straight forward. On reaching the end of the forward Path, the robot runs the backward Path the
 * same distance back to the start. Rinse and repeat! This is good for testing a variety of Vectors,
 * like the drive Vector, the translational Vector, and the heading Vector. Remember to test your
 * tunings on CurvedBackAndForth as well, since tunings that work well for straight lines might
 * have issues going in curves.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */
@Config
@Autonomous (name = "Park Auto", group = "Autonomous")
public class ParkAuto extends LinearOpMode {
    private boolean forward = true;

    private Follower follower;

    private Path parkPath;
    private ElapsedTime autoTime = new ElapsedTime();

    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void runOpMode() {
        follower = new Follower(hardwareMap);
//        follower.resetIMU();
        follower.setPose(new Pose(9, 83.5, Math.toRadians(0)));
        telemetry.update();


        parkPath = new Path(new BezierLine(
                new Point(8.991, 83.674, Point.CARTESIAN),
                new Point(10.151, 10, Point.CARTESIAN)
        ));

        parkPath.setConstantHeadingInterpolation(Math.toRadians(0));

        waitForStart();
        autoTime.reset();

        while (opModeIsActive()) {


            follower.update();
            follower.followPath(parkPath);
            if (!follower.isBusy()) {
                requestOpModeStop();
            }
            follower.telemetryDebug(telemetry);
        }
    }
}
