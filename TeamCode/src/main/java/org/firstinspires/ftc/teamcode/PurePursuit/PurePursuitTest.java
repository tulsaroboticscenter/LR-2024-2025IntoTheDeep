//package org.firstinspires.ftc.teamcode.PurePursuit;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.arcrobotics.ftclib.drivebase.MecanumDrive;
//import com.arcrobotics.ftclib.geometry.Pose2d;
//import com.arcrobotics.ftclib.geometry.Rotation2d;
//import com.arcrobotics.ftclib.geometry.Translation2d;
//import com.arcrobotics.ftclib.hardware.motors.Motor;
//import com.arcrobotics.ftclib.kinematics.DifferentialOdometry;
//import com.arcrobotics.ftclib.kinematics.Odometry;
//import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
//import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
//import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
//import com.arcrobotics.ftclib.purepursuit.Path;
//import com.arcrobotics.ftclib.purepursuit.Waypoint;
//import com.arcrobotics.ftclib.purepursuit.types.PathType;
//import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
//import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
//import com.arcrobotics.ftclib.purepursuit.waypoints.InterruptWaypoint;
//import com.arcrobotics.ftclib.purepursuit.waypoints.PointTurnWaypoint;
//import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
//
//import java.util.concurrent.TimeUnit;
//
//@TeleOp(name = "Pure Pursuit Test", group = "Test")
//@Config
//public class PurePursuitTest extends LinearOpMode {
//    PureTwoWheelOdo odometry;
////    MecanumDriveOdometry odometry;
//    HWProfile robot = new HWProfile();
//    private MecanumDrive mecDrive;
//    public static double followRadius = 1;
//    private static double movementSpeed = 1;
//    private static double radiusSpeed = 1;
//    private ElapsedTime timer;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        robot.init(hardwareMap, true,false);
//
//        odometry = new PureTwoWheelOdo(new Pose2d(new Translation2d(0,0), new Rotation2d(Math.toRadians(0))), hardwareMap, robot.imu);
////        MecanumDriveKinematics kinematics = new MecanumDriveKinematics(new Translation2d(-.25, .25),new Translation2d(.25, .25),new Translation2d(-.25, -.25),new Translation2d(.25, -.25));
////        odometry = new MecanumDriveOdometry(kinematics, new Rotation2d(robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)));
//
////        mecDrive = new MecanumDrive(robot.motorLF, robot.motorRF, robot.motorLR, robot.motorRR);
//        waitForStart();
//
//        new Thread(() -> {
//            while (opModeIsActive()) {
//                Pose2d pose = odometry.getPose();
//
//            }
//        }).start();
//
//        timer = new ElapsedTime();
//        timer.reset();
//
//        while (opModeIsActive()) {
//            odometry.updatePose();
//            Pose2d pose = odometry.getPose();
//
//            if(gamepad1.a) {
//                Waypoint startWaypoint = new StartWaypoint(0, 0);
//                Waypoint p1 = new PointTurnWaypoint(
//                        0, 0, movementSpeed,
//                        radiusSpeed, followRadius,
//                        2, Math.toRadians(5)
//                );
//                Waypoint p2 = new GeneralWaypoint(
//                        97, -4, Math.toRadians(0), movementSpeed,
//                        radiusSpeed, followRadius
//                );
//                Waypoint endPoint = new EndWaypoint(
//                        0, 0, Math.toRadians(90), movementSpeed,
//                        radiusSpeed, followRadius,
//                2, Math.toRadians(5)
//                );
//
//                Path path = new Path(startWaypoint, endPoint);
//                path.setPathType(PathType.WAYPOINT_ORDERING_CONTROLLED);
//                path.init();
////                path.followPath(mecDrive, odometry);
//                while (!path.isFinished() && opModeIsActive()) {
//                    if (path.timedOut())
//                        throw new InterruptedException("Timed out");
//
//                    odometry.updatePose();
//
//                    Pose2d robotPos = odometry.getPose();
//
//                    // return the motor speeds
//                    double speeds[] = path.loop(robotPos.getX(), robotPos.getY(),
//                            robotPos.getHeading());
//
//                    double turnSpeed = 1 - Math.abs(speeds[2]);
//
////                    if(speeds[2] < 0) turnSpeed *= -1;
//
////                    mecDrive.driveRobotCentric(speeds[0], speeds[1], -speeds[2]);
////                    mecDrive.driveRobotCentric(speeds[0], speeds[1], 0);
////                    mecDrive.driveRobotCentric(-speeds[1], -speeds[0], -speeds[2]);
////                    mecDrive.driveRobotCentric(0, 0, 0);
//                    mecDrive.driveRobotCentric(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x);
////                    mecDrive.driveRobotCentric(0, 0, speeds[2]);
//
//                    telemetry.addData("turnSpeed: ", turnSpeed);
//                    telemetry.addData("speeds 2: ", speeds[2]);
//                    telemetry.addData("x: ", robotPos.getX());
//                    telemetry.addData("y: ", robotPos.getY());
//                    telemetry.addData("heading: ", Math.toDegrees(robotPos.getHeading()));
//                    telemetry.addData("heading: ", Math.toDegrees(path.get(1).getPose().getHeading()));
//                    telemetry.update();
//                }
//                mecDrive.stop();
//            } else if(gamepad1.b) {
//                odometry.updatePose(new Pose2d(0,0, new Rotation2d(Math.toRadians(0))));
//            } else {
//                mecDrive.driveRobotCentric(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x);
//            }
//            telemetry.addData("x: ", pose.getX());
//            telemetry.addData("y: ", pose.getY());
//            telemetry.addData("heading: ", Math.toDegrees(pose.getHeading()));
//            telemetry.update();
//        }
//    }
//}