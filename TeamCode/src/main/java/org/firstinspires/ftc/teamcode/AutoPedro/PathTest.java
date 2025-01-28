//package org.firstinspires.ftc.teamcode.AutoPedro;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.Enums.AutoLocation;
//import org.firstinspires.ftc.teamcode.Enums.TeleopMode;
//import org.firstinspires.ftc.teamcode.Hardware.ArmSubsystem;
//import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
//import org.firstinspires.ftc.teamcode.Hardware.IntakeSubsystem;
//import org.firstinspires.ftc.teamcode.Hardware.Params;
//import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
//import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
//
//@Config
//@Autonomous (name = "Path test", group = "Autonomous Pathing Tuning")
//public class PathTest extends LinearOpMode {
//    private Telemetry telemetryA;
//
//    public static double DISTANCE = 40;
//
//    private boolean forward = true;
//
//    private Follower follower;
//
//    private PathBuilder forwards;
//    private Path backwards;
//    private AutoManagerPedro autoManager;
//    private ArmSubsystem arm;
//    private HWProfile robot;
//    private IntakeSubsystem intake;
//
//    /**
//     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
//     * initializes the FTC Dashboard telemetry.
//     */
//    public void runOpMode() {
//        follower = new Follower(hardwareMap);
//        robot = new HWProfile();
//        robot.init(hardwareMap, false, false);
//        arm = new ArmSubsystem(robot, this, new Params());
//        intake = new IntakeSubsystem(robot, this, new Params());
//        follower.setStartingPose(new Pose(0, 0, Math.toRadians(0)));
//        follower.setMaxPower(.5);
//
//        arm.setTeleopMode(TeleopMode.IDLE);
//
//        autoManager = new AutoManagerPedro(this, follower, () -> {
//            arm.update();
//            intake.update();
//        }, arm, intake, robot);
//
//        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
//        telemetryA.addLine("This will run the robot in a straight line going " + DISTANCE
//                + " inches forward. The robot will go forward and backward continuously"
//                + " along the path. Make sure you have enough room.");
//        telemetryA.update();
//
//        waitForStart();
//
//        autoManager.buildPaths(AutoLocation.PEDRO_LEFT_3_1_V1);
//
//        autoManager.setSpeed(.9);
//
//        autoManager.runPath(autoManager.specBackup, false);
//        autoManager.runPath(autoManager.intakeYellow1, true);
//        autoManager.safeSleep(1000000);
//    }
//
//}
//
