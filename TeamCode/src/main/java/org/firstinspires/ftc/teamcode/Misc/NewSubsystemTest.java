//package org.firstinspires.ftc.teamcode.Misc;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
//import org.firstinspires.ftc.teamcode.Hardware.Params;
//
//public class NewSubsystemTest extends LinearOpMode {
//    private NewArmSubsystem armSubsystem;
//    private HWProfile robot;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        robot.init(hardwareMap, false, false);
//
//        armSubsystem = new NewArmSubsystem(robot, this, new Params());
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            int test = (int)armSubsystem.getState().getParameter("test");
//        }
//    }
//}
