package org.firstinspires.ftc.teamcode.AutoRoadrunner;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.MecanumDrive;

@TeleOp(name = "StrafeFeedbackTest")
public class StrafeFeedbackTest extends LinearOpMode {
    private MecanumDrive drive;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive(hardwareMap, new Pose2d(new Vector2d(0,0), 0), false);

        waitForStart();

        while (opModeIsActive()) {
            Actions.runBlocking(drive.actionBuilder(new Pose2d(0,0,0))
                    .strafeToLinearHeading(new Vector2d(25, 25),0)
                    .strafeToLinearHeading(new Vector2d(0, 0),0)
                    .build());
        }
    }
}
