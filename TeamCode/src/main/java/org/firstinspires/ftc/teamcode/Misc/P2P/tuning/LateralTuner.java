package org.firstinspires.ftc.teamcode.Misc.P2P.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Misc.P2P.Path;
import org.firstinspires.ftc.teamcode.Misc.P2P.Pose2d;
import org.firstinspires.ftc.teamcode.Misc.P2P.P2PDrive;

@TeleOp(name = "P2P: Lateral Tuner")
public class LateralTuner extends LinearOpMode {
    private P2PDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        Path lateralPath = new Path(0, 40, 90);
        Path startPath = new Path(0, 0, 0);

        drive = new P2PDrive(hardwareMap, this);
        drive.setPose(new Pose2d(0, 0, 0));

        waitForStart();

        while (opModeIsActive()) {
            drive.followPath(lateralPath);
            drive.followPath(startPath);
        }
    }
}
