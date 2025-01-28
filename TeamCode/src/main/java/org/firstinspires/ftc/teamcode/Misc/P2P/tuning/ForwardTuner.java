package org.firstinspires.ftc.teamcode.Misc.P2P.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Misc.P2P.Path;
import org.firstinspires.ftc.teamcode.Misc.P2P.Pose2d;
import org.firstinspires.ftc.teamcode.Misc.P2P.P2PDrive;

@TeleOp(name = "P2P: Forward Tuner")
public class ForwardTuner extends LinearOpMode {
    private P2PDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        Path forwardPath = new Path(40, 0, 0);
        Path backwardsPath = new Path(0, 0, 0);

        drive = new P2PDrive(hardwareMap, this);
        drive.setPose(new Pose2d(0, 0, 0));

        waitForStart();

        while (opModeIsActive()) {
            drive.followPath(forwardPath);
            drive.followPath(backwardsPath);
        }
    }
}
