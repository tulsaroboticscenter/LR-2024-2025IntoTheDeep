package org.firstinspires.ftc.teamcode.Misc.P2P;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class SimplePathDemo extends LinearOpMode {
    private P2PDrive drive;

    public void runOpMode() {
        drive = new P2PDrive(hardwareMap, this);
        drive.runEveryUpdate(() -> {
            //run every update
        });

//        Servo handServo = hardwareMap.servo.get("hand_servo");
//        handServo.setPosition(0);

        waitForStart();

        PathGroup paths = new PathGroup(new Path(106, 5, -45), new Path(106, -75, -90), new Path(10, -75, 135), new Path(2, 7, 90));

        drive.setPose(new Pose2d(0, 0, 90));
//        drive.followPath(new Path(-63.2, 38.5, 90)
//                .runAfterTime(1, () -> {
//                    handServo.setPosition(1);
//                })
//                .runAfterTime(2, () -> {
//                    handServo.setPosition(0);
//                }));
//        drive.setPose(new Pose2d(0, 0, 0));
        drive.setPose(new Pose2d(2, 7, 90));

        drive.followPathBundle(paths);
//        drive.followPath(new Path(75, 0, 0));
//        drive.followPath(new Path(0, 0, 0));
//        drive.followPath(new Path(0, 75, 0));
    }
}