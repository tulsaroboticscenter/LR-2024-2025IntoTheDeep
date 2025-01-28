package org.firstinspires.ftc.teamcode.Misc.P2P;

import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

public class RRLocalizer extends Localizer {
    private MecanumDrive mecanumDrive;
    private HardwareMap hwMap;

    public RRLocalizer(HardwareMap hwMap) {
        this.hwMap = hwMap;
        mecanumDrive = new MecanumDrive(hwMap, new com.acmerobotics.roadrunner.Pose2d(new Vector2d(0, 0), Math.toRadians(0)), true);
    }

    public RRLocalizer(HardwareMap hwMap, Pose newPose) {
        this.hwMap = hwMap;
        mecanumDrive = new MecanumDrive(hwMap, new com.acmerobotics.roadrunner.Pose2d(new Vector2d(newPose.getX(), newPose.getY()), Math.toRadians(newPose.getHeading())), true);
    }

    @Override
    public Pose2d getPose() {
        return new Pose2d(mecanumDrive.pose.position.x, mecanumDrive.pose.position.y, Math.toDegrees(mecanumDrive.pose.heading.toDouble()));
    }

    public void setPose(Pose2d setPose) {
        mecanumDrive.setPose(new com.acmerobotics.roadrunner.Pose2d(setPose.getX(), setPose.getY(), setPose.getHeading()));
    }

    @Override
    public void update() {
        mecanumDrive.updatePoseEstimate();
    }
}
