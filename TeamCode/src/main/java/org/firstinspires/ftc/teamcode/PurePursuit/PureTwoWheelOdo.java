package org.firstinspires.ftc.teamcode.PurePursuit;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.Odometry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.teamcode.Hardware.MecanumDrive;

public class PureTwoWheelOdo extends Odometry {
    private MecanumDrive drive;

    public PureTwoWheelOdo(Pose2d newPose, HardwareMap hwMap, IMU imu) {
        super(newPose);

        com.acmerobotics.roadrunner.Pose2d pose = new com.acmerobotics.roadrunner.Pose2d(newPose.getX(), newPose.getY(), Math.toRadians(newPose.getHeading()));

        drive = new MecanumDrive(hwMap, pose, true);
    }

    @Override
    public void updatePose(Pose2d newPose) {
        com.acmerobotics.roadrunner.Pose2d pose = new com.acmerobotics.roadrunner.Pose2d(newPose.getX(), newPose.getY(), Math.toRadians(-newPose.getHeading()));

        drive.setPose(pose);
    }

    public Pose2d getPose() {
        Pose2d purePose = new Pose2d(new Translation2d(drive.pose.position.x, drive.pose.position.y), new Rotation2d(-drive.pose.heading.toDouble()));
//        Pose2d purePose = new Pose2d(new Translation2d(drive.pose.position.x, drive.pose.position.y), new Rotation2d(Math.toDegrees(drive.pose.heading.toDouble())));
        return purePose;
    }

    @Override
    public void updatePose() {
        drive.updatePoseEstimate();
    }
}
