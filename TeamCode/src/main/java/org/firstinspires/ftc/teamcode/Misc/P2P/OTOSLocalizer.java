package org.firstinspires.ftc.teamcode.Misc.P2P;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class OTOSLocalizer extends Localizer {
    SparkFunOTOS otos;

    OTOSLocalizer(HardwareMap hwMap) {
        otos = hwMap.get(SparkFunOTOS.class, "sensor_otos");
    }

    @Override
    public void update() {
    }

    @Override
    public Pose2d getPose() {
        Pose2d position = new Pose2d(otos.getPosition().x, otos.getPosition().y, otos.getPosition().h);

        return position;
    }

    @Override
    public void setPose(Pose2d newPose) {
        otos.setPosition(new SparkFunOTOS.Pose2D(newPose.getX(), newPose.getY(), newPose.getHeading()));
    }
}
