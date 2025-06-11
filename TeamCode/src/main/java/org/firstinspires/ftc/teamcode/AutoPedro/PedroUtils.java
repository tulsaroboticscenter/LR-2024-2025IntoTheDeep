package org.firstinspires.ftc.teamcode.AutoPedro;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Hardware.SpecimenArm;
import org.firstinspires.ftc.teamcode.Misc.P2P.Path;
import org.firstinspires.ftc.teamcode.bedroBathing.follower.Follower;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.PathChain;

import java.util.concurrent.TimeUnit;

public class PedroUtils {
    private static void run(Follower follower, PathChain path, Runnable updates, LinearOpMode opMode, boolean holdEnd) {
        follower.followPath(path, holdEnd);
        follower.update();

        while (follower.isBusy() && !opMode.isStopRequested() && opMode.opModeIsActive()) {
            follower.update();

            updates.run();
        }
    }

    public static void runPath(Follower follower, PathChain path, Runnable updates, LinearOpMode opMode) {
        run(follower, path, updates, opMode, false);
    }

    public static void runPath(Follower follower, PathChain path, boolean holdEnd, Runnable updates, LinearOpMode opMode) {
        run(follower, path, updates, opMode, holdEnd);
    }

    public static void safeSleep(Follower follower, double time, Runnable updates, LinearOpMode opMode) {
        ElapsedTime timer = new ElapsedTime();

        while (timer.time(TimeUnit.MILLISECONDS) < time && opMode.opModeIsActive() && !opMode.isStopRequested()) {
            updates.run();
            follower.update();
        }
    }
}
