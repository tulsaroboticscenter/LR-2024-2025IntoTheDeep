package org.firstinspires.ftc.teamcode.Misc;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.HWProfile;

@TeleOp(name = "diffyTest")
public class DifferentialTest extends LinearOpMode {
    private HWProfile robot = new HWProfile();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, false, false);

        robot.diffyRight.turnToAngle(0);
        robot.diffyLeft.turnToAngle(0);

        waitForStart();

        double wristPos = 180;
        double pivotPos = 0;

        while (opModeIsActive()) {
            if(gamepad1.a) {
                wristPos = 0;
            } else if(gamepad1.b) {
                wristPos = 180;
            } else if(gamepad1.y) {
                pivotPos = 0;
            } else if(gamepad1.x) {
                pivotPos = 90;
            }

            setWristPosition(wristPos, pivotPos);

            telemetry.addData("diffyRight", robot.diffyRight.getAngle());
            telemetry.addData("diffyLeft", robot.diffyLeft.getAngle());
            telemetry.update();
        }
    }

    public void setWristPosition(double wristPos, double pivotPos) {
        wristPos = Range.clip(wristPos, 0, 180);
        wristPos += 45;
        pivotPos = Range.clip(pivotPos, 0, 90) / 2;

        robot.diffyRight.turnToAngle(wristPos - pivotPos);
        robot.diffyLeft.turnToAngle(wristPos + pivotPos);
    }
}
