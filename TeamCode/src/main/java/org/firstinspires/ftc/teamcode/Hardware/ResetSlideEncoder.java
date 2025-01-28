package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "1: Reset Slide Encoder", group = "1")
public class ResetSlideEncoder extends LinearOpMode {
    private HWProfile robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new HWProfile();
        robot.init(hardwareMap, false, false);

        waitForStart();


        while (opModeIsActive()) {
            telemetry.addData("P: ", robot.slidesMotor1.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).p);
            telemetry.addData("I: ", robot.slidesMotor1.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).i);
            telemetry.addData("D: ", robot.slidesMotor1.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).d);
            telemetry.update();

            robot.slidesMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
}
