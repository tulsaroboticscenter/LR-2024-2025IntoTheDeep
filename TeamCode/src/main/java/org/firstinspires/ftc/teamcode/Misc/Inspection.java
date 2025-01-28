package org.firstinspires.ftc.teamcode.Misc;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Enums.TeleopMode;
import org.firstinspires.ftc.teamcode.Hardware.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Hardware.Params;

@TeleOp(name = "Inspection TeleOp")
public class Inspection extends LinearOpMode {
    private HWProfile robot = new HWProfile();
    private ArmSubsystem arm;
    private Params params = new Params();

    public void runOpMode() {
        robot.init(hardwareMap, false, false);
        arm = new ArmSubsystem(robot, this, params);
        arm.setTeleopMode(TeleopMode.CUSTOM_POSITION);

        while (opModeInInit()) {
            arm.update(opModeInInit());
            arm.setArmCustomPosition(params.ARM_INSPECTION_POS);
        }

        waitForStart();

        while (opModeIsActive()) {
            arm.update(opModeIsActive());
        }
    }
}
