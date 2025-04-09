package org.firstinspires.ftc.teamcode.AutoPedro;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;

import java.util.HashSet;

public class AutoSampleIntakeLib {
    private double clawAngle = 90;
    private final double[] clawAngles = {45, 90, 135, 0};
    private int clawAngleSelector = 1;
    private double x = 0;
    private double y = 0;
    private Telemetry telemetry;
    private Gamepad gp;
    private Gamepad previousGp = new Gamepad();
    private HashSet<Pose> sampleLocations = new HashSet<>();

    public AutoSampleIntakeLib(Telemetry telemetry, Gamepad gp) {
        this.telemetry = telemetry;
        this.gp = gp;
    }

    public void displayInfo() {
        this.telemetry.addData("X (right + / left -): ", this.x);
        this.telemetry.addLine("Changes with DPad right (increase) / left (decrease)");
        this.telemetry.addLine();

        this.telemetry.addData("Y (up + / down -): ", this.y);
        this.telemetry.addLine("Changes with DPad up (increase) / down (decrease)");
        this.telemetry.addLine();

        String angleMessage = "";

        if (this.clawAngle == 90) angleMessage = "Vertical";
        if (this.clawAngle == 135) angleMessage = "45 Deg to the right";
        if (this.clawAngle == 45) angleMessage = "45 Deg to the left";
        if (this.clawAngle == 0) angleMessage = "Horizontal";

        this.telemetry.addData("Claw Angle: ", angleMessage);
        this.telemetry.addLine("Changes with Bumpers");
        this.telemetry.addLine();
        this.telemetry.addLine("Press A to confirm and add another sample");


        this.telemetry.update();
    }

    public void update() {
        if (this.gp.dpad_up && !this.previousGp.dpad_up)
            y++;
        if (this.gp.dpad_down && !this.previousGp.dpad_down)
            y--;

        if (this.gp.dpad_right && !this.previousGp.dpad_right)
            x++;
        if (this.gp.dpad_left && !this.previousGp.dpad_left)
            x--;

        if (this.gp.right_bumper && !this.previousGp.right_bumper) {
            this.clawAngleSelector ++;

            if(this.clawAngleSelector > this.clawAngles.length - 1)
                this.clawAngleSelector = 0;
        }

        if (this.gp.left_bumper && !this.previousGp.left_bumper) {
            this.clawAngleSelector --;

            if(clawAngleSelector - 1 < 0)
                this.clawAngleSelector = this.clawAngles.length - 1;
        }

        this.clawAngle = this.clawAngles[this.clawAngleSelector];

        if (this.gp.a && !this.previousGp.a) {
            this.sampleLocations.add(new Pose(this.x * 1.5, this.y * 1.5, Math.toRadians(this.clawAngle)));

            this.x = 0;
            this.y = 0;
            this.clawAngleSelector = 1;
            this.clawAngle = this.clawAngles[this.clawAngleSelector];
        }


        this.previousGp.copy(this.gp);
    }

    public HashSet<Pose> getLocations() {
        return this.sampleLocations;
    }
}
