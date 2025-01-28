package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Abstracts.Subsystem;
import org.firstinspires.ftc.teamcode.Enums.GrabAngle;
import org.firstinspires.ftc.teamcode.Enums.GrabStyle;
import org.firstinspires.ftc.teamcode.Enums.IntakeMode;
import org.firstinspires.ftc.teamcode.Enums.IntakeType;
import org.firstinspires.ftc.teamcode.Enums.TeleopMode;
import org.firstinspires.ftc.teamcode.Enums.WristAngle;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;

@Config
public class IntakeSubsystem extends Subsystem {

    private HWProfile robot;
    public LinearOpMode opMode;
    public Params params;
    public IntakeMode currentIntakeMode = IntakeMode.HOLD;
    public GrabStyle grabStyle;
    public GrabAngle grabAngle;
    public boolean closed = false;
    private boolean shortRange = false;
    private double customAngle = 0;
    private WristAngle wristAngle = WristAngle.DOWN;
    private double customWristAngle = 0;
    private double currentWristAngle = 0;
    private double currentPivotAngle = 0;

    public IntakeSubsystem(HWProfile myRobot, LinearOpMode myOpMode, Params myParams) {
        robot = myRobot;
        opMode = myOpMode;
        params = myParams;

        grabStyle = GrabStyle.OUTSIDE_GRAB;
        grabAngle = GrabAngle.VERTICAL_GRAB;
        intake();
        closed = true;
    }

    public IntakeSubsystem(HWProfile myRobot, OpMode myOpMode, Params myParams) {
        robot = myRobot;
//        opMode = myOpMode;
        params = myParams;

        grabStyle = GrabStyle.OUTSIDE_GRAB;
        grabAngle = GrabAngle.VERTICAL_GRAB;
        intake();
        closed = true;
    }

    public void setCustomWristAngle(double set) {
        customWristAngle = set;
    }

    public void setShortRange(boolean set) {
        shortRange = set;
    }

    public void setGrabStyle(GrabStyle _grabStyle) {
        grabStyle = _grabStyle;
    }

    public void setCustomPivotAngle(double customAngle) {
        customAngle = MathFunctions.clamp(customAngle, 0, 270);
        this.customAngle = customAngle;
    }

    public double getCustomPivotAngle() {
        customAngle = MathFunctions.clamp(customAngle, 0, 270);
        return customAngle;
    }

    public void setGrabAngle(GrabAngle _grabAngle) {
        grabAngle = _grabAngle;
    }

    public GrabStyle getGrabStyle() {
        return grabStyle;
    }

    public GrabAngle getGrabAngle() {
        return grabAngle;
    }

    public void setWristAngle(WristAngle wristAngle) {
        this.wristAngle = wristAngle;
    }

    public void toggle() {
        if (closed) {
            outtake();
            closed = false;
        } else {
            intake();
            closed = true;
        }
    }

    public void intake() {
        currentIntakeMode = IntakeMode.INTAKE;
        closed = true;
    }

    public void outtake() {
        currentIntakeMode = IntakeMode.OUTTAKE;
        closed = false;
    }

    public void hold() {
        currentIntakeMode = IntakeMode.HOLD;
    }

    public void idle() {
        currentIntakeMode = IntakeMode.IDLE;
    }

    public void looseGrab() {
        currentIntakeMode = IntakeMode.LOOSE_GRAB;
    }

    public void update(boolean opModeActive) {
        if(!opModeActive) return;

        if (currentIntakeMode == IntakeMode.INTAKE) {
            if (grabStyle == GrabStyle.OUTSIDE_GRAB) {
                robot.clawServo.turnToAngle(params.CLAW_OUTSIDE_GRAB_ANGLE);
            } else if (grabStyle == GrabStyle.INSIDE_GRAB) {
                robot.clawServo.turnToAngle(params.CLAW_INSIDE_GRAB_ANGLE);
            }
        } else if (currentIntakeMode == IntakeMode.OUTTAKE) {
            if (grabStyle == GrabStyle.OUTSIDE_GRAB) {
                if (!shortRange) {
                    robot.clawServo.turnToAngle(params.CLAW_OUTSIDE_DROP_ANGLE);
                } else {
                    robot.clawServo.turnToAngle(params.CLAW_OUTSIDE_DROP_ANGLE_SHORT);
                }
            } else if (grabStyle == GrabStyle.INSIDE_GRAB) {
                robot.clawServo.turnToAngle(params.CLAW_INSIDE_DROP_ANGLE);
            }
        } else if (currentIntakeMode == IntakeMode.LOOSE_GRAB) {
            robot.clawServo.turnToAngle(params.LOOSE_GRAB);
        }

        if (wristAngle == WristAngle.DOWN) {
            currentWristAngle = params.WRIST_INTAKE_ANGLE;
        } else if (wristAngle == WristAngle.BUCKET_SCORE) {
            currentWristAngle = params.WRIST_SCORE_ANGLE;
        } else if (wristAngle == WristAngle.SPECIMEN_INTAKE) {
            currentWristAngle = params.WRIST_SPECIMEN_INTAKE_ANGLE;
        } else if (wristAngle == WristAngle.SPECIMEN_SCORE_1) {
            currentWristAngle = params.WRIST_SPECIMEN_SCORE_1_ANGLE;
        } else if (wristAngle == WristAngle.SPECIMEN_SCORE_2) {
            currentWristAngle = params.WRIST_SPECIMEN_SCORE_2_ANGLE;
        } else if (wristAngle == WristAngle.IDLE) {
            currentWristAngle = params.WRIST_IDLE_ANGLE;
        } else if (wristAngle == WristAngle.CUSTOM) {
            currentWristAngle = customWristAngle;
        }

        if (grabAngle == GrabAngle.VERTICAL_GRAB) {
            currentPivotAngle = params.PIVOT_VERTICAL_ANG;
        } else if (grabAngle == GrabAngle.HORIZONTAL_GRAB) {
            currentPivotAngle = params.PIVOT_HORIZONTAL_ANG;
        } else if (grabAngle == GrabAngle.CUSTOM) {
            currentPivotAngle = customAngle;
        } else if (grabAngle == GrabAngle.INVERTED) {
            currentPivotAngle = params.PIVOT_INVERTED;
        }

        updateDiffy();
    }

    public void setTeleopMode(TeleopMode mode) {
        return;
    }

    private void updateDiffy() {
        double fixedWristPos = Range.clip(currentWristAngle, 0, 180);
        fixedWristPos += 45;
        double fixedPivotPos = (Range.clip(currentPivotAngle, -90, 90) / 2);

//        opMode.telemetry.addData("fixedPivot: ", fixedPivotPos);

        robot.diffyRight.turnToAngle(fixedWristPos - fixedPivotPos);
        robot.diffyLeft.turnToAngle(fixedWristPos + fixedPivotPos);
    }

    public WristAngle getWristAngle() {
        return wristAngle;
    }
}
