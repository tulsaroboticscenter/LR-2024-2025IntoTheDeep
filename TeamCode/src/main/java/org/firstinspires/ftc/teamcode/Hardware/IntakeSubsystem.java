package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Abstracts.Subsystem;
import org.firstinspires.ftc.teamcode.Enums.GrabAngle;
import org.firstinspires.ftc.teamcode.Enums.GrabStyle;
import org.firstinspires.ftc.teamcode.Enums.IntakeMode;
import org.firstinspires.ftc.teamcode.Enums.TeleopMode;
import org.firstinspires.ftc.teamcode.Enums.WristAngle;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.MathFunctions;

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

    private double oldDiffyRServoAngle = -180;
    private double oldDiffyLServoAngle = -180;
    private double oldClawServoAngle = -180;

    private boolean pedroAuto = false;

    public void setPedroAuto(boolean set) {
        pedroAuto = set;
    }

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

    private void setClawPos(double newPos) {
        if(oldClawServoAngle != newPos) robot.clawServo.turnToAngle(newPos);

        oldClawServoAngle = newPos;
    }
    private void setDiffyRightPos(double newPos) {
        if(oldDiffyRServoAngle != newPos) robot.diffyRight.turnToAngle(newPos);

        oldDiffyRServoAngle = newPos;
    }
    private void setDiffyLeftPos(double newPos) {
        if(oldDiffyLServoAngle != newPos) robot.diffyLeft.turnToAngle(newPos);

        oldDiffyLServoAngle = newPos;
    }

    public void update(boolean opModeActive) {
        if(!opModeActive) return;

        if (currentIntakeMode == IntakeMode.INTAKE) {
            if (grabStyle == GrabStyle.OUTSIDE_GRAB) {
                setClawPos(params.CLAW_OUTSIDE_GRAB_ANGLE);
            } else if (grabStyle == GrabStyle.INSIDE_GRAB) {
                setClawPos(params.CLAW_INSIDE_GRAB_ANGLE);
            }
        } else if (currentIntakeMode == IntakeMode.OUTTAKE) {
            if (grabStyle == GrabStyle.OUTSIDE_GRAB) {
                if (!shortRange) {
                    setClawPos(params.CLAW_OUTSIDE_DROP_ANGLE);
                } else {
                    if(!pedroAuto) {
                        setClawPos(params.CLAW_OUTSIDE_DROP_ANGLE_SHORT);
                    } else {
                        setClawPos(params.CLAW_OUTSIDE_DROP_ANGLE_SHORT_PEDRO_AUTO);
                    }
                }
            } else if (grabStyle == GrabStyle.INSIDE_GRAB) {
                setClawPos(params.CLAW_INSIDE_DROP_ANGLE);
            }
        } else if (currentIntakeMode == IntakeMode.LOOSE_GRAB) {
            setClawPos(params.CLAW_LOOSE_GRAB);
        }

        if (wristAngle == WristAngle.DOWN) {
            currentWristAngle = params.WRIST_INTAKE_ANGLE;
        } else if (wristAngle == WristAngle.BUCKET_SCORE) {
            if(!pedroAuto) {
                currentWristAngle = params.WRIST_SCORE_ANGLE;
            } else {
                currentWristAngle = params.WRIST_SCORE_ANGLE_AUTO;
            }
        } else if (wristAngle == WristAngle.SPECIMEN_INTAKE) {
            currentWristAngle = params.WRIST_SPECIMEN_INTAKE_ANGLE;
        } else if (wristAngle == WristAngle.SPECIMEN_SCORE_1) {
            currentWristAngle = params.WRIST_SPECIMEN_SCORE_1_ANGLE;
        } else if (wristAngle == WristAngle.SPECIMEN_SCORE_2) {
            currentWristAngle = params.WRIST_SPECIMEN_SCORE_2_ANGLE;
        } else if (wristAngle == WristAngle.SPECIMEN_SCORE_FRONT) {
            currentWristAngle = params.WRIST_SPECIMEN_SCORE_FRONT_ANGLE;
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
        double fixedPivotPos = (Range.clip(currentPivotAngle, -135, 135) / 2.88);

//        opMode.telemetry.addData("fixedPivot: ", fixedPivotPos);

        setDiffyRightPos(fixedWristPos - fixedPivotPos);
        setDiffyLeftPos(fixedWristPos + fixedPivotPos);
    }

    public WristAngle getWristAngle() {
        return wristAngle;
    }
}
