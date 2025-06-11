package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Abstracts.Subsystem;
import org.firstinspires.ftc.teamcode.Enums.TeleopMode;

import java.util.concurrent.TimeUnit;

public class SpecimenArm extends Subsystem {
    private TeleopMode currentMode = TeleopMode.IDLE;
    private HWProfile robot;
    private ElapsedTime timer = new ElapsedTime();
    private boolean teleopModeStart = false;
    private boolean closed = false;
    private double armTargetPosition = 0;
    private boolean skipAnimation = false;
    private boolean animationComplete = false;
    private TeleopMode lastTeleopMode = currentMode;
    private boolean fixSpec = false;
    private boolean afterFixSpec = false;
    private boolean idleArmPowered = false;
    private boolean idleIntakeClawClosed = false;
    private boolean shortOpen = false;
    private boolean autoMode = false;

    private double armOldTarget = -180;
    private double pivotOldTarget = -180;
    private double clawOldTarget = -180;

    public SpecimenArm(HWProfile robot) {
        this.robot = robot;
    }

    public boolean isClosed() {
        return this.closed;
    }

    public void openClaw() {
        this.closed = false;
    }

    public void setFixSpec(boolean set) {
        this.fixSpec = set;
    }

    public void closeClaw() {
        this.closed = true;
    }

    public void toggleClaw() {
        this.closed = !this.closed;
    }

    private void skipAnimation() {
        this.skipAnimation = true;
        this.animationComplete = true;
    }

    private void armSetPos(double newPos) {
        if (armOldTarget != newPos) robot.specArmServo.turnToAngle(newPos);

        armOldTarget = newPos;
    }

    private void setClawPos(double newPos) {
        if (clawOldTarget != newPos) robot.specClawServo.turnToAngle(newPos);

        clawOldTarget = newPos;
    }

    private void setPivotPos(double newPos) {
        if (pivotOldTarget != newPos) robot.specClawPivot.turnToAngle(newPos);

        pivotOldTarget = newPos;
    }

    private void updateClaw() {
        if (closed) {
            setClawPos(Params.SPECIMEN_CLAW_CLOSE);
        } else {
            if (shortOpen) {
                setClawPos(Params.SPECIMEN_CLAW_OPEN_SHORT);
            } else {
                setClawPos(Params.SPECIMEN_CLAW_OPEN);
            }
        }
    }

    public void setAutoMode(boolean set) {
        this.autoMode = set;
    }

    @Override
    public void update(boolean opModeIsActive) {
        if (currentMode == TeleopMode.IDLE) {
            shortOpen = false;

            if (!idleArmPowered) {
                robot.specArmServo.disable();
            } else {
                armSetPos(Params.SPEC_ARM_INTAKE_POS);
            }
            if (idleIntakeClawClosed) {
                setClawPos(Params.SPECIMEN_CLAW_CLOSE_LOOSE);
            } else {
                setClawPos(Params.SPECIMEN_CLAW_OPEN);
            }
            setPivotPos(Params.SPEC_CLAW_PIVOT_INTAKE);

            if (teleopModeStart) skipAnimation();
        } else if (currentMode == TeleopMode.SPECIMEN_SCORE) {
            shortOpen = true;

            if (!fixSpec) {
                if (timer.time(TimeUnit.MILLISECONDS) <= 250 || teleopModeStart) {
//                setPivotPos();(Params.SPEC_CLAW_PIVOT_INTAKE);
                    if (!afterFixSpec) {
                        setClawPos(Params.SPECIMEN_CLAW_CLOSE_LOOSE);
                    } else {
                        setClawPos(Params.SPECIMEN_CLAW_CLOSE);
                    }
                    armSetPos(Params.SPEC_ARM_SCORE_POS);
                    closed = true;
                } else if (timer.time(TimeUnit.MILLISECONDS) > 250 && timer.time(TimeUnit.MILLISECONDS) <= 750) {
                    setPivotPos(Params.SPEC_CLAW_PIVOT_OUTTAKE);
                    setClawPos(Params.SPECIMEN_CLAW_CLOSE);
                    armSetPos(Params.SPEC_ARM_SCORE_POS);

                } else if (timer.time(TimeUnit.MILLISECONDS) > 1500) {
//                    robot.specArmServo.disable();
                    updateClaw();
                    afterFixSpec = false;
                }
            } else {
                afterFixSpec = true;

                setPivotPos(Params.SPEC_CLAW_PIVOT_INTAKE);
                armSetPos(Params.SPEC_ARM_MID_POS);
                setClawPos(Params.SPECIMEN_CLAW_CLOSE_LOOSE);

                timer.reset();
            }

//            closed = true;
        } else if (currentMode == TeleopMode.INTAKE) {
            shortOpen = false;

            if (timer.time(TimeUnit.MILLISECONDS) <= 150) {
                setPivotPos(Params.SPEC_CLAW_PIVOT_MIDWAY);
                setClawPos(Params.SPECIMEN_CLAW_OPEN);
//                closed = false;
            } else if (timer.time(TimeUnit.MILLISECONDS) > 150 && timer.time(TimeUnit.MILLISECONDS) <= 500) {
                armSetPos(Params.SPEC_ARM_INTAKE_POS);
            } else if (timer.time(TimeUnit.MILLISECONDS) > 600) {
                setPivotPos(Params.SPEC_CLAW_PIVOT_INTAKE);
                if (autoMode) {
                    armSetPos(Params.SPEC_ARM_INTAKE_POS_AUTO);
                } else {
                    armSetPos(Params.SPEC_ARM_INTAKE_POS);
                }
//                robot.specArmServo.disable();

                updateClaw();
            }
        }

        teleopModeStart = false;
    }

    @Override
    public void setTeleopMode(TeleopMode mode) {
        if (this.lastTeleopMode != mode) {
            this.teleopModeStart = true;
            this.currentMode = mode;
            this.timer.reset();

            this.skipAnimation = false;
            this.animationComplete = false;

            this.lastTeleopMode = this.currentMode;
        }
    }

    public void idleArmPowered(boolean set) {
        this.idleArmPowered = set;
    }

    public void idleIntakeClawClosed(boolean set) {
        this.idleIntakeClawClosed = set;
    }
}
