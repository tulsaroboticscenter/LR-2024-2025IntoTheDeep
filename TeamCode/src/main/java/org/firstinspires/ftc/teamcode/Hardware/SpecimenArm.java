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

    private void setArmTargetPosition(double armTargetPosition) {
        this.armTargetPosition = armTargetPosition;
    }

    private void animate() {
        if (currentMode == TeleopMode.SPECIMEN_SCORE) {

        }

        if (animationComplete) {
            robot.specArmServo.turnToAngle(armTargetPosition);
        }
    }

    private void skipAnimation() {
        this.skipAnimation = true;
        this.animationComplete = true;
    }

    private void updateClaw() {
        if (closed) {
            robot.specClawServo.turnToAngle(Params.SPECIMEN_CLAW_CLOSE);
        } else {
            robot.specClawServo.turnToAngle(Params.SPECIMEN_CLAW_OPEN);
        }
    }

    @Override
    public void update(boolean opModeIsActive) {
        if (currentMode == TeleopMode.IDLE) {
            if(!idleArmPowered) {
                robot.specArmServo.disable();
            } else {
                robot.specArmServo.turnToAngle(0);
            }
            if(idleIntakeClawClosed) {
                robot.specClawServo.turnToAngle(Params.SPECIMEN_CLAW_CLOSE_LOOSE);
            } else {
                robot.specClawServo.turnToAngle(Params.SPECIMEN_CLAW_OPEN);
            }
            robot.specClawPivot.turnToAngle(Params.SPEC_CLAW_PIVOT_INTAKE);

            if (teleopModeStart) skipAnimation();
        } else if (currentMode == TeleopMode.SPECIMEN_SCORE) {
            if (!fixSpec) {
                if (timer.time(TimeUnit.MILLISECONDS) <= 250) {
//                robot.specClawPivot.turnToAngle(Params.SPEC_CLAW_PIVOT_INTAKE);
                    if(!afterFixSpec) {
                        robot.specClawServo.turnToAngle(Params.SPECIMEN_CLAW_CLOSE_LOOSE);
                    } else {
                        robot.specClawServo.turnToAngle(Params.SPECIMEN_CLAW_CLOSE);
                    }
                    robot.specArmServo.turnToAngle(Params.SPEC_ARM_SCORE_POS);
                    closed = true;
                } else if (timer.time(TimeUnit.MILLISECONDS) > 250 && timer.time(TimeUnit.MILLISECONDS) <= 350) {
                    robot.specClawPivot.turnToAngle(Params.SPEC_CLAW_PIVOT_OUTTAKE);
                    robot.specClawServo.turnToAngle(Params.SPECIMEN_CLAW_CLOSE);
                    robot.specArmServo.turnToAngle(Params.SPEC_ARM_SCORE_POS);

                } else if (timer.time(TimeUnit.MILLISECONDS) > 1500) {
//                    robot.specArmServo.disable();
                    updateClaw();
                    afterFixSpec = false;
                }
            } else {
                afterFixSpec = true;

                robot.specClawPivot.turnToAngle(Params.SPEC_CLAW_PIVOT_INTAKE);
                robot.specArmServo.turnToAngle(Params.SPEC_ARM_MID_POS);
                robot.specClawServo.turnToAngle(Params.SPECIMEN_CLAW_CLOSE_LOOSE);

                timer.reset();
            }

//            closed = true;
        } else if (currentMode == TeleopMode.INTAKE) {
            if (timer.time(TimeUnit.MILLISECONDS) <= 150) {
                robot.specClawPivot.turnToAngle(Params.SPEC_CLAW_PIVOT_MIDWAY);
                robot.specClawServo.turnToAngle(Params.SPECIMEN_CLAW_OPEN);
//                closed = false;
            } else if (timer.time(TimeUnit.MILLISECONDS) > 150 && timer.time(TimeUnit.MILLISECONDS) <= 500) {
                robot.specArmServo.turnToAngle(Params.SPEC_ARM_INTAKE_POS);
            } else if (timer.time(TimeUnit.MILLISECONDS) > 600) {
                robot.specClawPivot.turnToAngle(Params.SPEC_CLAW_PIVOT_INTAKE);
                robot.specArmServo.turnToAngle(Params.SPEC_ARM_INTAKE_POS);
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
