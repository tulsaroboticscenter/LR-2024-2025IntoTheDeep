package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Abstracts.Subsystem;
import org.firstinspires.ftc.teamcode.Enums.AnimationType;
import org.firstinspires.ftc.teamcode.Enums.IntakeType;
import org.firstinspires.ftc.teamcode.Enums.TeleopMode;
import org.firstinspires.ftc.teamcode.Hardware.KookyMotionProfile.MotionProfile;
import org.firstinspires.ftc.teamcode.Hardware.KookyMotionProfile.ProfileConstraints;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;

import java.util.concurrent.TimeUnit;

@Config
public class ArmSubsystem extends Subsystem {

    private HWProfile robot;
    public LinearOpMode opMode;
    public Params params;
    private TeleopMode currentMode;
    public boolean slidesDisabled = false;
    /*
    P: if you’re not where you want to be, get there.
    I: if you haven’t been where you want to be for a long time, get there faster
    D: if you’re getting close to where you want to be, slow down.
     */
//    public static double Kp = 70;
    public static double Kp = .1;
    public static double KpScoreSpec = 0.05;
    public static double KpScore = .02;
    public static double Ki = 0.001;
    public static double Kd = 0.001;
    public static double KdScore = 0;

    public static double Kp_Intake = .2;
    public static double Ki_Intake = 0.25;
    public static double Kd_Intake = 0;

    public static double SlidesKp = 0.0035;
    public static double SlidesKi = 0.05;
    public static double SlidesKd = 0;

    public static double SlidesKpScore = 0.05;
    public static double SlidesKiScore = .2;
    public static double SlidesKdScore = 0.0004;
    private boolean pedroAuto = false;
    private boolean armGoingDown = false;
    private boolean slidesGoingDown = false;
    public double out = 0;
    public double slidesOut = 0;
    private double oldTargetPos = 0;
    private boolean armUpPark = false;
    public double slidesMultPower = 0;
    public double slidesPower = 1;
    private int bucketScore = 2;
    private static PIDController armPID = new PIDController(Kp, Ki, Kd);
    private static PIDController slidesPID = new PIDController(SlidesKp, SlidesKi, SlidesKd);
    private boolean intakeDownMode = false;
    private boolean intakePushSample = false;
    public boolean teleopModeStart = false;
    private double intakePos = 0;
    private double slidesTargetPos = 0;
    private boolean slidesRetract = false;
    private boolean waitSlidesTransition = false;
    private double armTargetPos = 0;
    private boolean armReachedPosition = false;
    public int armTransistionStage = 0;
    public boolean intakeSpecimen = false;
    public int intakeUpSpecimen = 0;
    private double armPosSpecimen = params.ARM_SPECIMEN_POLE_2_START;
    private double slidesPosSpecimen = params.SLIDES_SPECIMEN_POLE_2_START;
    private TeleopMode lastTeleopMode = TeleopMode.IDLE;
    private boolean outtakeSlidesRetracted = false;
    private boolean armTipBucketScore = false;
    private static double armPower = 1;
    private boolean autoMode = false;
    public static ProfileConstraints motionProfileConstraints = new ProfileConstraints(1, 1, 1);
    private double climbPos = 0;
    private boolean autoLastSample = false;
    private boolean useMotionProfile = false;
    private boolean scoreProtectSlideMotors = false;
    private boolean scoreReachedPos = false;
    private AnimationType animationType = AnimationType.NORMAL;
    private double armCustomPos = 0;
    private double slidesCustomPos = 0;
    private MotionProfile motionProfile;
    private ElapsedTime timer = new ElapsedTime();
    private static double voltage = 12;
    private boolean useThread = false;
    private static boolean opModeRunning = true;
    private static int threadCycles = 0;
    private int animationDelay = 0;
    private double slidesRawCurrentPos;

    private Thread armPidThread = new Thread(() -> {
        double armPosition = (robot.armEncoder.getVoltage() - params.ARM_ZERO) * params.ARM_ABS_TICK_PER_DEG;
        double out = 0;

        try {
            while (opMode.opModeIsActive() && !Thread.interrupted() && useThread && opModeRunning) {
                armPosition = (robot.armEncoder.getVoltage() - params.ARM_ZERO) * params.ARM_ABS_TICK_PER_DEG;

                out = armPID.calculate(armPosition) * armPower * (12.0 / voltage);

                try {
                    robot.armMotor.setVelocity(out);
                } catch (Exception e) {
                    // boo hoo an error threw
                    robot.armMotor.setPower(0);
                }

                threadCycles++;
                Thread.sleep(5);
            }
        } catch (Exception e) {

        }
    });

    public void setParkArmUp(boolean set) {
        armUpPark = set;
    }

    public void setAutoLastSample(boolean autoLastSample) {
        this.autoLastSample = autoLastSample;
    }

    public void setAnimationType(AnimationType animationType) {
        this.animationType = animationType;
        animateTransition();
    }

    public void setIntakePush(boolean set) {
        intakePushSample = set;
    }

    private void animateTransition() {
        if (animationType == AnimationType.NORMAL || animationType == AnimationType.FAST || animationType == AnimationType.ROTATE_SOME_WHILE_RETRACT) {
            if (teleopModeStart) waitSlidesTransition = true;
            if (teleopModeStart) armReachedPosition = false;
            if (teleopModeStart) armTransistionStage = 1;

//            opMode.telemetry.addData("armTransitionStage: ", armTransistionStage);

            opMode.telemetry.addData("timer", timer.time(TimeUnit.MILLISECONDS));
            opMode.telemetry.addData("delay", animationDelay);

            if (timer.time(TimeUnit.MILLISECONDS) <= animationDelay) return;

            if (armTransistionStage == 1) {
                slidesRetract = true;

                double slidesTransitionLen = params.SLIDES_TRANSITION_LEN;

                if (animationType == AnimationType.FAST && autoMode) {
                    slidesTransitionLen = slidesTargetPos;
                }

                if (getSlidesPosition() >= slidesTransitionLen + params.SLIDES_ERROR_TOLERANCE) {
                    waitSlidesTransition = true;
                } else {
                    armTransistionStage = 2;
                }
            } else if (armTransistionStage == 2) {
                slidesRetract = true;
                waitSlidesTransition = false;
                if (animationType == AnimationType.NORMAL) {
                    if (armAtPosition()) {
                        armTransistionStage = 3;
                    }
                } else if (animationType == AnimationType.FAST) {
//                    if(armAtPosition()) {
//                        armTransistionStage = 3;
//                    }
                    if (getArmPosition() >= armTargetPos - params.ARM_FAST_OFFSET && getArmPosition() <= armTargetPos + params.ARM_FAST_OFFSET) {
                        armTransistionStage = 3;
                    }
                }
            } else {
                slidesRetract = false;
                armReachedPosition = true;
            }
        } else if (animationType == AnimationType.NONE) {
            waitSlidesTransition = false;
            armReachedPosition = true;
            armTransistionStage = 3;
            slidesRetract = false;
        }

    }

    public void setClimbPos(double newVal) {
        climbPos = newVal;
    }

    public void setPedroAuto(boolean pedroAuto) {
        this.pedroAuto = pedroAuto;
    }

    public void setAutoMode(boolean newState) {
        autoMode = newState;
    }

    public void useMotionProfile(boolean set) {
        useMotionProfile = set;
    }

    public void setAnimationDelay(int set) {
        animationDelay = set;
    }

    public void setTeleopMode(TeleopMode mode) {
        if (lastTeleopMode != mode) lastTeleopMode = currentMode;
        currentMode = mode;
        teleopModeStart = true;

        timer.reset();
        animationType = AnimationType.NORMAL;
        animationDelay = 0;
        armTransistionStage = 1;
    }

    public void setArmPower(double newPower) {
        armPower = newPower;
    }

    public void setArmTipBucketScore(boolean set) {
        armTipBucketScore = set;
    }

    /*
     * Constructor method
     */
    public ArmSubsystem(HWProfile myRobot, LinearOpMode myOpMode, Params myParams) {
        robot = myRobot;
        opMode = myOpMode;
        params = myParams;

        if (useThread) armPidThread.start();

        timer.reset();
        timer.startTime();

        motionProfile = new MotionProfile(0, 0, motionProfileConstraints);
    }   // close RRMechOps constructor Method

    public ArmSubsystem(HWProfile myRobot, OpMode myOpMode, Params myParams) {
        robot = myRobot;
//        opMode = myOpMode;
        params = myParams;
    }   // close RRMechOps constructor Method

    public double getArmPosition() {
        double pos = (robot.armEncoder.getVoltage() - params.ARM_ZERO) * params.ARM_ABS_TICK_PER_DEG;

        if (params.ARM_ENCODER_INVERTED) pos *= -1;

        return pos;
    }

    public void setArmCustomPosition(double deg) {
        armCustomPos = deg;
    }

    public void setSlidesCustomPosition(double slides) {
        slidesCustomPos = slides;
    }

    private void setArmTargetPosition(double deg) {
        if (deg != armTargetPos) {
            armGoingDown = (deg < getArmPosition());
        }

        if (waitSlidesTransition == false) armTargetPos = deg;
    }

    private void setArmPosition() {
        if (waitSlidesTransition == false) {
            armPID.setSetPoint(armTargetPos);
        } else {
            if (animationType == AnimationType.ROTATE_SOME_WHILE_RETRACT) {
                armPID.setSetPoint(params.ARM_ROTATE_SOME_MODE_DEG);
            }
        }

        motionProfile.finalPosition = armTargetPos;
    }

    public double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    public void resetSlidesPosition() {
//        robot.pinpoint.resetPosAndIMU();
        robot.slidesMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.slidesMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public double getSlidesPosition() {
        return (double) slidesRawCurrentPos / params.SLIDES_TICKS_PER_INCH;
//        return robot.pinpoint.getEncoderX() / params.SLIDES_TICKS_PER_INCH;
    }

    public void setSlidesMultiplier(double newPower) {
        slidesMultPower = newPower;
    }

    public void setSlidesPowerLimit(double set) {
        slidesPower = set;
    }

    private void setTargetSlidesPosition(double len) {
        if (len != slidesTargetPos) {
            slidesGoingDown = (len < getSlidesPosition());
        }

        slidesTargetPos = len;

        setSlidesPosition(slidesTargetPos);
    }

    public double getSlidesTargetPos() {
        return slidesTargetPos;
    }

    public void retractSlidesOuttake() {
        outtakeSlidesRetracted = true;
    }

    public void extendSlidesOuttake() {
        outtakeSlidesRetracted = false;
    }

    private void setSlidesPosition(double len) {
        len = MathFunctions.clamp(len, params.SLIDES_MIN_POS, params.SLIDES_MAX_POS);

        if (slidesRetract || waitSlidesTransition) {
            if (animationType == AnimationType.NORMAL || !autoMode) {
                len = params.SLIDES_TRANSITION_LEN * params.SLIDES_TICKS_PER_INCH;
            } else if (animationType == AnimationType.FAST) {
                len = len * params.SLIDES_TICKS_PER_INCH;
            }
        } else {
            len = len * params.SLIDES_TICKS_PER_INCH;
        }

        slidesPID.setSetPoint((int) len);
    }

    public void setIntakePosition(double len) {
        intakePos = len;
    }

    public void intakeDownMode() {
        intakeDownMode = true;
    }

    public double setArmPositionSpecimen(double pos) {
        armPosSpecimen = pos;
        return armPosSpecimen;
    }

    public double setSlidesPositionSpecimen(double pos) {
        slidesPosSpecimen = pos;

        return slidesPosSpecimen;
    }

    public void intakeUpMode() {
        intakeDownMode = false;
    }

    public boolean armAtPosition() {
        double armPos = getArmPosition();

        double errorTolerance = params.ARM_ERROR_TOLERANCE;

        return (armPos + errorTolerance > armTargetPos && armPos - errorTolerance < armTargetPos);
//        return armPID.atSetPoint();
    }

    public boolean armAtPosition(double tolerance) {
        double armPos = getArmPosition();


        return (armPos + tolerance > armTargetPos && armPos - tolerance < armTargetPos);
//        return armPID.atSetPoint();
    }

    public boolean slidesAtPosition() {
        double slidesPos = getSlidesPosition();

        double errorTolerance = params.SLIDES_ERROR_TOLERANCE;

        return (slidesPos + errorTolerance > slidesTargetPos && slidesPos - errorTolerance < slidesTargetPos);
    }

    public boolean slidesAtPosition(double error) {
        double slidesPos = getSlidesPosition();

        return (slidesPos + error > slidesTargetPos && slidesPos - error < slidesTargetPos);
    }

    public boolean slidesAtPosition(double error, double pos) {
        double slidesPos = getSlidesPosition();

        return (slidesPos + error > pos && slidesPos - error < pos);
    }

    public void setBucket(int bucket) {
        bucketScore = bucket;
    }

    public double getIntakePosition() {
        return Math.sqrt((getSlidesPosition() * getSlidesPosition()) - (params.SLIDE_GROUND_POS * params.SLIDE_GROUND_POS));
    }

    public double getArmTargetPosition() {
        return armPID.getSetPoint();
    }

    public void update(boolean opModeActive) {
        opModeRunning = opModeActive;

        double slides1Current = robot.slidesMotor1.getCurrent(CurrentUnit.AMPS);
        double slides2Current = robot.slidesMotor2.getCurrent(CurrentUnit.AMPS);
        slidesRawCurrentPos = robot.slidesMotor1.getCurrentPosition();

        if (currentMode == TeleopMode.INTAKE) {
//            setSlidesMultiplier(params.SLIDE_MOTOR_POWER);
            double slidesPos = 0;

            if (!intakeSpecimen) {
                if (!autoMode) {
                    slidesPos = MathFunctions.clamp(intakePos, params.INTAKE_MIN_POS, params.INTAKE_MAX_POS);
                } else {
                    slidesPos = MathFunctions.clamp(intakePos, 0, params.INTAKE_MAX_POS);
                }

                slidesPos = Math.sqrt((params.SLIDE_GROUND_POS * params.SLIDE_GROUND_POS) + (slidesPos * slidesPos));

                setTargetSlidesPosition(slidesPos);
                double armDeg = 0;

                if (intakeDownMode) {
//                    armDeg = map(slidesPos, 3, 36, 17 - 5, 25 - 1.5);
                    armDeg = params.ARM_INTAKE_MODE_UP_DEG;
                } else {
                    armDeg = params.ARM_INTAKE_POS;
                }

                setArmTargetPosition(armDeg);
            } else {
                if (!autoMode) {
                    if(slidesAtPosition(1)) {
                        setSlidesPowerLimit(0);
                    } else {
                        setSlidesPowerLimit(1);
                    }

                    if (intakeDownMode) {
                        setArmTargetPosition(params.ARM_TELEOP_SPECIMEN_INTAKE);
                    } else {
                        setArmTargetPosition(params.ARM_TELEOP_SPECIMEN_INTAKE + params.ARM_SPECIMEN_INTAKE_OFFSET);
                    }
                    setTargetSlidesPosition(params.SLIDES_TELEOP_SPECIMEN_INTAKE);
                } else {
                    if (intakeDownMode) {
                        setArmTargetPosition(params.ARM_AUTO_SPECIMEN_INTAKE);
                    } else {
                        setArmTargetPosition(params.ARM_AUTO_SPECIMEN_INTAKE + params.ARM_SPECIMEN_INTAKE_OFFSET);
                    }
                    setTargetSlidesPosition(params.SLIDES_AUTO_SPECIMEN_INTAKE);
                }
            }

            animateTransition();
        } else if (currentMode == TeleopMode.IDLE) {
            setArmTargetPosition(params.ARM_IDLE_DEG);
            setTargetSlidesPosition(params.SLIDES_MIN_POS);
//            setSlidesMultiplier(params.SLIDE_MOTOR_POWER);

            if ((slides1Current >= 2 || slides2Current >= 2) && slidesAtPosition()) {
                setSlidesPowerLimit(0);
            } else {
                setSlidesPowerLimit(1);
            }

            if (lastTeleopMode != TeleopMode.INTAKE) {
                animateTransition();
            } else {
                setAnimationType(AnimationType.NONE);
            }
        } else if (currentMode == TeleopMode.BUCKET_SCORE) {
//            setSlidesMultiplier(params.SLIDE_MOTOR_POWER_OUTTAKE);
            if (teleopModeStart) scoreProtectSlideMotors = false;
            if (teleopModeStart) scoreReachedPos = false;

            if (bucketScore == 2) {
                if (outtakeSlidesRetracted && armTransistionStage == 3) {
                    scoreProtectSlideMotors = false;

                    setTargetSlidesPosition(params.SLIDES_MIN_POS);
                } else {
                    if (armTipBucketScore) {
                        if (autoMode) {
                            setTargetSlidesPosition(params.SLIDES_OUTTAKE_RETRACT_MODE_LEN_AUTO);
                            setArmTargetPosition(100);
                        } else {
                            setTargetSlidesPosition(params.SLIDES_OUTTAKE_RETRACT_MODE_LEN);
                            setArmTargetPosition(params.ARM_BUCKET2_SCORE_DEG);
                        }
                    } else {
                        if (autoMode) {
                            setTargetSlidesPosition(params.SLIDES_BUCKET_2_SCORE_LEN_CLAW_AUTO);
                            setArmTargetPosition(params.ARM_BUCKET2_SCORE_DEG_AUTO);
                        } else {
                            if (!scoreProtectSlideMotors)
                                setTargetSlidesPosition(params.SLIDES_BUCKET_2_SCORE_LEN_CLAW);

                            setArmTargetPosition(params.ARM_BUCKET2_SCORE_DEG);
                        }
                    }
                }
            } else if (bucketScore == 1) {
                if (params.INTAKE_TYPE == IntakeType.TWO_WHEEL_INTAKE) {
                    if (outtakeSlidesRetracted) {
                        setTargetSlidesPosition(params.SLIDES_MIN_POS);
                    } else {
                        setTargetSlidesPosition(params.SLIDES_BUCKET_1_SCORE_LEN);
                    }
                    setArmTargetPosition(params.ARM_BUCKET1_SCORE_DEG);
                } else if (params.INTAKE_TYPE == IntakeType.CLAW) {
                    if (outtakeSlidesRetracted) {
                        setTargetSlidesPosition(params.SLIDES_MIN_POS);
                    } else {
                        setTargetSlidesPosition(params.SLIDES_BUCKET_1_SCORE_LEN_CLAW);
                    }
                    setArmTargetPosition(params.ARM_BUCKET1_SCORE_DEG);
                }
            }

            if (getArmPosition() < 90 || !teleopModeStart) {
                if (lastTeleopMode == TeleopMode.INTAKE) {
                    setAnimationType(AnimationType.NORMAL);
                }
                animateTransition();
            } else {
                setAnimationType(AnimationType.NONE);
                animateTransition();
            }
        } else if (currentMode == TeleopMode.CLIMB) {
            setTargetSlidesPosition(params.SLIDES_MIN_POS);
            setArmTargetPosition(climbPos);

            animateTransition();
        } else if (currentMode == TeleopMode.SPECIMEN_SCORE) {
            if (teleopModeStart) armPosSpecimen = params.ARM_SPECIMEN_POLE_2_SCORE;
            if (teleopModeStart) {
                if (autoMode) {
                    slidesPosSpecimen = params.SLIDES_SPECIMEN_POLE_2_START_AUTO;
                } else {
                    slidesPosSpecimen = params.SLIDES_SPECIMEN_POLE_2_START;
                }
            }

            if(slidesPower != 1) {
                setSlidesMultiplier(1);
            }

            if(slidesMultPower != 1) {
                setSlidesPowerLimit(1);
            }

            setTargetSlidesPosition(slidesPosSpecimen);
            setArmTargetPosition(armPosSpecimen);

            animateTransition();
        } else if (currentMode == TeleopMode.CUSTOM_POSITION) {
            animationType = AnimationType.NONE;

            setArmTargetPosition(armCustomPos);
            setSlidesCustomPosition(slidesCustomPos);

            animateTransition();
        } else if (currentMode == TeleopMode.TOUCH_POLE_AUTO) {
//            if(teleopModeStart) setParkArmUp(false);

            if (armUpPark) {
                setArmTargetPosition(params.ARM_TOUCH_POLE_AUTO_UP);
            } else {
                setArmTargetPosition(params.ARM_TOUCH_POLE_AUTO_DOWN);
            }

            setTargetSlidesPosition(params.SLIDES_TOUCH_POLE_AUTO);

            setAnimationType(AnimationType.NONE);
        } else if (currentMode == TeleopMode.AUTO_SLIDES_IN) {
            setSlidesPosition(0);
            setSlidesMultiplier(1);
            setArmTargetPosition(123);

            setAnimationType(AnimationType.NONE);
        }

        if(lastTeleopMode == TeleopMode.INTAKE && intakeSpecimen && slidesPower == 0) {
            setSlidesPowerLimit(1);
        }

        setArmPosition();

        if (armTransistionStage == 3 && (currentMode == TeleopMode.INTAKE && !intakeSpecimen && !intakeDownMode) && !autoMode) {
            armPID.setP(Kp_Intake);
            armPID.setI(Ki_Intake);
            armPID.setD(Kd_Intake);
        } else {
            if (currentMode == TeleopMode.BUCKET_SCORE) {
                armPID.setP(KpScore);
                armPID.setD(KdScore);
            } else if (currentMode == TeleopMode.SPECIMEN_SCORE) {
                armPID.setP(KpScoreSpec);
            } else {
                armPID.setP(Kp);
                armPID.setD(Kd);
            }
            armPID.setI(Ki);
        }

        voltage = robot.voltageSensor.getVoltage();

        if (slidesGoingDown && (currentMode == TeleopMode.BUCKET_SCORE || (lastTeleopMode == TeleopMode.BUCKET_SCORE && armTransistionStage != 3))) {
            setSlidesPowerLimit(.5);
        } else if ((currentMode == TeleopMode.BUCKET_SCORE)) {
            setSlidesPowerLimit(1);
        }

        armPID.setTolerance(params.ARM_ERROR_TOLERANCE);
        slidesPID.setTolerance(params.SLIDES_ERROR_TOLERANCE);

        if (currentMode == TeleopMode.BUCKET_SCORE && slidesAtPosition(1, params.SLIDES_BUCKET_2_SCORE_LEN_CLAW)) {
            scoreReachedPos = true;
        }

        if (currentMode == TeleopMode.BUCKET_SCORE && !scoreReachedPos && !slidesGoingDown && armTransistionStage == 3) {
            slidesPID.setPID(SlidesKpScore, SlidesKiScore, SlidesKdScore);
        } else {
            slidesPID.setPID(SlidesKp, SlidesKi, SlidesKd);
        }
        out = armPID.calculate(getArmPosition()) * armPower * (12.0 / voltage);
        slidesOut = Range.clip(slidesPID.calculate(slidesRawCurrentPos) * slidesMultPower, -slidesPower, slidesPower);

        if (slidesDisabled) slidesOut = 0;
        opMode.telemetry.addData("arm going down: ", armGoingDown);

        if (!useThread) {
            try {
                robot.armMotor.setPower(out);
            } catch (Exception e) {
                // boo hoo an error threw
                robot.armMotor.setPower(0);
            }
        }

        if (opMode.isStopRequested() || !opModeRunning) {
            armPidThread.interrupt();
        }

        robot.slidesMotor1.setPower(slidesOut);
        robot.slidesMotor2.setPower(slidesOut);

        teleopModeStart = false;

    }

    public TeleopMode getTeleopMode() {
        return currentMode;
    }

    public boolean getIntakeDownMode() {
        return intakeDownMode;
    }

    public TeleopMode getLastTeleopMode() {
        return lastTeleopMode;
    }

    public int getArmTransistionStage() {
        return armTransistionStage;
    }
}