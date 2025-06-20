package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Enums.AnimationType;
import org.firstinspires.ftc.teamcode.Enums.GrabAngle;
import org.firstinspires.ftc.teamcode.Enums.GrabStyle;
import org.firstinspires.ftc.teamcode.Enums.IntakeType;
import org.firstinspires.ftc.teamcode.Enums.WristAngle;
import org.firstinspires.ftc.teamcode.Hardware.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.Hardware.Params;
import org.firstinspires.ftc.teamcode.Enums.TeleopMode;
import org.firstinspires.ftc.teamcode.Hardware.SpecimenArm;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.bedroBathing.tuning.FollowerConstants;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "0: Main TeleOp", group = "0")
public class MainTeleOp extends LinearOpMode {
    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;
    private HWProfile robot;
    private Params params;
    private double armPosition = 0;
    private ArmSubsystem arm;
    private IntakeSubsystem intake;
    private boolean g1_dpadDownCooldown = false;
    private boolean rBumperCooldown = false;
    private boolean g1_dpadUpCooldown = false;
    private TeleopMode teleopMode;
    private double intakePosition = params.INTAKE_MAX_POS; //inches
    private boolean aCooldown = false;
    private boolean telemetryDebug = false;
    private boolean firstRun = true;
    private boolean LSB_cooldown = false;
    private boolean RSB_cooldown = false;
    private ElapsedTime specIntakeTimer = new ElapsedTime();
    private boolean xCooldown = false;
    private int HBSampleCount = 3;
    private int HighSpecCount = 1;
    private int LBSampleCount = 0;
    private boolean g2DpadUpCooldown = false;
    private boolean g2DpadLeftCooldown = false;
    private boolean g2DpadRightCooldown = false;
    private boolean g2DpadDownCooldown = false;
    private boolean g2RBCooldown = false;
    private boolean g2LBCooldown = false;
    private boolean g2DpadLT = false;
    private boolean g2DpadRT = false;
    private boolean rtCooldown = false;
    private boolean slowMode = false;
    private boolean g2BCooldown = false;
    private GrabStyle grabStyle = GrabStyle.OUTSIDE_GRAB;
    private GrabAngle grabAngle = GrabAngle.VERTICAL_GRAB;
    private WristAngle wristAngle = WristAngle.BUCKET_SCORE;
    private double armPosSpecimen = 0;
    private double slidesPosSpecimen = 0;
    private ElapsedTime loopTime = new ElapsedTime();
    private boolean climbUp = false;
    private boolean lbCooldown = false;
    private double climbPos = 0;
    private boolean autoPathingEnabled = false;
    private boolean guideCooldown = false;
    private boolean dropSample = false;
    private boolean g2ACooldown = false;
    private PathChain currentPath;
    private boolean runningPath = false;
    private boolean shareCooldown = false;
    private boolean afterIntakeState = false;
    private double climbs = 0;
    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime timerSec = new ElapsedTime();
    private MultipleTelemetry mTelemetry;
    private boolean endgameNotified = false;
    private boolean climbNotified = false;
    private boolean gameOverNotified = false;
    private int cycles = 0;
    private int lastLoopHertz = 0;
    private boolean specimenInScorePos = false;
    private boolean useSpecArm = true;
    private SpecimenArm specimenArm;
    //    private Follower follower;
    private MecanumDrive mecDrive;

    @Override
    public void runOpMode() throws InterruptedException {
//        follower = new Follower(hardwareMap);
        mecDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0), false);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        robot = new HWProfile();
        robot.init(hardwareMap, false, false);
        params = new Params();
        arm = new ArmSubsystem(robot, this, params);
        intake = new IntakeSubsystem(robot, this, params);

        leftFront = hardwareMap.get(DcMotorEx.class, FollowerConstants.leftFrontMotorName);
        leftRear = hardwareMap.get(DcMotorEx.class, FollowerConstants.leftRearMotorName);
        rightRear = hardwareMap.get(DcMotorEx.class, FollowerConstants.rightRearMotorName);
        rightFront = hardwareMap.get(DcMotorEx.class, FollowerConstants.rightFrontMotorName);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.otos.calibrateImu();
        robot.otos.setPosition(new SparkFunOTOS.Pose2D(0, 0, Math.toRadians(Params.AUTO_END_HEADING)));

        waitForStart();

        robot.slidesMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setAutoMode(false);
        arm.setSlidesMultiplier(1);

        loopTime.reset();
        timer.reset();
        timerSec.reset();

        arm.update(opModeIsActive());

        if (arm.getArmPosition() >= 30) {
            if (arm.getSlidesPosition() < 30) {
                teleopMode = TeleopMode.TOUCH_POLE_AUTO;
                arm.setParkArmUp(true);
            } else {
                teleopMode = TeleopMode.BUCKET_SCORE;
                arm.setBucket(2);
            }
        } else {
            teleopMode = TeleopMode.IDLE;
        }

        specimenArm = new SpecimenArm(robot);
        specimenArm.setTeleopMode(TeleopMode.IDLE);
        specimenArm.idleArmPowered(true);

        arm.setTeleopMode(teleopMode);
        arm.setAnimationType(AnimationType.NONE);
        arm.update(opModeIsActive());

        params.TELEOP_START_MODE = TeleopMode.IDLE;

        while (opModeIsActive()) {
            if (loopTime.time(TimeUnit.MILLISECONDS) >= 1000) {
                lastLoopHertz = cycles;
                cycles = 0;
                loopTime.reset();
            }

            cycles++;

            if (firstRun) {
                firstRun = false;
            }

            double botHeading = robot.otos.getPosition().h;
            double rotX = -gamepad1.left_stick_x * Math.cos(botHeading) - -gamepad1.left_stick_y * Math.sin(botHeading);
            double rotY = -gamepad1.left_stick_x * Math.sin(botHeading) + -gamepad1.left_stick_y * Math.cos(botHeading);

            telemetry.addData("rot x: ", rotX);
            telemetry.addData("rot y: ", rotY);

            if (slowMode) {
                mecDrive.setDrivePowers(new PoseVelocity2d(new Vector2d(rotY, rotX), -gamepad1.right_stick_x));
            } else {
                mecDrive.setDrivePowers(new PoseVelocity2d(new Vector2d(rotY, rotX), -gamepad1.right_stick_x));
            }

            mecDrive.updatePoseEstimate();

            /* *******************INTAKE******************* */

            if(gamepad2.b && !g2BCooldown) {
                g2BCooldown = true;

                useSpecArm = !useSpecArm;
            }

            if(!gamepad2.b) g2BCooldown = false;

            if ((gamepad1.dpad_right || (gamepad1.touchpad && useSpecArm)) && !autoPathingEnabled || (teleopMode == TeleopMode.SPECIMEN_SCORE && gamepad1.left_bumper && !lbCooldown)) {
                teleopMode = TeleopMode.INTAKE;
                grabAngle = GrabAngle.VERTICAL_GRAB;

                if (gamepad1.left_bumper) lbCooldown = true;

                arm.intakeSpecimen = true;

                if (!useSpecArm) {
                    intake.setGrabAngle(grabAngle);
                    arm.setTeleopMode(teleopMode);
                    intake.outtake();
                    intake.update(opModeIsActive());
                } else {
                    arm.setTeleopMode(TeleopMode.CUSTOM_POSITION);
                    arm.setAnimationType(AnimationType.NORMAL);
                    arm.setArmCustomPosition(65);
                    arm.setSlidesCustomPosition(4);
                    arm.update(opModeIsActive());

                    intake.setWristAngle(WristAngle.BUCKET_SCORE);
                    intake.setGrabAngle(GrabAngle.VERTICAL_GRAB);
                    intake.update(opModeIsActive());

                    specimenArm.setTeleopMode(TeleopMode.INTAKE);
                    specimenArm.openClaw();

                    if(gamepad1.touchpad) {
                        dropSample = true;
                        arm.setAnimationDelay(500);
                    }

                    specimenArm.update(opModeIsActive());
                }
            }

            if (teleopMode == TeleopMode.INTAKE && arm.intakeSpecimen) {
                if (!useSpecArm) {
                    grabAngle = GrabAngle.VERTICAL_GRAB;
                    grabStyle = GrabStyle.OUTSIDE_GRAB;
                    wristAngle = WristAngle.SPECIMEN_INTAKE;

                    intake.setWristAngle(wristAngle);
                    intake.setGrabStyle(grabStyle);
                    intake.setGrabAngle(grabAngle);
//                intake.setShortRange(true);

                    if (params.INTAKE_TYPE == IntakeType.TWO_WHEEL_INTAKE) {
                        intake.intake();
                    } else if (params.INTAKE_TYPE == IntakeType.CLAW) {
                        if (gamepad1.right_bumper && !rBumperCooldown) {
                            specIntakeTimer.reset();
                            intake.toggle();
                            rBumperCooldown = true;
                        }
                    }

                    if (gamepad1.right_trigger > .1) {
                        arm.intakeUpMode();
                    } else {
                        arm.intakeDownMode();
                    }

                    if (params.INTAKE_TYPE == IntakeType.TWO_WHEEL_INTAKE) {
                        if (gamepad1.right_bumper) {
                            arm.intakeUpSpecimen = 1;
                        } else {
                            arm.intakeUpSpecimen = 0;
                        }
                    } else if (params.INTAKE_TYPE == IntakeType.CLAW) {
//                    if (gamepad1.left_bumper && !lbCooldown) {
////                        lbCooldown = true;
//
//                        if (intake.closed) {
//                            arm.intakeUpSpecimen = 2;
//                        } else {
//                            arm.intakeUpSpecimen = 1;
//                        }
//                    } else {
//                        arm.intakeUpSpecimen = 0;
//                    }
                    }
                } else {
                    if (!dropSample) {
                        arm.setArmCustomPosition(65);
                        arm.setSlidesCustomPosition(4);

                        if (gamepad1.right_bumper && !rBumperCooldown) {
                            rBumperCooldown = true;
                            specimenArm.toggleClaw();
                        }
                    } else {
                        arm.setArmCustomPosition(90);
                        arm.setSlidesCustomPosition(1.5);
                        specimenArm.openClaw();

                        if(gamepad1.left_bumper) {
                            lbCooldown = true;
                            dropSample = false;
                        }

                        if (gamepad1.right_bumper && !rBumperCooldown) {
                            rBumperCooldown = true;
                            intake.toggle();
                        }
                    }
                }
            } else {
                intake.setShortRange(false);
            }


            if (gamepad1.a && !aCooldown) {
                aCooldown = true;
                grabAngle = GrabAngle.VERTICAL_GRAB;

                intake.setGrabAngle(grabAngle);
                wristAngle = WristAngle.DOWN;
                intake.setWristAngle(wristAngle);
                boolean skipTransistion = false;

                if (teleopMode == TeleopMode.IDLE && arm.armTransistionStage == 3)
                    skipTransistion = true;

                teleopMode = TeleopMode.INTAKE;
                arm.intakeSpecimen = false;
                arm.setTeleopMode(teleopMode);

                if (skipTransistion) {
//                    arm.setAnimationType(AnimationType.NONE);
//                    arm.update(true);
                }

                intakePosition = params.INTAKE_DEF_POS;
                if (params.INTAKE_TYPE == IntakeType.CLAW) intake.outtake();
            } else if (!gamepad1.a) {
                aCooldown = false;
            }

            if (teleopMode == TeleopMode.INTAKE && !arm.intakeSpecimen) {
                if (gamepad1.right_trigger > .1) {
                    intakePosition += 3 * gamepad1.right_trigger;
                } else if (gamepad1.left_trigger > .1) {
                    intakePosition -= 3 * gamepad1.left_trigger;
                }

                arm.setIntakePosition(intakePosition);

                if (params.INTAKE_TYPE == IntakeType.TWO_WHEEL_INTAKE) {
                    if (gamepad1.right_bumper) {
                        intake.intake();
                        arm.intakeDownMode();
                    } else if (gamepad1.left_bumper) {
                        intake.outtake();
                        arm.intakeDownMode();
                    } else {
                        intake.hold();
                        arm.intakeUpMode();
                    }
                } else if (params.INTAKE_TYPE == IntakeType.CLAW) {
                    if (gamepad1.right_bumper && !rBumperCooldown) {
                        rBumperCooldown = true;
                        intake.toggle();
                    }

                    if (gamepad1.left_bumper) {
                        arm.intakeUpMode();
                    } else {
                        arm.intakeDownMode();
                    }

//                    if (gamepad1.left_stick_button && !LSB_cooldown) {
//                        LSB_cooldown = true;
//
//                        if (intake.getGrabStyle() == GrabStyle.OUTSIDE_GRAB) {
//                            intake.setGrabStyle(GrabStyle.INSIDE_GRAB);
//                        } else if (intake.getGrabStyle() == GrabStyle.INSIDE_GRAB) {
//                            intake.setGrabStyle(GrabStyle.OUTSIDE_GRAB);
//                        }
//                    }

                    if (gamepad1.right_stick_button && !RSB_cooldown) {
                        RSB_cooldown = true;

                        if (intake.getGrabAngle() == GrabAngle.VERTICAL_GRAB) {
                            intake.setGrabAngle(GrabAngle.HORIZONTAL_GRAB);
                        } else if (intake.getGrabAngle() == GrabAngle.HORIZONTAL_GRAB) {
                            intake.setGrabAngle(GrabAngle.VERTICAL_GRAB);
                        }
                    }
                }
            }

            if (teleopMode == TeleopMode.INTAKE) {
                slowMode = true;
            } else {
                slowMode = false;
            }


            /* *******************BUCKET SCORE☑******************* */
            if (gamepad1.y) {
                teleopMode = TeleopMode.BUCKET_SCORE;
                arm.setTeleopMode(teleopMode);
                arm.setAnimationType(AnimationType.FAST);
                grabAngle = GrabAngle.HORIZONTAL_GRAB;
//                intake.looseGrab();
//                arm.setAnimationType(AnimationType.FAST);
                arm.setBucket(2);
            }
            if (gamepad1.b) {
                teleopMode = TeleopMode.BUCKET_SCORE;
                arm.setTeleopMode(teleopMode);
                arm.setAnimationType(AnimationType.FAST);
                grabAngle = GrabAngle.HORIZONTAL_GRAB;
//                intake.looseGrab();
//                arm.setAnimationType(AnimationType.FAST);
                arm.setBucket(1);
            }

            if (teleopMode == TeleopMode.BUCKET_SCORE) {
                if (gamepad1.right_stick_button && !RSB_cooldown) {
                    RSB_cooldown = true;

                    if (intake.getGrabAngle() == GrabAngle.INVERTED) {
                        grabAngle = GrabAngle.HORIZONTAL_GRAB;
                    } else if (intake.getGrabAngle() == GrabAngle.HORIZONTAL_GRAB) {
                        grabAngle = GrabAngle.INVERTED;
                    }
                }

                if (arm.getArmTransistionStage() == 3) {
                    if (gamepad1.left_bumper) {
                        wristAngle = WristAngle.IDLE;
                    } else {
                        wristAngle = WristAngle.BUCKET_SCORE;
                    }
                } else {
                    if (arm.getArmTransistionStage() == 1) {
                        wristAngle = WristAngle.IDLE;
                        intake.setWristAngle(wristAngle);
                    } else {
                        wristAngle = WristAngle.CUSTOM;
                        intake.setWristAngle(wristAngle);
                        intake.setCustomWristAngle(params.WRIST_OUTTAKE_DURING_ANIMATION);
                    }
                }

                if(gamepad1.right_trigger >= .1 && !rtCooldown) {
                    rtCooldown = true;
                    Params.SLIDES_HIGH_SCORE = !Params.SLIDES_HIGH_SCORE;
                }

                intake.setWristAngle(wristAngle);
                intake.setGrabAngle(grabAngle);

                if (gamepad1.right_bumper && !rBumperCooldown) {
                    rBumperCooldown = true;

                    intake.closed = !intake.closed;
                }

                if (intake.closed) {
                    if (arm.armTransistionStage == 2) {
//                        intake.looseGrab();
                        intake.intake();
                    } else {
                        intake.intake();
                    }
                } else {
                    intake.outtake();
                }

                if (gamepad1.left_trigger > .1) {
                    arm.retractSlidesOuttake();
                } else {
                    arm.extendSlidesOuttake();
                }
            }

            /* *******************IDLE******************* */

            if (gamepad1.x && !xCooldown) {
                xCooldown = true;

                if (teleopMode == TeleopMode.INTAKE) {
                    wristAngle = WristAngle.IDLE;
                } else {
                    wristAngle = WristAngle.IDLE;
                }
                grabAngle = GrabAngle.VERTICAL_GRAB;

                intake.setWristAngle(wristAngle);
                intake.setGrabAngle(grabAngle);

                teleopMode = TeleopMode.IDLE;
                arm.idleExtend(false);
                arm.setTeleopMode(teleopMode);
            }

            if (!gamepad1.x) {
                xCooldown = false;
            }

            if (teleopMode == TeleopMode.IDLE) {
                if (gamepad1.right_stick_button && !RSB_cooldown) {
                    RSB_cooldown = true;

                    arm.idleExtend(!arm.isIdleExtended());
                }

                if (params.INTAKE_TYPE == IntakeType.TWO_WHEEL_INTAKE) {
                    if (gamepad1.right_bumper) {
                        intake.intake();
                    } else if (gamepad1.left_bumper) {
                        intake.outtake();
                    } else {
                        intake.hold();
                    }
                } else if (params.INTAKE_TYPE == IntakeType.CLAW) {
                    if (gamepad1.right_bumper && !rBumperCooldown && params.INTAKE_TYPE == IntakeType.CLAW) {
                        rBumperCooldown = true;
                        intake.toggle();
                    }
                }
            }

            /* *******************CLIMB******************* */

            if (gamepad1.dpad_up) {
                teleopMode = TeleopMode.CLIMB;
                arm.setTeleopMode(teleopMode);
                climbUp = true;

                climbPos = params.ARM_CLIMB_UP_MAX_POS;
            }

            if (teleopMode == TeleopMode.CLIMB) {
                if (gamepad1.dpad_down) {
                    climbUp = false;
                    climbPos = params.ARM_CLIMB_DOWN_MIN_POS;
                }

                if (!climbUp) {
//                    if (gamepad1.left_trigger > .1) {
//                        climbPos = params.ARM_CLIMB_DOWN_MAX_POS;
//                    } else if (gamepad1.right_trigger > .1) {
//                        climbPos = params.ARM_CLIMB_DOWN_MIN_POS;
//                    }
                }

                arm.setClimbPos(climbPos);
            }

            /* *******************SPECIMEN SCORE******************* */

            if (gamepad1.dpad_left && !autoPathingEnabled || (!lbCooldown && teleopMode == TeleopMode.INTAKE && arm.intakeSpecimen && gamepad1.left_bumper)) {
                teleopMode = TeleopMode.SPECIMEN_SCORE;

                if (gamepad1.left_bumper) lbCooldown = true;

                if (!useSpecArm) {
                    arm.setTeleopMode(teleopMode);
                    arm.setAnimationType(AnimationType.NORMAL);
                    if (gamepad1.left_bumper) lbCooldown = true;

                    int delay = (int) Range.clip((params.SPEC_SCORE_INTAKE_DELAY - (int) specIntakeTimer.time(TimeUnit.MILLISECONDS)), 0, params.SPEC_SCORE_INTAKE_DELAY);

                    arm.setAnimationDelay(delay);
                    intake.intake();
                    intake.looseGrab();

                    if (delay == 0) {
                        wristAngle = WristAngle.SPECIMEN_SCORE_1;
                    }
//                grabAngle = GrabAngle.INVERTED;
                    intake.setWristAngle(wristAngle);
                    intake.setGrabAngle(grabAngle);
                    specimenInScorePos = false;
                    armPosSpecimen = params.ARM_SPECIMEN_POLE_2_START;
                    slidesPosSpecimen = params.SLIDES_SPECIMEN_POLE_2_START;
                    arm.setArmPositionSpecimen(armPosSpecimen);
                    arm.setSlidesPositionSpecimen(slidesPosSpecimen);
                } else {
                    arm.setTeleopMode(TeleopMode.CUSTOM_POSITION);
                    arm.setArmCustomPosition(65);
                    arm.setSlidesCustomPosition(4);
                    arm.update(opModeIsActive());

                    specimenArm.setTeleopMode(TeleopMode.SPECIMEN_SCORE);
                    specimenArm.update(opModeIsActive());
                }
            }

            if (teleopMode == TeleopMode.SPECIMEN_SCORE) {
                if (!useSpecArm) {
                    if (gamepad1.right_bumper && !rBumperCooldown && params.INTAKE_TYPE == IntakeType.CLAW) {
                        rBumperCooldown = true;
                        if (intake.closed) {
                            intake.closed = false;
                        } else {
                            intake.closed = true;
                        }
                    }

                    if (!intake.closed) {
                        intake.outtake();
                    } else {
                        if (specimenInScorePos) {
                            intake.intake();
                        } else {
//                        intake.intake();
                            intake.looseGrab();
                        }
                    }

                    int delay = (int) Range.clip((params.SPEC_SCORE_INTAKE_DELAY - (int) specIntakeTimer.time(TimeUnit.MILLISECONDS)), 0, params.SPEC_SCORE_INTAKE_DELAY);

//                if(arm.getArmTransistionStage() != 1) {
//                    wristAngle = WristAngle.SPECIMEN_SCORE_1;
//                    intake.setWristAngle(wristAngle);
//                }

//                    armPosSpecimen = params.ARM_SPECIMEN_POLE_2_SCORE;

                    if (gamepad1.right_trigger > .1) {
                        slidesPosSpecimen = params.SLIDES_SPECIMEN_POLE_2_SCORE_CLAW;
                        specimenInScorePos = true;

                        wristAngle = WristAngle.SPECIMEN_SCORE_2;
                        intake.setWristAngle(wristAngle);
                    } else if (gamepad1.left_trigger > .1) {
                        slidesPosSpecimen = params.SLIDES_SPECIMEN_POLE_2_START;
                        specimenInScorePos = false;

                        wristAngle = WristAngle.SPECIMEN_SCORE_1;
                        intake.setWristAngle(wristAngle);
                    }

                    armPosSpecimen = arm.setArmPositionSpecimen(armPosSpecimen);
                    slidesPosSpecimen = arm.setSlidesPositionSpecimen(slidesPosSpecimen);
                } else {
                    if (gamepad1.right_bumper && !rBumperCooldown) {
                        rBumperCooldown = true;
                        specimenArm.toggleClaw();
                    }

                    specimenArm.setFixSpec(gamepad1.right_trigger > .1);
                }
            }

            /* *******************RESET IMU & POSITION******************* */
            if (gamepad1.options) {
//                follower.resetIMU();
//                robot.imu.resetYaw();
//                follower.resetIMU();
//                follower.setPose(new Pose(follower.getPose().getX(), follower.getPose().getY(), 0));
                mecDrive.setPose(new Pose2d(0, 0, 0));
                robot.otos.resetTracking();
                Params.AUTO_END_HEADING = 0;
                gamepad1.rumble(500);
            }

            if (!gamepad1.right_bumper) rBumperCooldown = false;

            if (!gamepad1.left_stick_button) LSB_cooldown = false;
            if (!gamepad1.right_stick_button) RSB_cooldown = false;

//            arm.setSlidesMultiplier(0);
            arm.setArmPower(params.ARM_POWER_DEFAULT);

            if (teleopMode == TeleopMode.INTAKE && arm.getIntakeDownMode()) {
//                arm.setArmPower(1.5);
            } else {
                arm.setArmPower(params.ARM_POWER_DEFAULT);
            }

            if (gamepad1.share && !shareCooldown) {
                shareCooldown = true;

                useSpecArm = !useSpecArm;
            }

            if(!gamepad1.share) shareCooldown = false;

            arm.update(opModeIsActive());
            intake.update(opModeIsActive());

            if (teleopMode != TeleopMode.SPECIMEN_SCORE && (teleopMode != TeleopMode.INTAKE && arm.intakeSpecimen)) {
                specimenArm.idleArmPowered(true);
                specimenArm.setTeleopMode(TeleopMode.IDLE);
                specimenArm.idleArmPowered(true);

                telemetry.addLine("set spec arm");
            }

            specimenArm.update(opModeIsActive());


            intakePosition = MathFunctions.clamp(intakePosition, params.INTAKE_MIN_POS, params.INTAKE_MAX_POS);

            /* *******************NOTIFICATIONS******************* */

            if (timer.time(TimeUnit.SECONDS) >= (120 - 30) && !endgameNotified) {
                endgameNotified = true;
                gamepad1.rumbleBlips(1);
            }

            if (timer.time(TimeUnit.SECONDS) >= (120 - 8) && !climbNotified) {
                climbNotified = true;
                gamepad1.rumble(750);
            }

            if (timer.time(TimeUnit.SECONDS) >= 120 && !gameOverNotified) {
                gameOverNotified = true;
                gamepad1.rumbleBlips(2);
            }


            /* *******************TELEMETRY AND SCORING*******************

            mTelemetry.addLine("A: Debug Telemetry");
            mTelemetry.addLine("Dpad Up/Down: High Bucket Samples");
            mTelemetry.addLine("Dpad Right/Left: High Pole Specimen");
            mTelemetry.addLine("Right/Left Trigger: Climbs");

            //Sample
            if (gamepad2.dpad_up && !g2DpadUpCooldown) {
                g2DpadUpCooldown = true;
                HBSampleCount++;
            }
            if (!gamepad2.dpad_up) g2DpadUpCooldown = false;

            if (gamepad2.dpad_down && !g2DpadDownCooldown) {
                g2DpadDownCooldown = true;
                HBSampleCount--;
            }
            if (!gamepad2.dpad_down) g2DpadDownCooldown = false;


            if (gamepad2.right_bumper && !g2RBCooldown) {
                g2RBCooldown = true;
                LBSampleCount++;
            }
            if (!gamepad2.right_bumper) g2RBCooldown = false;

            if (gamepad2.left_bumper && !g2LBCooldown) {
                g2LBCooldown = true;
                LBSampleCount--;
            }
            if (!gamepad2.left_bumper) g2LBCooldown = false;

            //Specimen
            if (gamepad2.dpad_right && !g2DpadRightCooldown) {
                g2DpadRightCooldown = true;
                HighSpecCount++;
            }
            if (!gamepad2.dpad_right) g2DpadRightCooldown = false;

            if (gamepad2.dpad_left && !g2DpadLeftCooldown) {
                g2DpadLeftCooldown = true;
                HighSpecCount--;
            }
            if (!gamepad2.dpad_left) g2DpadLeftCooldown = false;

            //Climb
            if (gamepad2.right_trigger > .1 && !g2DpadRT) {
                g2DpadRT = true;
                climbs++;
            }
            if (gamepad2.right_trigger < .1) g2DpadRT = false;

            if (gamepad2.left_trigger > .1 && !g2DpadLT) {
                g2DpadLT = true;
                climbs--;
            }
            if (gamepad2.left_trigger < .1) g2DpadLT = false;

            climbs = MathFunctions.clamp(climbs, 0, 2);

            double autoScore = 37;

            double timeMin = 1 - timer.time(TimeUnit.MINUTES);
            double timeSec = (60 - timerSec.time(TimeUnit.SECONDS));

            if (timerSec.time(TimeUnit.SECONDS) >= 60) timerSec.reset();

            if (params.AUTO_SCORE != 0) autoScore = params.AUTO_SCORE;

             */

//            mTelemetry.addLine();
//            mTelemetry.addLine();
//            mTelemetry.addData("score: ", 37 + (HBSampleCount * 8) + (LBSampleCount * 6) + (HighSpecCount * 10) + (climbs * 15));
//            mTelemetry.addData("time: ", ((int) timeMin + ":" + (int) timeSec));
//            mTelemetry.addData("high samples: ", HBSampleCount);
//            mTelemetry.addData("high rung samples: ", HighSpecCount);
//            mTelemetry.addData("climbs: ", climbs);

            if (gamepad2.a && !g2ACooldown) {
                g2ACooldown = true;
                telemetryDebug = !telemetryDebug;
            }

            if(gamepad1.right_trigger <= .1) {
                rtCooldown = false;
            }

            if (!gamepad2.a) g2ACooldown = false;
            if (!gamepad1.left_bumper) lbCooldown = false;

            mTelemetry.addData("lbc: ", lbCooldown);
//            mTelemetry.addData("slides 1 current: ", robot.slidesMotor1.getCurrent(CurrentUnit.AMPS));
//            mTelemetry.addData("slides 2 current: ", robot.slidesMotor2.getCurrent(CurrentUnit.AMPS));
//            mTelemetry.addData("motorLR power: ", leftRear.getPower());
//            mTelemetry.addData("motorLF power: ", leftFront.getPower());
//            mTelemetry.addData("motorRF power: ", rightFront.getPower());
//            mTelemetry.addData("motorRR power: ", rightRear.getPower());
//            mTelemetry.addData("slides 1 power", robot.slidesMotor1.getPower());
//            mTelemetry.addData("slides 2 power", robot.slidesMotor2.getPower());
//            mTelemetry.addData("arm current: ", robot.armMotor.getCurrent(CurrentUnit.AMPS));
            mTelemetry.addData("hertz: ", lastLoopHertz);
            mTelemetry.addData("dampen: ", arm.dampen);
            mTelemetry.addData("loop: ", loopTime.time(TimeUnit.MILLISECONDS));
            mTelemetry.addData("arm abs position: ", arm.getArmPosition());
            mTelemetry.addData("slides pos: ", arm.getSlidesPosition());
//            mTelemetry.addData("arm motor1 amp: ", robot.armMotor1.getCurrent(CurrentUnit.AMPS));
//            mTelemetry.addData("arm motor2 amp: ", robot.armMotor2.getCurrent(CurrentUnit.AMPS));
            mTelemetry.addData("extension time: ", arm.extensionTime);
            mTelemetry.addData("slides out: ", arm.slidesOut);
            mTelemetry.addData("slides mult: ", arm.slidesMultPower);
            mTelemetry.addData("slides power: ", arm.slidesPower);
            mTelemetry.addData("auto end heading: ", Params.AUTO_END_HEADING);
            mTelemetry.addData("slides at position: ", arm.slidesAtPosition());
//            mTelemetry.addData("motorLR", leftRear.getCurrent(CurrentUnit.AMPS));
//            mTelemetry.addData("motorLF", leftFront.getCurrent(CurrentUnit.AMPS));
//            mTelemetry.addData("motorRR", rightRear.getCurrent(CurrentUnit.AMPS));
//            mTelemetry.addData("motorRF", rightFront.getCurrent(CurrentUnit.AMPS));

            //            mTelemetry.addData("pivot angle: ", intake.getGrabAngle());
//            mTelemetry.addData("arm out: ", arm.out);

            if (telemetryDebug) {
                mTelemetry.addData("arm abs position: ", arm.getArmPosition());
                mTelemetry.addData("arm target position: ", arm.getArmTargetPosition());
                mTelemetry.addData("arm at position: ", arm.armAtPosition());
                mTelemetry.addData("slides out: ", arm.slidesOut);
                mTelemetry.addData("slides target pos: ", arm.getSlidesTargetPos());
                mTelemetry.addData("diffyLeft: ", robot.diffyLeft.getAngle());
                mTelemetry.addData("diffyRight: ", robot.diffyRight.getAngle());
                mTelemetry.addData("arm abs position: ", arm.getArmPosition());
                mTelemetry.addData("heading: ", Math.toDegrees(botHeading));
                mTelemetry.addData("arm out: ", arm.out);
                mTelemetry.addData("wristAngle: ", intake.getWristAngle());
                mTelemetry.addData("auto pathing enabled: ", autoPathingEnabled);
//                mTelemetry.addData("robot x: ", follower.getPose().getX());
//                mTelemetry.addData("robot y: ", follower.getPose().getX());
                mTelemetry.addData("slow mode: ", slowMode);
                mTelemetry.addData("arm transistion: ", arm.armTransistionStage);
                mTelemetry.addData("teleop mode start: ", arm.teleopModeStart);
                mTelemetry.addData("slides raw pos: ", robot.slidesMotor1.getCurrentPosition());
                mTelemetry.addData("intake position: ", arm.getIntakePosition());
                mTelemetry.addData("arm intake pos: ", arm.getIntakePosition());
                mTelemetry.addData("arm intake grab: ", arm.getIntakeDownMode());
                mTelemetry.addData("arm at position: ", arm.armAtPosition());
                mTelemetry.addData("slides current: ", robot.slidesMotor1.getCurrent(CurrentUnit.AMPS));
            }
            mTelemetry.update();
        }
    }

    public void safeWait(int millis) {
        ElapsedTime time = new ElapsedTime();
        time.reset();

        while (time.time(TimeUnit.MILLISECONDS) >= millis) {
            arm.update(opModeIsActive());
            intake.update(opModeIsActive());
        }
    }
}