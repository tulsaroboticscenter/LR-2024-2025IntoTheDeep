package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.Enums.AnimationType;
import org.firstinspires.ftc.teamcode.Enums.GrabAngle;
import org.firstinspires.ftc.teamcode.Enums.GrabStyle;
import org.firstinspires.ftc.teamcode.Enums.IntakeType;
import org.firstinspires.ftc.teamcode.Enums.WristAngle;
import org.firstinspires.ftc.teamcode.Hardware.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Hardware.Params;
import org.firstinspires.ftc.teamcode.Enums.TeleopMode;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants;

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
    private boolean slowMode = false;
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
    private boolean g2ACooldown = false;
    private PathChain currentPath;
    private boolean runningPath = false;
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
//    private Follower follower;
    private MecanumDrive mecDrive;

    @Override
    public void runOpMode() throws InterruptedException {
//        follower = new Follower(hardwareMap);
        mecDrive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0), false);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for(LynxModule hub : allHubs) {
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

//        mecDrive = new MecanumDrive(robot.motorLF, robot.motorRF, robot.motorLR, robot.motorRR);

//        leftFront = robot.motorLF;
//        leftRear = robot.motorLR;
//        rightRear = robot.motorRR;
//        rightFront = robot.motorRF;

        mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        robot.slidesMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setAutoMode(false);
        arm.setSlidesMultiplier(params.SLIDE_MOTOR_POWER);


        loopTime.reset();
        timer.reset();
        timerSec.reset();

        if (arm.getArmPosition() >= 50) {
            if(arm.getSlidesPosition() < 30) {
                teleopMode = TeleopMode.SPECIMEN_SCORE;
                arm.setArmPositionSpecimen(params.SLIDES_SPECIMEN_POLE_2_START);
            } else {
                teleopMode = TeleopMode.BUCKET_SCORE;
                arm.setBucket(2);
            }
        }

        arm.setTeleopMode(teleopMode);
        arm.update(opModeIsActive());
        arm.setAnimationType(AnimationType.NONE);
        arm.update(opModeIsActive());

        params.TELEOP_START_MODE = TeleopMode.IDLE;

        if (teleopMode == TeleopMode.TOUCH_POLE_AUTO) {
            arm.setParkArmUp(true);
        }

        if (params.INTAKE_TYPE == IntakeType.CLAW) {
            intake.setGrabStyle(GrabStyle.OUTSIDE_GRAB);
            intake.setGrabAngle(GrabAngle.VERTICAL_GRAB);
            intake.outtake();
            intake.update(opModeIsActive());
        }

//        follower.startTeleopDrive();

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

            if (slowMode) {
//                MecanumDrive.driveFieldCentric(-gamepad1.left_stick_x * params.SLOWMODE_XY_MULT, gamepad1.left_stick_y * params.SLOWMODE_XY_MULT, -gamepad1.right_stick_x * params.SLOWMODE_TURN_MULT, robot.imu.getRobotYawPitchRollAngles().getYaw() + params.AUTO_END_HEADING, true);
//                mecDrive.driveFieldCentric(-gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x, robot.imu.getRobotYawPitchRollAngles().getYaw() + params.AUTO_END_HEADING, true);
//                follower.setTeleOpMovementVectors(-gamepad1.left_stick_y * params.SLOWMODE_XY_MULT, -gamepad1.left_stick_x * params.SLOWMODE_XY_MULT, -gamepad1.right_stick_x * params.SLOWMODE_TURN_MULT, false);

                mecDrive.setDrivePowers(new PoseVelocity2d(new Vector2d(rotY, rotX), -gamepad1.right_stick_x));
            } else {
//                mecDrive.driveRobotCentric(-gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x, false);
//                follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
//                mecDrive.driveFieldCentric(-gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x, robot.imu.getRobotYawPitchRollAngles().getYaw() + params.AUTO_END_HEADING, true);

                mecDrive.setDrivePowers(new PoseVelocity2d(new Vector2d(rotY, rotX), -gamepad1.right_stick_x));
            }

            mecDrive.updatePoseEstimate();

            /* *******************INTAKE******************* */

            if (gamepad1.dpad_right && !autoPathingEnabled || (teleopMode == TeleopMode.SPECIMEN_SCORE && gamepad1.left_bumper && arm.getArmTransistionStage() == 3)) {
                teleopMode = TeleopMode.INTAKE;
                grabAngle = GrabAngle.VERTICAL_GRAB;

                intake.setGrabAngle(grabAngle);
                arm.setTeleopMode(teleopMode);
                arm.intakeSpecimen = true;
                intake.outtake();
                intake.update(opModeIsActive());
            }

            if (teleopMode == TeleopMode.INTAKE && arm.intakeSpecimen) {
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
                    if (gamepad1.left_bumper) {
                        lbCooldown = true;

                        if (intake.closed) {
                            arm.intakeUpSpecimen = 2;
                        } else {
                            arm.intakeUpSpecimen = 1;
                        }
                    } else {
                        arm.intakeUpSpecimen = 0;
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
                teleopMode = TeleopMode.INTAKE;
                arm.intakeSpecimen = false;
                arm.setTeleopMode(teleopMode);
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
                        arm.intakeDownMode();
                    } else {
                        arm.intakeUpMode();
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

            if (!gamepad1.left_bumper) lbCooldown = false;

            if (gamepad1.y) {
                teleopMode = TeleopMode.BUCKET_SCORE;
                arm.setTeleopMode(teleopMode);
                intake.looseGrab();
//                arm.setAnimationType(AnimationType.FAST);
                arm.setBucket(2);
            }
            if (gamepad1.b) {
                teleopMode = TeleopMode.BUCKET_SCORE;
                arm.setTeleopMode(teleopMode);
                arm.setBucket(1);
            }

            if (teleopMode == TeleopMode.BUCKET_SCORE) {
                grabAngle = GrabAngle.INVERTED;
                if (arm.getArmTransistionStage() == 3) {
                    if (gamepad1.left_bumper) {
                        wristAngle = WristAngle.IDLE;
                    } else {
                        wristAngle = WristAngle.BUCKET_SCORE;
                    }
                } else {
                    if(arm.getArmTransistionStage() == 1) {
                        wristAngle = WristAngle.IDLE;
                        intake.setWristAngle(wristAngle);
                    } else {
                        wristAngle = WristAngle.CUSTOM;
                        intake.setWristAngle(wristAngle);
                        intake.setCustomWristAngle(params.WRIST_OUTTAKE_DURING_ANIMATION);
                    }
                }

                intake.setWristAngle(wristAngle);
                intake.setGrabAngle(grabAngle);

                if (gamepad1.right_bumper && !rBumperCooldown) {
                    rBumperCooldown = true;

                    intake.closed = !intake.closed;
                }

                if(intake.closed) {
                    if(arm.armTransistionStage == 2) {
                        intake.looseGrab();
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

                arm.setArmTipBucketScore(gamepad1.right_trigger > .1);
            }

            /* *******************IDLE******************* */

            if (gamepad1.x && !xCooldown) {
                xCooldown = true;

                if (teleopMode == TeleopMode.INTAKE) {
                    wristAngle = WristAngle.IDLE;
                } else {
                    wristAngle = WristAngle.DOWN;
                }
                grabAngle = GrabAngle.VERTICAL_GRAB;

                intake.setWristAngle(wristAngle);
                intake.setGrabAngle(grabAngle);

                teleopMode = TeleopMode.IDLE;
                arm.setTeleopMode(teleopMode);
            }

            if (!gamepad1.x) {
                xCooldown = false;
            }

            if (teleopMode == TeleopMode.IDLE) {
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
                    climbPos = params.ARM_CLIMB_DOWN_POS;
                }

                if (climbUp) {
                    if (gamepad1.left_trigger > .1) {
                        climbPos -= 1;
                    } else if (gamepad1.right_trigger > .1) {
                        climbPos += 1;
                    }

                    climbPos = MathFunctions.clamp(climbPos, params.ARM_CLIMB_UP_MIN_POS, params.ARM_CLIMB_UP_MAX_POS);
                }

                arm.setClimbPos(climbPos);
            }

            /* *******************SPECIMEN SCORE******************* */

            if (gamepad1.dpad_left && !autoPathingEnabled || (teleopMode == TeleopMode.INTAKE && intake.closed && arm.intakeSpecimen && gamepad1.left_bumper && arm.getArmTransistionStage() == 3)) {
                teleopMode = TeleopMode.SPECIMEN_SCORE;
                arm.setTeleopMode(teleopMode);
                intake.intake();
                intake.looseGrab();

                wristAngle = WristAngle.SPECIMEN_SCORE_1;
//                grabAngle = GrabAngle.INVERTED;
                intake.setWristAngle(wristAngle);
                intake.setGrabAngle(grabAngle);
                specimenInScorePos = false;
                armPosSpecimen = params.ARM_SPECIMEN_POLE_2_START;
                slidesPosSpecimen = params.SLIDES_SPECIMEN_POLE_2_START;
                arm.setArmPositionSpecimen(armPosSpecimen);
                arm.setSlidesPositionSpecimen(slidesPosSpecimen);
            }

            if (teleopMode == TeleopMode.SPECIMEN_SCORE) {

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

//                    armPosSpecimen = params.ARM_SPECIMEN_POLE_2_SCORE;

                if (gamepad1.right_trigger > .1) {
                    slidesPosSpecimen = params.SLIDES_SPECIMEN_POLE_2_SCORE_CLAW;
                    specimenInScorePos = true;

                    wristAngle = WristAngle.SPECIMEN_SCORE_1;
                    intake.setWristAngle(wristAngle);
                } else if (gamepad1.left_trigger > .1) {
                    slidesPosSpecimen = params.SLIDES_SPECIMEN_POLE_2_START;
                    specimenInScorePos = false;

                    wristAngle = WristAngle.SPECIMEN_SCORE_1;
                    intake.setWristAngle(wristAngle);
                }

                armPosSpecimen = arm.setArmPositionSpecimen(armPosSpecimen);
                slidesPosSpecimen = arm.setSlidesPositionSpecimen(slidesPosSpecimen);
            }

            /* *******************RESET IMU & POSITION******************* */
            if (gamepad1.options) {
//                follower.resetIMU();
//                robot.imu.resetYaw();
//                follower.resetIMU();
//                follower.setPose(new Pose(follower.getPose().getX(), follower.getPose().getY(), 0));
                mecDrive.setPose(new Pose2d(0, 0, 0));
                robot.otos.resetTracking();
                params.AUTO_END_HEADING = 0;
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

            arm.update(opModeIsActive());
            intake.update(opModeIsActive());

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

            if (!gamepad2.a) g2ACooldown = false;


            mTelemetry.addData("slides 1 current: ", robot.slidesMotor1.getCurrent(CurrentUnit.AMPS));
            mTelemetry.addData("slides 2 current: ", robot.slidesMotor2.getCurrent(CurrentUnit.AMPS));
//            mTelemetry.addData("slides 1 power", robot.slidesMotor1.getPower());
//            mTelemetry.addData("slides 2 power", robot.slidesMotor2.getPower());
//            mTelemetry.addData("arm current: ", robot.armMotor.getCurrent(CurrentUnit.AMPS));
            mTelemetry.addData("hertz: ", lastLoopHertz);
            mTelemetry.addData("loop: ", loopTime.time(TimeUnit.MILLISECONDS));
            mTelemetry.addData("arm abs position: ", arm.getArmPosition());
            mTelemetry.addData("slides pos: ", arm.getSlidesPosition());
            mTelemetry.addData("slides out: ", arm.slidesOut);
            mTelemetry.addData("slides mult: ", arm.slidesMultPower);
            mTelemetry.addData("slides power: ", arm.slidesPower);
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
                mTelemetry.addData("auto end heading: ", params.AUTO_END_HEADING);
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