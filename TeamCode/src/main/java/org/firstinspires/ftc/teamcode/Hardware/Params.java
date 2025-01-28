package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Enums.IntakeType;
import org.firstinspires.ftc.teamcode.Enums.TeleopMode;

@Config
public class Params {
    public static double ARM_INTAKE_POS = 1.5;
    public static double LOOSE_GRAB = 80;
    public static double PIVOT_INVERTED = -90;
    public static double WRIST_SPECIMEN_SCORE_1_ANGLE = 170;
    public static double WRIST_SPECIMEN_SCORE_2_ANGLE = WRIST_SPECIMEN_SCORE_1_ANGLE;
    public static double WRIST_SPECIMEN_INTAKE_ANGLE = 70;
    public static double WRIST_INTAKE_ANGLE = 0;
    public static double WRIST_IDLE_ANGLE = 90;
    public static double WRIST_SCORE_ANGLE = 120;
    public static double ARM_FAST_OFFSET = 5;
    public static double ARM_BUCKET2_SCORE_DEG = 100; //143
    public static double ARM_BUCKET2_SCORE_DEG_AUTO = 110; //143
    public static double ARM_BUCKET1_SCORE_DEG = 70; //143
    public static double ARM_CLIMB_UP_MAX_POS = 105; //143
    public static double ARM_CLIMB_UP_MIN_POS = 105; //143
    public static double ARM_CLIMB_DOWN_POS = 7.5; //143
    public static double ARM_TOUCH_POLE_AUTO_DOWN = 58; //143
    public static double ARM_TOUCH_POLE_AUTO_UP = 80; //143
    public static double ARM_SPECIMEN_POLE_2_SCORE = 95; //143.195
    public static double ARM_SPECIMEN_POLE_2_START = ARM_SPECIMEN_POLE_2_SCORE; //143
    //    public static double ARM_SPECIMEN_POLE_2_MIN_CLAW = 131; //143
    public static double ARM_ZERO = 1.899;
    public static boolean ARM_ENCODER_INVERTED = true;
    public static double ARM_ABS_TICK_PER_DEG = 360/3.22;
    public static double SLIDE_GROUND_POS = 13;
    public static double SLIDES_BUCKET_2_SCORE_LEN_CLAW = 44;
    public static double SLIDES_BUCKET_2_SCORE_LEN_CLAW_AUTO = SLIDES_BUCKET_2_SCORE_LEN_CLAW;
    public static double SLIDES_BUCKET_1_SCORE_LEN = 35;
    public static double SLIDES_BUCKET_1_SCORE_LEN_CLAW = 33;
    public static double SLIDES_TOUCH_POLE_AUTO = 15;
    public static double SLIDES_SPECIMEN_POLE_2_SCORE_CLAW = 25;
    public static double WRIST_OUTTAKE_DURING_ANIMATION = 46;
    public static double SLIDES_SPECIMEN_POLE_2_START = 12;
    public static double SLIDES_SPECIMEN_POLE_2_START_AUTO = SLIDES_SPECIMEN_POLE_2_START;
    public static float SLIDES_TICKS_PER_INCH = 940/44;
    public static double SLIDES_TRANSITION_LEN = 6;
    public static double SLIDES_MAX_POS = 55;
    public static double ARM_INSPECTION_POS = 42;
    public static double SLIDES_MIN_POS = 0;
    public static double SLIDES_OUTTAKE_RETRACT_MODE_LEN = 27;
    public static double SLIDES_OUTTAKE_RETRACT_MODE_LEN_AUTO = 32;
    public static double INTAKE_MAX_POS = 29;
    public static double INTAKE_DEF_POS = 15;
    public static double INTAKE_MIN_POS = 2;
    public static double ARM_IDLE_DEG = 0;
    public static double ARM_ROTATE_SOME_MODE_DEG = 35;
    public static double ARM_INTAKE_MODE_UP_DEG = 11;
    public static double SLIDE_MOTOR_POWER = .75;
    public static double ARM_POWER_DEFAULT = 1;
    public static IntakeType INTAKE_TYPE = IntakeType.CLAW;
    public static double CLAW_OUTSIDE_GRAB_ANGLE = 95;
    public static double CLAW_OUTSIDE_DROP_ANGLE = 0;
    public static double CLAW_OUTSIDE_DROP_ANGLE_SHORT = 25;
    public static double CLAW_INSIDE_GRAB_ANGLE = 5;
    public static double CLAW_INSIDE_DROP_ANGLE = 30;
    public static double PIVOT_VERTICAL_ANG = 90;
    public static double PIVOT_HORIZONTAL_ANG = 0;
    public static double ARM_ERROR_TOLERANCE = 9;
    public static double ARM_ERROR_TOLERANCE_AUTO = ARM_ERROR_TOLERANCE;
    public static double ARM_TELEOP_SPECIMEN_INTAKE = 20;
    public static double ARM_AUTO_SPECIMEN_INTAKE = ARM_TELEOP_SPECIMEN_INTAKE;
    public static double ARM_SPECIMEN_INTAKE_OFFSET = 7;
    public static double SLIDES_AUTO_SPECIMEN_INTAKE = 0;
    public static double SLIDES_TELEOP_SPECIMEN_INTAKE = 0;
    public static double SLIDES_ERROR_TOLERANCE = 7;
    public static double SLIDES_ERROR_TOLERANCE_AUTO = 4;
    public static double AUTO_DEFAULT_SPEED = 1;
    public static double AUTO_END_HEADING = 0;
    public static TeleopMode TELEOP_START_MODE = TeleopMode.IDLE;
    public static double AUTO_SCORE = 0;
    public static double DISTANCE_ONE_SPEC_OFFSET = 5; //less is closer to the wall


    /* Constructor */
    public Params(){
    }

    /* Initialize standard Hardware interfaces */
    public void init() {
    }
}
