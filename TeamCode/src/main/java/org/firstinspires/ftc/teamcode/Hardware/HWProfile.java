package org.firstinspires.ftc.teamcode.Hardware;

import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

public class HWProfile {
    public IMU imu;
    public DcMotorEx slidesMotor1 = null;
    public DcMotorEx slidesMotor2 = null;
    public DcMotorEx armMotor1 = null;
    public DcMotorEx armMotor2= null;
    public DcMotorEx motorLF = null;
    public DcMotorEx motorLR = null;
    public DcMotorEx motorRF = null;
    public DcMotorEx motorRR = null;
    public AnalogInput armEncoder = null;
    public SimpleServo clawServo = null;
    public SimpleServo diffyLeft = null;
    public SimpleServo diffyRight = null;
    public LessSimpleServo specArmServo = null;
    public LessSimpleServo specClawPivot = null;
    public LessSimpleServo specClawServo = null;
    public IMU.Parameters imuParams;
    public VoltageSensor voltageSensor;
    public Rev2mDistanceSensor distanceOne;
    public SparkFunOTOS otos;


    /* local OpMode members. */
    public HardwareMap hwMap           =  null;

    /* Constructor */
    public HWProfile(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, boolean driveMotors, boolean purePursuit) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

        imuParams = new IMU.Parameters(new RevHubOrientationOnRobot(
                logoFacingDirection, usbFacingDirection));

        imu = hwMap.get(IMU.class, "imu");
        imu.initialize(imuParams);
        imu.resetYaw();

        if(driveMotors) {
            // Define and Initialize Motors
            motorLF = hwMap.get(DcMotorEx.class, "motorLF");
            motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorLF.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
            motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorLF.setPower(0);

            motorLR = hwMap.get(DcMotorEx.class, "motorLR");
            motorLR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorLR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorLR.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
            motorLR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorLR.setPower(0);

            motorRF = hwMap.get(DcMotorEx.class, "motorRF");
            motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorRF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorRF.setDirection(DcMotor.Direction.FORWARD);
            motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorRF.setPower(0);

            motorRR = hwMap.get(DcMotorEx.class, "motorRR");
            motorRR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorRR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorRR.setDirection(DcMotor.Direction.FORWARD);
            motorRR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorRR.setPower(0);
        }

        slidesMotor1 = hwMap.get(DcMotorEx.class, "slidesMotor1");
        slidesMotor1.setDirection(DcMotorEx.Direction.REVERSE);
        slidesMotor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slidesMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        slidesMotor1.setPower(0);               // set motor power

        slidesMotor2 = hwMap.get(DcMotorEx.class, "slidesMotor2");
        slidesMotor2.setDirection(DcMotorEx.Direction.FORWARD);
        slidesMotor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slidesMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        slidesMotor2.setPower(0);               // set motor power
//        slidesMotor1.setMotorDisable();

        armMotor1 = hwMap.get(DcMotorEx.class, "armMotor1");
        armMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armMotor1.setTargetPosition(0);
        armMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        armMotor1.setPower(0);

        armMotor2 = hwMap.get(DcMotorEx.class, "armMotor2");
        armMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armMotor2.setTargetPosition(0);
        armMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        armMotor2.setPower(0);

        armEncoder = hwMap.get(AnalogInput.class, "armEncoder");

        clawServo = new SimpleServo(hwMap, "clawServo", 0, 180, AngleUnit.DEGREES);
        clawServo.setInverted(false);

        diffyLeft = new SimpleServo(hwMap, "diffyLeft", 0, 320, AngleUnit.DEGREES);
        diffyLeft.setInverted(false);

        diffyRight = new SimpleServo(hwMap, "diffyRight", 0, 320, AngleUnit.DEGREES);
        diffyRight.setInverted(true);

        specArmServo = new LessSimpleServo(hwMap, "specArmServo", 0, 180, AngleUnit.DEGREES);
        specArmServo.setInverted(false);

        specClawPivot = new LessSimpleServo(hwMap, "specClawPivot", 0, 320, AngleUnit.DEGREES);
        specClawPivot.setInverted(true);

        specClawServo = new LessSimpleServo(hwMap, "specClawServo", 0, 320, AngleUnit.DEGREES);
        specClawServo.setInverted(true);

        otos = hwMap.get(SparkFunOTOS.class, "otos");
        otos.calibrateImu();
        otos.setAngularUnit(AngleUnit.RADIANS);

        voltageSensor = hwMap.voltageSensor.iterator().next();

        List<LynxModule> modules = hwMap.getAll(LynxModule.class);

        for(LynxModule hub : modules) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
//
//
//        pinpoint = hwMap.get(GoBildaPinpointDriver.class,"pinpoint");
   }
}  // end of HWProfile Class
