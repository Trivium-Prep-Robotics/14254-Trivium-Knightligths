package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Parts {

    // declaring parts
    // TODO: get rid of and/or add any parts you need
    public static DcMotor FR, FL, BR, BL;
    public static DcMotor leftArm, rightArm, rightExtend, leftExtend;
    public static Servo clawMovement, claw;


    public static IMU imu;
    IMU.Parameters myIMUparameters;

    /**
     * important variables
     */

    public static double armPower;
    public static double extendPower;
    public static double driveMaxSpd = 1;
    public static double driveSlwSpd = 0.5;

    public static double openClaw;
    public static double closeClaw;

    public static double sample;
    public static double specimen;

    /**
     * You can put any other varaibles you'd like to here as well, just make sure they are public static
     * I'd recommend stuff like the following:
     */
    public static double ticksPerRev; // for encoders
    public static double armGearRatio; // if you use gears for example

    public Parts(HardwareMap hardwareMap) {


        // TODO: assign drive train names here
        FR = hardwareMap.get(DcMotor.class, "rightFront");
        FL = hardwareMap.get(DcMotor.class, "leftFront");
        BR = hardwareMap.get(DcMotor.class, "rightRear");
        BL = hardwareMap.get(DcMotor.class, "leftRear");

        // TODO: reverse any motors that go the opposite direction
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);

        // when setPower(0) -> motors brake
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // TODO: assigning other motors used here
        leftArm = hardwareMap.get(DcMotor.class, "leftArm");   //port 2 on expansion
        rightArm = hardwareMap.get(DcMotor.class, "rightArm"); // port 3  on expansion
        leftExtend = hardwareMap.get(DcMotor.class, "leftExtend"); //port 0 on expansion
        rightExtend = hardwareMap.get(DcMotor.class, "rightExtend"); //port 1 on expansion


        // TODO: reverse any motors you need
        // we had two motors for our pivot


        // if you use encoders this is needed
        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // when setPower(0) -> motors brake
        leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // TODO: assigning servos here
        clawMovement = hardwareMap.get(Servo.class, "clawMovement"); //port 0 on control
        claw = hardwareMap.get(Servo.class, "claw");

        // TODO: assign and set up IMU here
        imu = hardwareMap.get(IMU.class, "imu");
        // Reconfiguring IMU orientation
        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        // TODO: this is what you'll need to change this is for the orientation of your control hub
                        RevHubOrientationOnRobot.LogoFacingDirection.UP, // what direction the logo is facing
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT // what direction the USB is facing
                )
        );
        imu.initialize(myIMUparameters);
        imu.resetYaw();
    }
}