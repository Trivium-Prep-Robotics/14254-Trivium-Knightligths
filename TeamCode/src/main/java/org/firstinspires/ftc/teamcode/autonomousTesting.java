package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "testEncoders")
public class autonomousTesting extends LinearOpMode {

    DcMotor rightFront, leftFront, rightBack, leftBack, leftArm, rightArm, rightExtend, leftExtend;

    public Servo clawMovement, claw;


    @Override
    public void runOpMode() throws InterruptedException {
        rightFront = hardwareMap.get(DcMotor.class, "rightFront"); //port 2 on control
        leftFront = hardwareMap.get(DcMotor.class, "leftFront"); //port 0 on control
        rightBack = hardwareMap.get(DcMotor.class, "rightBack"); //port 3 on control
        leftBack = hardwareMap.get(DcMotor.class, "leftBack"); //port 1 on control
        leftArm = hardwareMap.get(DcMotor.class, "leftArm");   //port 2 on expansion
        rightArm = hardwareMap.get(DcMotor.class, "rightArm"); // port 3  on expansion
        leftExtend = hardwareMap.get(DcMotor.class, "leftExtend"); //port 0 on expansion
        rightExtend = hardwareMap.get(DcMotor.class, "rightExtend"); //port 1 on expansion
        clawMovement = hardwareMap.get(Servo.class, "clawMovement"); //port 0 on control
        claw = hardwareMap.get(Servo.class, "claw");              //port 1 on expansion


        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);


        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        waitForStart();

        int verticalArm = -2145; //rightArm encoder value at vertical position
        int specimenHeightExtend = 518; //slide extension to reach specimenHeightExtension
        int specimenPullDown = 1200; //encoder value to pull the specimen onto chamber

        claw.setPosition(0); //makes sure the specimen is gripped tightly


        rightExtend.setTargetPosition(518);
        leftExtend.setTargetPosition(-518);
        //sets the target position of the slides to extend for the specimen


        leftExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftExtend.setPower(1);

        rightExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightExtend.setPower(1);

        //actually runs to the positions set

        while ((rightExtend.isBusy() && leftExtend.isBusy() && opModeIsActive())){}




    }


} //class





