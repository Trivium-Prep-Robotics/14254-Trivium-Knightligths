package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "possiblyScore")
public class specimenAndPark extends LinearOpMode {

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


        rightFront.setPower(0.4);
        leftFront.setPower(0.4);
        rightBack.setPower(0.4);
        leftBack.setPower(0.4);
        //slowly moves forward for 3 seconds
        sleep(3000); //runs above code for 3 seconds

        claw.setPosition(0); //makes sure specimen is help tightly again

        rightArm.setTargetPosition(verticalArm);
        leftArm.setTargetPosition(-verticalArm);
        //sets the target position of the two motors to be the vertical 90 degree angle

        leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftArm.setPower(1);

        rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArm.setPower(1);

        //actually runs to the positions set

        while(rightArm.isBusy() && opModeIsActive() && leftArm.isBusy()) {}

        rightExtend.setTargetPosition(-specimenHeightExtend);
        leftExtend.setTargetPosition(specimenHeightExtend);
        //sets the target position of the slides to extend for the specimen


        leftExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftExtend.setPower(0.6);

        rightExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightExtend.setPower(0.6);

        //actually runs to the positions set

        while ((rightExtend.isBusy() && leftExtend.isBusy() && opModeIsActive())){}

        clawMovement.setPosition(1); //pivots claw to scoring position

        rightExtend.setTargetPosition(-specimenPullDown);
        leftExtend.setTargetPosition(specimenPullDown);
        //sets the target position of the slides just below the chamber bar


        leftExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftExtend.setPower(0.2);

        rightExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightExtend.setPower(0.2);

        //made them slower so they dont break anything

        while ((rightExtend.isBusy() && leftExtend.isBusy() && opModeIsActive())){}


        claw.setPosition(1); //opens claw

        rightExtend.setTargetPosition(0);
        leftExtend.setTargetPosition(0);
        //sets the target position of the slides back to start


        leftExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftExtend.setPower(0.6);

        rightExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightExtend.setPower(0.6);

        //sets the slides back to base starting position

        while ((rightExtend.isBusy() && leftExtend.isBusy() && opModeIsActive())){}


        clawMovement.setPosition(0.25);// sets the claw position back to starting position

        rightArm.setTargetPosition(0);
        leftArm.setTargetPosition(0);
        //sets the target position of the two motors back to the starting position

        leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftArm.setPower(1);

        rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArm.setPower(1);

        while(rightArm.isBusy() && opModeIsActive() && leftArm.isBusy()) {}


        //reobot is fully back to starting position

        rightFront.setPower(-0.4);
        leftFront.setPower(-0.4);
        rightBack.setPower(-0.4);
        leftBack.setPower(-0.4);
        //slowly moves backwards for 3 seconds
        sleep(3000); //runs above code for 3 seconds

        rightFront.setPower(-0.4);
        leftFront.setPower(0.4);
        rightBack.setPower(0.4);
        leftBack.setPower(-0.4);

        sleep(6000);

        //parks in observation zone

        rightFront.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        leftBack.setPower(0);

    }


} //class





