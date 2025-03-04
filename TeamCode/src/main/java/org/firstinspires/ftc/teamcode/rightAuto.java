/*
Kunal Trivedi
V.01
This does not work
have not yet tested
should park in the observation zone
may run into wall/ other robot

 */


package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "rightAuto")
public class rightAuto extends LinearOpMode {

    DcMotor rightFront, leftFront, rightBack, leftBack, leftArm, rightArm, rightHook, leftHook;

    public Servo clawMovement, claw, slide;




    @Override
    public void runOpMode() throws InterruptedException {
        rightFront= hardwareMap.get(DcMotor.class, "rightFront"); //port 2 on control
        leftFront = hardwareMap.get(DcMotor.class, "leftFront"); //port 0 on control
        rightBack = hardwareMap.get(DcMotor.class, "rightBack"); //port 3 on control
        leftBack = hardwareMap.get(DcMotor.class, "leftBack"); //port 1 on control
        leftArm = hardwareMap.get(DcMotor.class, "leftArm");   //port 2 on expansion
        rightArm = hardwareMap.get(DcMotor.class, "rightArm"); // port 3  on expansion
        leftHook = hardwareMap.get(DcMotor.class, "leftHook"); //port 0 on expansion
        rightHook = hardwareMap.get(DcMotor.class, "rightHook"); //port 1 on expansion
        clawMovement = hardwareMap.get(Servo.class, "clawMovement"); //port 0 on control
        slide = hardwareMap.get(Servo.class, "slide");         //port 1 on control
        claw = hardwareMap.get(Servo.class, "claw");              //port 1 on expansion


        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);



        waitForStart();

        rightFront.setPower(1);
        leftFront.setPower(-1);
        rightBack.setPower(-1);
        leftBack.setPower(1);

        sleep(3000); //execute for 3 seconds

        rightFront.setPower(0);

        }


} //class





